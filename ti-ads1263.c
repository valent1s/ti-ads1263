#include <asm/unaligned.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

/* Commands */
#define ADS1263_CMD_NOP         0x00 /* Sends a no operation command to the device */
#define ADS1263_CMD_RESET       0x06 /* Resets the ADC operation and resets the device registers to default */
#define ADS1263_CMD_START1      0x08 /* Start ADC1 conversions */
#define ADS1263_CMD_STOP1       0x0A /* Stop ADC1 conversions */
#define ADS1263_CMD_START2      0x0C /* Start ADC2 conversions */
#define ADS1263_CMD_STOP2       0x0E /* Stop ADC2 conversions */
#define ADS1263_CMD_RDATA1      0x12 /* Read ADC1 conversion data from the respective data holding buffers */
#define ADS1263_CMD_RDATA2      0x14 /* Read ADC2 conversion data from the respective data holding buffers */
#define ADS1263_CMD_SYOCAL1     0x16 /* ADC1 system offset calibration */
#define ADS1263_CMD_SYGCAL1     0x17 /* ADC1 system gain calibration */
#define ADS1263_CMD_SFOCAL1     0x19 /* ADC1 self offset calibration */
#define ADS1263_CMD_SYOCAL2     0x1B /* ADC2 system offset calibration */
#define ADS1263_CMD_SYGCAL2     0x1C /* ADC2 system gain calibration */
#define ADS1263_CMD_SFOCAL2     0x1E /* ADC2 self offset calibration */
#define ADS1263_CMD_RREG        0x20 /* Read the device register data */
#define ADS1263_CMD_WREG        0x40 /* Write the device register data */

/* Registers */
#define ADS1263_REG_ID          0x00
#define ADS1263_REG_POWER       0x01
#define ADS1263_REG_INTERFACE   0x02
#define ADS1263_REG_MODE0       0x03
#define ADS1263_REG_MODE1       0x04
#define ADS1263_REG_MODE2       0x05
#define ADS1263_REG_INPMUX      0x06
#define ADS1263_REG_OFCAL0      0x07
#define ADS1263_REG_OFCAL1      0x08
#define ADS1263_REG_OFCAL2      0x09
#define ADS1263_REG_FSCAL0      0x0A
#define ADS1263_REG_FSCAL1      0x0B
#define ADS1263_REG_FSCAL2      0x0C
#define ADS1263_REG_IDACMUX     0x0D
#define ADS1263_REG_IDACMAG     0x0E
#define ADS1263_REG_REFMUX      0x0F
#define ADS1263_REG_TDACP       0x10
#define ADS1263_REG_TDACN       0x11
#define ADS1263_REG_GPIOCON     0x12
#define ADS1263_REG_GPIODIR     0x13
#define ADS1263_REG_GPIODAT     0x14
#define ADS1263_REG_ADC2CFG     0x15
#define ADS1263_REG_ADC2MUX     0x16
#define ADS1263_REG_ADC2OFC0    0x17
#define ADS1263_REG_ADC2OFC1    0x18
#define ADS1263_REG_ADC2FSC0    0x19
#define ADS1263_REG_ADC2FSC1    0x1A

/* Device Identification Register (ID) */
#define ADS1263_ID_DEV_ID       GENMASK(7, 5)
#define ADS1263_ID_REV_ID       GENMASK(4, 0)

#define ADS1263_ID_ADS1262      0
#define ADS1263_ID_ADS1263      1

/* Power Register (POWER) */
#define ADS1263_POWER_RESET     BIT(4)
#define ADS1263_POWER_VBIAS     BIT(1)
#define ADS1263_POWER_INTREF    BIT(0)

/* Interface Register (INTERFACE) */
#define ADS1263_INTERFACE_TIMEOUT   BIT(3)
#define ADS1263_INTERFACE_STATUS    BIT(2)
#define ADS1263_INTERFACE_CRC       GENMASK(1, 0)

#define ADS1263_CRC_DISABLE         0
#define ADS1263_CRC_CHECKSUM_MODE   1
#define ADS1263_CRC_CRC_MODE        2

/* Mode0 Register (MODE0) */
#define ADS1263_MODE0_REFREV    BIT(7)
#define ADS1263_MODE0_RUNMODE   BIT(6)
#define ADS1263_MODE0_CHOP      GENMASK(5, 4)
#define ADS1263_MODE0_DELAY     GENMASK(3, 0)

#define ADS1263_CHOP_DISABLE    0
#define ADS1263_CHOP_CHOP       1
#define ADS1263_CHOP_IDAC       2
#define ADS1263_CHOP_CHOP_IDAC  3

#define ADS1263_DELAY_0s        0
#define ADS1263_DELAY_8d7us     1
#define ADS1263_DELAY_17us      2
#define ADS1263_DELAY_35us      3
#define ADS1263_DELAY_169us     4
#define ADS1263_DELAY_139us     5
#define ADS1263_DELAY_278us     6
#define ADS1263_DELAY_555us     7
#define ADS1263_DELAY_1d1ms     8
#define ADS1263_DELAY_2d2ms     9
#define ADS1263_DELAY_4d4ms     10
#define ADS1263_DELAY_8d8ms     11

/* Mode1 Register (MODE1) */
#define ADS1263_MODE1_FILTER    GENMASK(7, 5)
#define ADS1263_MODE1_SBADC     BIT(4)
#define ADS1263_MODE1_SBPOL     BIT(3)
#define ADS1263_MODE1_SBMAG     GENMASK(2, 0)

#define ADS1263_SINC1   0
#define ADS1263_SINC2   1
#define ADS1263_SINC3   2
#define ADS1263_SINC4   3
#define ADS1263_FIR     4

/* Mode2 Register (MODE2) */
#define ADS1263_MODE2_BYPASS    BIT(7)
#define ADS1263_MODE2_GAIN      GENMASK(6, 4)
#define ADS1263_MODE2_DR        GENMASK(3, 0)

#define ADS1263_GAIN_1      0
#define ADS1263_GAIN_2      1
#define ADS1263_GAIN_4      2
#define ADS1263_GAIN_8      3
#define ADS1263_GAIN_16     4
#define ADS1263_GAIN_32     5
#define ADS1263_GAIN_64     6
#define ADS1263_GAIN_128    7

#define ADS1263_2d5SPS      0
#define ADS1263_5SPS        1
#define ADS1263_10SPS       2
#define ADS1263_16d6SPS     3
#define ADS1263_20SPS       4
#define ADS1263_50SPS       5
#define ADS1263_60SPS       6
#define ADS1263_100SPS      7
#define ADS1263_400SPS      8
#define ADS1263_1200SPS     9
#define ADS1263_2400SPS     10
#define ADS1263_4800SPS     11
#define ADS1263_7200SPS     12
#define ADS1263_14400SPS    13
#define ADS1263_19200SPS    14
#define ADS1263_38400SPS    15

/* Input Multiplexer Register (INPMUX) */
#define ADS1263_INPMUX_MUXP     GENMASK(7, 4)
#define ADS1263_INPMUX_MUXN     GENMASK(3, 0)

#define ADS1263_AIN0    0
#define ADS1263_AIN1    1
#define ADS1263_AIN2    2
#define ADS1263_AIN3    3
#define ADS1263_AIN4    4
#define ADS1263_AIN5    5
#define ADS1263_AIN6    6
#define ADS1263_AIN7    7
#define ADS1263_AIN8    8
#define ADS1263_AIN9    9
#define ADS1263_AINCOM  10
#define ADS1263_T_SEN   11
#define ADS1263_AVDD    12
#define ADS1263_DVDD    13
#define ADS1263_TDAC    14
#define ADS1263_FLOAT   15

/* IDAC Multiplexer Register (IDACMUX) */
#define ADS1263_IDACMUX_MUX2    GENMASK(7, 4)
#define ADS1263_IDACMUX_MUX1    GENMASK(3, 0)

/* IDAC Magnitude Register (IDACMAG) */
#define ADS1263_IDACMAG_MAG2    GENMASK(7, 4)
#define ADS1263_IDACMAG_MAG1    GENMASK(3, 0)

#define ADS1263_IDAC_OFF        0
#define ADS1263_IDAC_50         1
#define ADS1263_IDAC_100        2
#define ADS1263_IDAC_250        3
#define ADS1263_IDAC_500        4
#define ADS1263_IDAC_750        5
#define ADS1263_IDAC_1000       6
#define ADS1263_IDAC_1500       7
#define ADS1263_IDAC_2000       8
#define ADS1263_IDAC_2500       9
#define ADS1263_IDAC_3000       10

/* Reference Multiplexer Register (REFMUX) */
#define ADS1263_REFMUX_RMUXP    GENMASK(5, 3)
#define ADS1263_REFMUX_RMUXN    GENMASK(2, 0)

#define ADS1263_INTERNAL_REF    0
#define ADS1263_EXTERNAL_AIN0   1
#define ADS1263_EXTERNAL_AIN2   2
#define ADS1263_EXTERNAL_AIN4   3
#define ADS1263_INTERNAL_VAVDD  4

/* TDACP Control Register (TDACP) */
#define ADS1263_TDACP_OUTP          BIT(7)
#define ADS1263_TDACP_MAGP          GENMASK(4, 0)

/* TDACN Control Register (TDACN) */
#define ADS1263_TDACN_OUTN          BIT(7)
#define ADS1263_TDACN_MAGN          GENMASK(4, 0)

#define ADS1263_DAC_VLOT_4_5        0b01001
#define ADS1263_DAC_VLOT_3_5        0b01000
#define ADS1263_DAC_VLOT_3          0b00111
#define ADS1263_DAC_VLOT_2_75       0b00110
#define ADS1263_DAC_VLOT_2_625      0b00101
#define ADS1263_DAC_VLOT_2_5625     0b00100
#define ADS1263_DAC_VLOT_2_53125    0b00011
#define ADS1263_DAC_VLOT_2_515625   0b00010
#define ADS1263_DAC_VLOT_2_5078125  0b00001
#define ADS1263_DAC_VLOT_2_5        0b00000
#define ADS1263_DAC_VLOT_2_4921875  0b10001
#define ADS1263_DAC_VLOT_2_484375   0b10010
#define ADS1263_DAC_VLOT_2_46875    0b10011
#define ADS1263_DAC_VLOT_2_4375     0b10100
#define ADS1263_DAC_VLOT_2_375      0b10101
#define ADS1263_DAC_VLOT_2_25       0b10110
#define ADS1263_DAC_VLOT_2          0b10111
#define ADS1263_DAC_VLOT_1_5        0b11000
#define ADS1263_DAC_VLOT_0_5        0b11001

/* GPIO Connection Register (GPIOCON) */
#define ADS1263_GPIOCON             BIT(x)

/* GPIO Direction Register (GPIODIR) */
#define ADS1263_GPIODIR             BIT(x)

/* GPIO Data Register (GPIODAT) */
#define ADS1263_GPIODAT             BIT(x)

/* ADC2 Configuration Register (ADC2CFG) */
#define ADS1263_ADC2CFG_DR2         GENMASK(7, 6)
#define ADS1263_ADC2CFG_REF2        GENMASK(5, 3)
#define ADS1263_ADC2CFG_GAIN2       GENMASK(2, 0)

#define ADS1263_ADC2_10SPS      0
#define ADS1263_ADC2_100SPS     1
#define ADS1263_ADC2_400SPS     2
#define ADS1263_ADC2_800SPS     3

#define ADS1263_ADC2_GAIN_1     0
#define ADS1263_ADC2_GAIN_2     1
#define ADS1263_ADC2_GAIN_4     2
#define ADS1263_ADC2_GAIN_8     3
#define ADS1263_ADC2_GAIN_16    4
#define ADS1263_ADC2_GAIN_32    5
#define ADS1263_ADC2_GAIN_64    6
#define ADS1263_ADC2_GAIN_128   7

/* ADC2 Input Multiplexer Register (ADC2MUX) */
#define ADS1263_ADC2MUX_MUXP2       GENMASK(7, 4)
#define ADS1263_ADC2MUX_MUXN2       GENMASK(3, 0)

/* Electrical Characteristics */
#define ADS1263_RESOLUTION			32
#define ADS1263_MAX_CLK_MHZ			7.3728 /* Internal oscillator frequency */
#define ADS1263_WAIT_RESET_CYCLES	4
#define ADS1263_WAIT_CSSC_NS		50
#define ADS1263_WAIT_SCCS_NS		40

struct ads1263_channel_config {
	unsigned int pga_gain;
};

struct ads1263 {
	struct spi_device *spi;

	struct ads1263_channel_config *channel_config;

	struct completion completion;

	struct gpio_desc  *reset_pin;
};

#define ADC1263_VOLTAGE_CHANNEL(chan1, chan2, si)			\
		{													\
			.type = IIO_VOLTAGE,							\
			.indexed = 1,									\
			.channel = (chan1),								\
			.channel2 = (chan2),							\
			.differential = (chan2) < ADS1263_AINCOM,		\
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |	\
				BIT(IIO_CHAN_INFO_HARDWAREGAIN),			\
			.scan_index = (si),								\
			.scan_type = {									\
				.sign = 's',								\
				.realbits = 24,								\
				.storagebits = ADS1263_RESOLUTION,			\
				.shift = 8,									\
                .endianness = IIO_BE,                       \
			},												\
		}

static void ads1263_reset(struct ads1263 *adc)
{
	gpiod_set_value(adc->reset_pin, 0);
	udelay(ADS1263_WAIT_RESET_CYCLES / ADS1263_MAX_CLK_MHZ);
	gpiod_set_value(adc->reset_pin, 1);
}

static int ads1263_write_cmd(struct ads1263 *adc, u8 cmd)
{
	int ret;

	ret = spi_write(adc->spi, &cmd, 1);
	if (ret)
		dev_err(&adc->spi->dev, "Write cmd(%02x) failed\n", cmd);

    return ret;
}

static int ads1263_read_reg(struct ads1263 *adc, u8 reg)
{
    const u8 txbuf[] = { ADS1263_CMD_RREG | reg, 0 };
	u8 result;
    int ret;

	ret = spi_write_then_read(adc->spi, txbuf, ARRAY_SIZE(txbuf), &result, 1);
	if (ret) {
		dev_err(&adc->spi->dev, "Read register failed\n");
    	return ret;
	}

	return result;
}

static int ads1263_write_reg(struct ads1263 *adc, u8 reg, u8 data)
{
    const u8 txbuf[] = { ADS1263_CMD_WREG | reg, 0, data };
	int ret;

	ret = spi_write(adc->spi, txbuf, ARRAY_SIZE(txbuf));
	if (ret)
		dev_err(&adc->spi->dev, "Write register failed\n");

    return ret;
}

static size_t ads1263_check_sum(u32 val, u8 byt)
{
    u8 sum = 0x9B;

    while (val) {
        sum += val & 0xFF;  // only add the lower values
        val >>= 8;          // shift down
    }
    
    return sum ^ byt;       // if equal, this will be 0
}

static int ads1263_setup(struct iio_dev *indio_dev)
{
    struct ads1263 *adc = iio_priv(indio_dev);
    int ret;
    
    const u8 MODE2 = FIELD_PREP(ADS1263_MODE2_DR, ADS1263_1200SPS);
    const u8 REFMUX = FIELD_PREP(ADS1263_REFMUX_RMUXN, ADS1263_INTERNAL_VAVDD) |
        FIELD_PREP(ADS1263_REFMUX_RMUXP, ADS1263_INTERNAL_VAVDD);
    const u8 MODE1 = FIELD_PREP(ADS1263_MODE1_FILTER, ADS1263_SINC4);
    struct ads1263_channel_config *channel_config;

    ads1263_reset(adc);

    ret = ads1263_write_reg(adc, ADS1263_REG_MODE2, MODE2);
    if (ads1263_read_reg(adc, ADS1263_REG_MODE2) != MODE2) {
        dev_err(&adc->spi->dev, "REG_MODE2 unsuccess\r\n");
        return ret;
    }

    ret = ads1263_write_reg(adc, ADS1263_REG_REFMUX, REFMUX);
    if (ads1263_read_reg(adc, ADS1263_REG_REFMUX) != REFMUX) {
        dev_err(&adc->spi->dev, "REG_REFMUX unsuccess\r\n");
        return ret;
    }

    ret = ads1263_write_reg(adc, ADS1263_REG_MODE1, MODE1);
    if (ads1263_read_reg(adc, ADS1263_REG_MODE1) != MODE1) {
        dev_err(&adc->spi->dev, "REG_MODE1 unsuccess\r\n");
        return ret;
    }

    ret = ads1263_write_cmd(adc, ADS1263_CMD_START1);
	if (ret)
		return ret;

    channel_config = devm_kcalloc(&adc->spi->dev, indio_dev->num_channels, sizeof(*channel_config), GFP_KERNEL);
	if (!channel_config)
        return -ENOMEM;

    adc->channel_config = channel_config;

    return 0;
}

static inline void ads1263_set_channel(struct ads1263 *adc, int chan1, int chan2)
{   
    ads1263_write_reg(adc, ADS1263_REG_INPMUX,
        FIELD_PREP(ADS1263_INPMUX_MUXP, chan1) |
        FIELD_PREP(ADS1263_INPMUX_MUXN, chan2));
}

static u32 ads1263_read_adc1_data(struct ads1263 *adc)
{
    u8 buf[6];
    u32 read;

    spi_read(adc->spi, buf, ARRAY_SIZE(buf));

    read = sign_extend32(get_unaligned_be32(&buf[1]), ADS1263_RESOLUTION - 1);

    return ads1263_check_sum(read, buf[5]) == 0 ? read : 0;
}

static int ads1263_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int *val, int *val2, long mask) {
	struct ads1263 *adc = iio_priv(indio_dev);
	int ret;

    const u8 MODE2 = FIELD_PREP(ADS1263_MODE2_GAIN, adc->channel_config[chan->channel].pga_gain) |
        FIELD_PREP(ADS1263_MODE2_DR, ADS1263_400SPS);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
        ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

        ads1263_write_reg(adc, ADS1263_REG_MODE2, MODE2);
		ads1263_set_channel(adc, chan->channel, chan->channel2);

		reinit_completion(&adc->completion);
		ret = wait_for_completion_timeout(&adc->completion, msecs_to_jiffies(1000));
		if (!ret)
			return -ETIMEDOUT;

		*val = ads1263_read_adc1_data(adc);

        iio_device_release_direct_mode(indio_dev);
		
		return IIO_VAL_INT;
    case IIO_CHAN_INFO_HARDWAREGAIN:
        *val = adc->channel_config[chan->channel].pga_gain;
        return IIO_VAL_INT;
	}

	return -EINVAL;
}

static int ads1263_write_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int val, int val2, long mask) {
	struct ads1263 *adc = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		adc->channel_config[chan->channel].pga_gain = val;
		return 0;
	}

	return -EINVAL;
}

static const struct iio_chan_spec ads1263_channels[] = {
	ADC1263_VOLTAGE_CHANNEL(ADS1263_AIN0, ADS1263_AINCOM, 0),
	ADC1263_VOLTAGE_CHANNEL(ADS1263_AIN1, ADS1263_AINCOM, 1),
	ADC1263_VOLTAGE_CHANNEL(ADS1263_AIN2, ADS1263_AINCOM, 2),
	ADC1263_VOLTAGE_CHANNEL(ADS1263_AIN3, ADS1263_AINCOM, 3),
	ADC1263_VOLTAGE_CHANNEL(ADS1263_AIN4, ADS1263_AIN5,   4)
};

// static IIO_CONST_ATTR(hardwaregain_available,
// 	"0 1 2 3 4 5 6 7");

// static struct attribute *ads1263_attributes[] = {
// 	&iio_const_attr_hardwaregain_available.dev_attr.attr,
// 	NULL,
// };

// static const struct attribute_group ads1263_attrs_group = {
// 	.attrs = ads1263_attributes,
// };

static const struct iio_info ads1263_info = {
	.read_raw = ads1263_read_raw,
    .write_raw = ads1263_write_raw,
};

static irqreturn_t ads1263_trigger_handler(int irq, void *private)
{
    struct iio_poll_func *pf = private;
	struct iio_dev *indio_dev = pf->indio_dev;
    const u32 i[indio_dev->num_channels];

    iio_push_to_buffers(indio_dev, i);

// out:
	// iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static irqreturn_t ads1263_drdy_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct ads1263 *adc = iio_priv(indio_dev);

	complete(&adc->completion);

	return IRQ_HANDLED;
}

static int ads1263_probe(struct spi_device *spi)
{
	struct ads1263 *adc;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	adc->spi = spi;
	init_completion(&adc->completion);
	adc->reset_pin = devm_gpiod_get(&spi->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(adc->reset_pin))
		return PTR_ERR(adc->reset_pin);
	
	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = "ads1263";
	indio_dev->info = &ads1263_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ads1263_channels;
	indio_dev->num_channels = ARRAY_SIZE(ads1263_channels);

    if (spi->irq) {
		ret = devm_request_irq(&spi->dev, spi->irq, ads1263_drdy_handler,
                IRQF_TRIGGER_FALLING, indio_dev->name, indio_dev);
		if (ret)
			return dev_err_probe(&spi->dev, ret,
					     "request irq failed\n");
	} else {
		dev_err(&spi->dev, "data ready IRQ missing\n");
		return -ENODEV;
	}

    ret = devm_iio_triggered_buffer_setup(&spi->dev, indio_dev,
		NULL, &ads1263_trigger_handler, NULL);
	if (ret) {
		dev_err(&spi->dev, "failed to setup IIO buffer\n");
		return ret;
	}

	spi->cs_setup.value = ADS1263_WAIT_CSSC_NS;
	spi->cs_setup.unit = SPI_DELAY_UNIT_NSECS;

	spi->cs_hold.value = ADS1263_WAIT_SCCS_NS;
	spi->cs_hold.unit = SPI_DELAY_UNIT_NSECS;
    
    ret = ads1263_setup(indio_dev);
    if (ret) {
        dev_err(&spi->dev, "setup failed\r\n");
        return ret;
    }

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id ads1263_of_match[] = {
	{
		.compatible	= "ti,ads1262",
		.compatible	= "ti,ads1263",
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ads1263_of_match);

static struct spi_driver ads1263_driver = {
	.driver = {
		.name = "ads1263",
		.of_match_table = ads1263_of_match
	},
	.probe = ads1263_probe,
};
module_spi_driver(ads1263_driver);

MODULE_AUTHOR("Fedorinov Valentine <fedorinovvalek@gmail.com>");
MODULE_DESCRIPTION("ADS1263 SPI driver");
MODULE_LICENSE("GPL");