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
#define ADS1263_CMD_NOP			0x00 /* Sends a no operation command to the device */
#define ADS1263_CMD_RESET		0x06 /* Resets the ADC operation and resets the device registers to default */
#define ADS1263_CMD_START1		0x08 /* Start ADC1 conversions */
#define ADS1263_CMD_STOP1		0x0A /* Stop ADC1 conversions */
#define ADS1263_CMD_START2		0x0C /* Start ADC2 conversions */
#define ADS1263_CMD_STOP2		0x0E /* Stop ADC2 conversions */
#define ADS1263_CMD_RDATA1		0x12 /* Read ADC1 conversion data from the respective data holding buffers */
#define ADS1263_CMD_RDATA2		0x14 /* Read ADC2 conversion data from the respective data holding buffers */
#define ADS1263_CMD_SYOCAL1		0x16 /* ADC1 system offset calibration */
#define ADS1263_CMD_SYGCAL1		0x17 /* ADC1 system gain calibration */
#define ADS1263_CMD_SFOCAL1		0x19 /* ADC1 self offset calibration */
#define ADS1263_CMD_SYOCAL2		0x1B /* ADC2 system offset calibration */
#define ADS1263_CMD_SYGCAL2		0x1C /* ADC2 system gain calibration */
#define ADS1263_CMD_SFOCAL2		0x1E /* ADC2 self offset calibration */
#define ADS1263_CMD_RREG(x)		0x20 | FIELD_PREP(GENMASK(4, 0), x) /* Read the device register data */
#define ADS1263_CMD_WREG(x)		0x40 | FIELD_PREP(GENMASK(4, 0), x) /* Write the device register data */

/* Registers */
enum ads1263_reg {
	ADS1263_REG_ID,
	ADS1263_REG_POWER,
	ADS1263_REG_INTERFACE,
	ADS1263_REG_MODE0,
	ADS1263_REG_MODE1,
	ADS1263_REG_MODE2,
	ADS1263_REG_INPMUX,
	ADS1263_REG_OFCAL0,
	ADS1263_REG_OFCAL1,
	ADS1263_REG_OFCAL2,
	ADS1263_REG_FSCAL0,
	ADS1263_REG_FSCAL1,
	ADS1263_REG_FSCAL2,
	ADS1263_REG_IDACMUX,
	ADS1263_REG_IDACMAG,
	ADS1263_REG_REFMUX,
	ADS1263_REG_TDACP,
	ADS1263_REG_TDACN,
	ADS1263_REG_GPIOCON,
	ADS1263_REG_GPIODIR,
	ADS1263_REG_GPIODAT,
	ADS1263_REG_ADC2CFG,
	ADS1263_REG_ADC2MUX,
	ADS1263_REG_ADC2OFC0,
	ADS1263_REG_ADC2OFC1,
	ADS1263_REG_ADC2FSC0,
	ADS1263_REG_ADC2FSC1,
};

/* Device Identification Register (ID) */
#define ADS1263_ID_DEV_ID(x)		FIELD_PREP(GENMASK(7, 5), x)
#define ADS1263_ID_REV_ID(x)		FIELD_PREP(GENMASK(4, 0), x)

enum ads1263_id {
	ADS1263_ID_ADS1262,
	ADS1263_ID_ADS1263,
};

/* Power Register (POWER) */
#define ADS1263_POWER_RESET(x)		FIELD_PREP(BIT(4), x)
#define ADS1263_POWER_VBIAS(x)		FIELD_PREP(BIT(1), x)
#define ADS1263_POWER_INTREF(x)		FIELD_PREP(BIT(0), x)

/* Interface Register (INTERFACE) */
#define ADS1263_INTERFACE_TIMEOUT(x)	FIELD_PREP(BIT(3), x)
#define ADS1263_INTERFACE_STATUS(x)		FIELD_PREP(BIT(2), x)
#define ADS1263_INTERFACE_CRC(x)		FIELD_PREP(GENMASK(1, 0), x)

enum ads1263_crc {
	ADS1263_CRC_DISABLE,
	ADS1263_CRC_CHECKSUM_MODE,
	ADS1263_CRC_CRC_MODE,
};

/* Mode0 Register (MODE0) */
#define ADS1263_MODE0_REFREV(x)		FIELD_PREP(BIT(7), x)
#define ADS1263_MODE0_RUNMODE(x)	FIELD_PREP(BIT(6), x)
#define ADS1263_MODE0_CHOP(x)		FIELD_PREP(GENMASK(5, 4), x)
#define ADS1263_MODE0_DELAY(x)		FIELD_PREP(GENMASK(3, 0), x)

enum ads1263_chop {
	ADS1263_CHOP_DISABLE,
	ADS1263_CHOP_CHOP,
	ADS1263_CHOP_IDAC,
	ADS1263_CHOP_CHOP_IDAC,
};

enum ads1263_delay {
	ADS1263_DELAY_0s,
	ADS1263_DELAY_8d7us,
	ADS1263_DELAY_17us,
	ADS1263_DELAY_35us,
	ADS1263_DELAY_169us,
	ADS1263_DELAY_139us,
	ADS1263_DELAY_278us,
	ADS1263_DELAY_555us,
	ADS1263_DELAY_1d1ms,
	ADS1263_DELAY_2d2ms,
	ADS1263_DELAY_4d4ms,
	ADS1263_DELAY_8d8ms,
};

/* Mode1 Register (MODE1) */
#define ADS1263_MODE1_FILTER(x)		FIELD_PREP(GENMASK(7, 5), x)
#define ADS1263_MODE1_SBADC(x)		FIELD_PREP(BIT(4), x)
#define ADS1263_MODE1_SBPOL(x)		FIELD_PREP(BIT(3), x)
#define ADS1263_MODE1_SBMAG(x)		FIELD_PREP(GENMASK(2, 0), x)

enum ads1263_filter {
	ADS1263_SINC1,
	ADS1263_SINC2,
	ADS1263_SINC3,
	ADS1263_SINC4,
	ADS1263_FIR,
};

/* Mode2 Register (MODE2) */
#define ADS1263_MODE2_BYPASS(x)	FIELD_PREP(BIT(7), x)
#define ADS1263_MODE2_GAIN(x)	FIELD_PREP(GENMASK(6, 4), x)
#define ADS1263_MODE2_DR(x)		FIELD_PREP(GENMASK(3, 0), x)

enum ads1263_gain {
	ADS1263_GAIN_1,
	ADS1263_GAIN_2,
	ADS1263_GAIN_4,
	ADS1263_GAIN_8,
	ADS1263_GAIN_16,
	ADS1263_GAIN_32,
	ADS1263_GAIN_64,
	ADS1263_GAIN_128,
};

enum ads1263_dr {
	ADS1263_2d5SPS,
	ADS1263_5SPS,
	ADS1263_10SPS,
	ADS1263_16d6SPS,
	ADS1263_20SPS,
	ADS1263_50SPS,
	ADS1263_60SPS,
	ADS1263_100SPS,
	ADS1263_400SPS,
	ADS1263_1200SPS,
	ADS1263_2400SPS,
	ADS1263_4800SPS,
	ADS1263_7200SPS,
	ADS1263_14400SPS,
	ADS1263_19200SPS,
	ADS1263_38400SPS,
};

/* Input Multiplexer Register (INPMUX) */
#define ADS1263_INPMUX_MUXP(x)	FIELD_PREP(GENMASK(7, 4), x)
#define ADS1263_INPMUX_MUXN(x)	FIELD_PREP(GENMASK(3, 0), x)

enum ads1263_inpmux {
	ADS1263_AIN0,
	ADS1263_AIN1,
	ADS1263_AIN2,
	ADS1263_AIN3,
	ADS1263_AIN4,
	ADS1263_AIN5,
	ADS1263_AIN6,
	ADS1263_AIN7,
	ADS1263_AIN8,
	ADS1263_AIN9,
	ADS1263_AINCOM,
	ADS1263_T_SEN,
	ADS1263_AVDD,
	ADS1263_DVDD,
	ADS1263_TDAC,
	ADS1263_FLOAT,
};

/* IDAC Multiplexer Register (IDACMUX) */
#define ADS1263_IDACMUX_MUX2(x)		FIELD_PREP(GENMASK(7, 4), x)
#define ADS1263_IDACMUX_MUX1(x)		FIELD_PREP(GENMASK(3, 0), x)

/* IDAC Magnitude Register (IDACMAG) */
#define ADS1263_IDACMAG_MAG2(x)		FIELD_PREP(GENMASK(7, 4), x)
#define ADS1263_IDACMAG_MAG1(x)		FIELD_PREP(GENMASK(3, 0), x)

enum ads1263_idac {
	ADS1263_IDAC_OFF,
	ADS1263_IDAC_50,
	ADS1263_IDAC_100,
	ADS1263_IDAC_250,
	ADS1263_IDAC_500,
	ADS1263_IDAC_750,
	ADS1263_IDAC_1000,
	ADS1263_IDAC_1500,
	ADS1263_IDAC_2000,
	ADS1263_IDAC_2500,
	ADS1263_IDAC_3000,
};

/* Reference Multiplexer Register (REFMUX) */
#define ADS1263_REFMUX_RMUXP(x)		FIELD_PREP(GENMASK(5, 3), x)
#define ADS1263_REFMUX_RMUXN(x)		FIELD_PREP(GENMASK(2, 0), x)

enum ads1263_refmux {
	ADS1263_INTERNAL_REF,
	ADS1263_EXTERNAL_AIN0,
	ADS1263_EXTERNAL_AIN2,
	ADS1263_EXTERNAL_AIN4,
	ADS1263_INTERNAL_VAVDD,
};

/* TDACP Control Register (TDACP) */
#define ADS1263_TDACP_OUTP(x)		FIELD_PREP(BIT(7), x)
#define ADS1263_TDACP_MAGP(x)		FIELD_PREP(GENMASK(4, 0), x)

/* TDACN Control Register (TDACN) */
#define ADS1263_TDACN_OUTN(x)		FIELD_PREP(BIT(7), x)
#define ADS1263_TDACN_MAGN(x)		FIELD_PREP(GENMASK(4, 0), x)

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
#define ADS1263_ADC2CFG_DR2(x)		FIELD_PREP(GENMASK(7, 6), x)
#define ADS1263_ADC2CFG_REF2(x)		FIELD_PREP(GENMASK(5, 3), x)
#define ADS1263_ADC2CFG_GAIN2(x)	FIELD_PREP(GENMASK(2, 0), x)

enum ads1263_adc2_dr {
	ADS1263_ADC2_10SPS,
	ADS1263_ADC2_100SPS,
	ADS1263_ADC2_400SPS,
	ADS1263_ADC2_800SPS,

};

enum ads1263_adc2_gain {
	ADS1263_ADC2_GAIN_1,
	ADS1263_ADC2_GAIN_2,
	ADS1263_ADC2_GAIN_4,
	ADS1263_ADC2_GAIN_8,
	ADS1263_ADC2_GAIN_16,
	ADS1263_ADC2_GAIN_32,
	ADS1263_ADC2_GAIN_64,
	ADS1263_ADC2_GAIN_128,
};

/* ADC2 Input Multiplexer Register (ADC2MUX) */
#define ADS1263_ADC2MUX_MUXP2(x)	FIELD_PREP(GENMASK(7, 4), x)
#define ADS1263_ADC2MUX_MUXN2(x)	FIELD_PREP(GENMASK(3, 0), x)

/* Electrical Characteristics */
#define ADS1263_RESOLUTION			32
#define ADS1263_MAX_CLK_MHZ			7.3728 /* Internal oscillator frequency */
#define ADS1263_WAIT_RESET_CYCLES	4
#define ADS1263_WAIT_CSSC_NS		50
#define ADS1263_WAIT_SCCS_NS		40
#define ADS1263_MAX_SETTLING_TIME_MS	1000

struct ads1263_channel_config {
	unsigned int pga_gain;
};

struct ads1263 {
	struct spi_device *spi;
	struct iio_trigger	*trig;

	struct ads1263_channel_config *channel_config;

	struct completion completion;

	struct gpio_desc  *reset_pin;
	
	u8 rx_buf[1 + 4 + 1];
	u32 data[10];

	unsigned data_rate;
};

static const float ads1263_data_rates[] = {
	[ADS1263_2d5SPS] = 2.5,
	[ADS1263_5SPS] = 5,
	[ADS1263_10SPS] = 10,
	[ADS1263_16d6SPS] = 16.6,
	[ADS1263_20SPS] = 20,
	[ADS1263_50SPS] = 50,
	[ADS1263_60SPS] = 60,
	[ADS1263_100SPS] = 100,
	[ADS1263_400SPS] = 400,
	[ADS1263_1200SPS] = 1200,
	[ADS1263_2400SPS] = 2400,
	[ADS1263_4800SPS] = 4800,
	[ADS1263_7200SPS] = 7200,
	[ADS1263_14400SPS] = 14400,
	[ADS1263_19200SPS] = 19200,
	[ADS1263_38400SPS] = 38400,
};

#define ADC1263_VOLTAGE_CHANNEL(chan1, chan2, si)					\
		{															\
			.type = IIO_VOLTAGE,									\
			.indexed = 1,											\
			.channel = (chan1),										\
			.channel2 = (chan2),									\
			.differential = (chan2) < ADS1263_AINCOM,				\
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
								  BIT(IIO_CHAN_INFO_HARDWAREGAIN) |	\
								  BIT(IIO_CHAN_INFO_SAMP_FREQ),		\
			.scan_index = (si),										\
			.scan_type = {											\
				.sign = 's',										\
				.realbits = 24,										\
				.storagebits = ADS1263_RESOLUTION,					\
				.shift = 8,											\
                .endianness = IIO_BE,                       		\
			},														\
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
    const u8 txbuf[] = { ADS1263_CMD_RREG(reg), 0 };
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
    const u8 txbuf[] = { ADS1263_CMD_WREG(reg), 0, data };
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

static int ads1263_read_adc1_data(struct ads1263 *adc)
{
    u8 buf[6];
    u32 read;

    spi_read(adc->spi, buf, ARRAY_SIZE(buf));

    read = get_unaligned_be32(&buf[1]);

    return ads1263_check_sum(read, buf[5]) == 0 ? read : 0;
}

static int ads1263_setup(struct iio_dev *indio_dev)
{
    struct ads1263 *adc = iio_priv(indio_dev);
    int ret;
    
    const u8 MODE2 = ADS1263_MODE2_DR(ADS1263_1200SPS);
    const u8 REFMUX = ADS1263_REFMUX_RMUXN(ADS1263_INTERNAL_VAVDD) | ADS1263_REFMUX_RMUXP(ADS1263_INTERNAL_VAVDD);
    const u8 MODE1 = ADS1263_MODE1_FILTER(ADS1263_SINC4);
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

    channel_config = devm_kcalloc(&adc->spi->dev, indio_dev->num_channels, sizeof(*channel_config), GFP_KERNEL);
	if (!channel_config)
        return -ENOMEM;

    adc->channel_config = channel_config;

    return 0;
}

static int ads1263_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int *val, int *val2, long mask) {
	struct ads1263 *adc = iio_priv(indio_dev);
	int ret;

    const u8 MODE2 = ADS1263_MODE2_GAIN(adc->channel_config[chan->channel].pga_gain) | ADS1263_MODE2_DR(ADS1263_100SPS);
	const u8 INPMUX = ADS1263_INPMUX_MUXP(chan->channel) | ADS1263_INPMUX_MUXN(chan->channel2);
	const u8 txbuf[] = { ADS1263_CMD_WREG(ADS1263_REG_MODE2), 1, MODE2, INPMUX };

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
        ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		ret = spi_write(adc->spi, txbuf, ARRAY_SIZE(txbuf));
		if (ret)
			dev_err(&adc->spi->dev, "Write register failed\n");

		reinit_completion(&adc->completion);

		ads1263_write_cmd(adc, ADS1263_CMD_START1);

		ret = wait_for_completion_timeout(&adc->completion, msecs_to_jiffies(ADS1263_MAX_SETTLING_TIME_MS));
		if (!ret)
			return -ETIMEDOUT;

		*val = ads1263_read_adc1_data(adc);

		ads1263_write_cmd(adc, ADS1263_CMD_STOP1);

        iio_device_release_direct_mode(indio_dev);
		
		return IIO_VAL_INT;
    case IIO_CHAN_INFO_HARDWAREGAIN:
        *val = adc->channel_config[chan->channel].pga_gain;
        return IIO_VAL_INT;

	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = adc->data_rate;
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
	case IIO_CHAN_INFO_SAMP_FREQ:
		adc->data_rate = val;
		return IIO_VAL_INT;
	}

	return -EINVAL;
}

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

static int ads1263_set_trigger_state(struct iio_trigger *trig, bool state)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct ads1263 *adc = iio_priv(indio_dev);

	return ads1263_write_cmd(adc, state ? ADS1263_CMD_START1 : ADS1263_CMD_STOP1);
}

static const struct iio_trigger_ops ads1263_trigger_ops = {
	.set_trigger_state = &ads1263_set_trigger_state,
};

static irqreturn_t ads1263_trigger_handler(int irq, void *private)
{
    struct iio_poll_func *pf = private;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ads1263 *adc = iio_priv(indio_dev);
	int ret;
	int scan_index = find_first_bit(indio_dev->active_scan_mask, indio_dev->masklength);
	int next_scan_index;
	int i = 0;
	u32 read;

	if (!iio_trigger_using_own(indio_dev))
		ads1263_write_cmd(adc, ADS1263_CMD_START1);

	const struct iio_chan_spec *scan_chan = &indio_dev->channels[scan_index];

	const u8 MODE2 = ADS1263_MODE2_GAIN(adc->channel_config[scan_chan->channel].pga_gain) |
		ADS1263_MODE2_DR(ADS1263_100SPS);

	const u8 txbuf[] = { ADS1263_CMD_WREG(ADS1263_REG_MODE2), 1, MODE2, ADS1263_INPMUX_MUXP(scan_chan->channel) |
		ADS1263_INPMUX_MUXN(scan_chan->channel2) };

	ret = spi_write(adc->spi, txbuf, ARRAY_SIZE(txbuf));
	if (ret)
		dev_err(&adc->spi->dev, "Write register failed\n");

	for_each_set_bit(scan_index, indio_dev->active_scan_mask, indio_dev->masklength) {

		next_scan_index = find_next_bit(indio_dev->active_scan_mask, indio_dev->masklength, scan_index + 1);
		if (next_scan_index >= indio_dev->masklength)
			next_scan_index = find_first_bit(indio_dev->active_scan_mask, indio_dev->masklength);
		
		const struct iio_chan_spec *next_scan_chan = &indio_dev->channels[next_scan_index];

		const u8 NEXT_MODE2 = ADS1263_MODE2_GAIN(adc->channel_config[next_scan_chan->channel].pga_gain) |
			ADS1263_MODE2_DR(ADS1263_100SPS);

		const u8 next_txbuf[] = { ADS1263_CMD_WREG(ADS1263_REG_MODE2), 1, NEXT_MODE2, ADS1263_INPMUX_MUXP(next_scan_chan->channel) |
			ADS1263_INPMUX_MUXN(next_scan_chan->channel2) };
		
		struct spi_transfer transfer[] = {
			{
				.rx_buf = adc->rx_buf,
				.len = ARRAY_SIZE(adc->rx_buf),
			},
			{
				.tx_buf = next_txbuf,
				.len = ARRAY_SIZE(next_txbuf),
			},
		};

		reinit_completion(&adc->completion);

		ret = wait_for_completion_timeout(&adc->completion, msecs_to_jiffies(ADS1263_MAX_SETTLING_TIME_MS));
		if (!ret)
			return -ETIMEDOUT;

		ret = spi_sync_transfer(adc->spi, transfer, ARRAY_SIZE(transfer));
		if (ret)
			dev_err(&adc->spi->dev, "Read data failed\n");

		read = get_unaligned_be32(&adc->rx_buf[1]);
		adc->data[i] = ads1263_check_sum(read, adc->rx_buf[5]) == 0 ? read : 0;
		
		printk("chan %i, next %i, lenght %i, array size %i: %li, %i",
			scan_index,
			next_scan_index,
			indio_dev->masklength,
			ARRAY_SIZE(adc->rx_buf),
			ads1263_check_sum(read, adc->rx_buf[5]) == 0 ? adc->data[i] : 0,
			adc->rx_buf[0]
		);

		i++;
	}

	if (!iio_trigger_using_own(indio_dev))
		ads1263_write_cmd(adc, ADS1263_CMD_STOP1);

    iio_push_to_buffers(indio_dev, adc->data);

	iio_trigger_notify_done(indio_dev->trig);

	printk("done!");

	return IRQ_HANDLED;
}

static irqreturn_t ads1263_interrupt(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct ads1263 *adc = iio_priv(indio_dev);

	if (iio_buffer_enabled(indio_dev) && iio_trigger_using_own(indio_dev))
		iio_trigger_poll(adc->trig);
	
	complete(&adc->completion);

	return IRQ_HANDLED;
}

static const struct iio_chan_spec ads1263_channels[] = {
	ADC1263_VOLTAGE_CHANNEL(ADS1263_AIN0, ADS1263_AINCOM, 0),
	ADC1263_VOLTAGE_CHANNEL(ADS1263_AIN1, ADS1263_AINCOM, 1),
	ADC1263_VOLTAGE_CHANNEL(ADS1263_AIN2, ADS1263_AINCOM, 2),
	ADC1263_VOLTAGE_CHANNEL(ADS1263_AIN3, ADS1263_AINCOM, 3),
	ADC1263_VOLTAGE_CHANNEL(ADS1263_AIN4, ADS1263_AIN5,   4)
};

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
		ret = devm_request_irq(&spi->dev, spi->irq, ads1263_interrupt,
                IRQF_TRIGGER_FALLING, indio_dev->name, indio_dev);
		if (ret)
			return dev_err_probe(&spi->dev, ret,
					     "request irq failed\n");
	} else {
		dev_err(&spi->dev, "data ready IRQ missing\n");
		return -ENODEV;
	}

	adc->trig = devm_iio_trigger_alloc(&spi->dev, "%s-dev%d",
		indio_dev->name, iio_device_id(indio_dev));
	if (!adc->trig) {
		dev_err(&spi->dev, "failed to allocate IIO trigger\n");
		return -ENOMEM;
	}

	adc->trig->ops = &ads1263_trigger_ops;
	adc->trig->dev.parent = &spi->dev;
	iio_trigger_set_drvdata(adc->trig, indio_dev);
	ret = devm_iio_trigger_register(&spi->dev, adc->trig);
	if (ret) {
		dev_err(&spi->dev, "failed to register IIO trigger\n");
		return -ENOMEM;
	}

	indio_dev->trig = iio_trigger_get(adc->trig);

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