#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

#define ADS1263_RESOLUTION          32

#define ADS1263_CLK_HZ				7372800
#define ADS1263_CLK_NS				(NSEC_PER_SEC / ADS1263_CLK_HZ)

#define ADS1263_WAIT_RESET_CYCLES	4

typedef enum {
    /* Register address, followed by reset the default values */
    REG_ID,         // xxh
    REG_POWER,      // 11h
    REG_INTERFACE,  // 05h
    REG_MODE0,      // 00h
    REG_MODE1,      // 80h
    REG_MODE2,      // 04h
    REG_INPMUX,     // 01h
    REG_OFCAL0,     // 00h
    REG_OFCAL1,     // 00h
    REG_OFCAL2,     // 00h
    REG_FSCAL0,     // 00h
    REG_FSCAL1,     // 00h
    REG_FSCAL2,     // 40h
    REG_IDACMUX,    // BBh
    REG_IDACMAG,    // 00h
    REG_REFMUX,     // 00h
    REG_TDACP,      // 00h
    REG_TDACN,      // 00h
    REG_GPIOCON,    // 00h
    REG_GPIODIR,    // 00h
    REG_GPIODAT,    // 00h
    REG_ADC2CFG,    // 00h
    REG_ADC2MUX,    // 01h
    REG_ADC2OFC0,   // 00h
    REG_ADC2OFC1,   // 00h
    REG_ADC2FSC0,   // 00h
    REG_ADC2FSC1,   // 40h
} ADS1263_REG;

#define ADS1263_ID_DEV_ID(x)        GENMASK(7, 5)
#define ADS1263_ID_REV_ID           GENMASK(4, 0)

enum ADS1263_ID {
    ADS1263_ID_ADS1262,
    ADS1263_ID_ADS1263
};

#define ADS1263_POWER_RESET         BIT(4)
#define ADS1263_POWER_VBIAS         BIT(1)
#define ADS1263_POWER_INTREF        BIT(0)

#define ADS1263_INTERFACE_TIMEOUT   BIT(3)
#define ADS1263_INTERFACE_STATUS    BIT(2)
#define ADS1263_INTERFACE_CRC(x)    (((x) << 0) & GENMASK(1, 0))

#define ADS1263_CRC_DISABLE         0
#define ADS1263_CRC_CHECKSUM_MODE   1
#define ADS1263_CRC_CRC_MODE        2

#define ADS1263_MODE0_REFREV        BIT(7)
#define ADS1263_MODE0_RUNMODE       BIT(6)
#define ADS1263_MODE0_CHOP(x)       (((x) << 4) & GENMASK(5, 4))
#define ADS1263_MODE0_DELAY(x)      (((x) << 0) & GENMASK(3, 0))

#define ADS1263_CHOP_DISABLE        0
#define ADS1263_CHOP_CHOP           1
#define ADS1263_CHOP_IDAC           2
#define ADS1263_CHOP_CHOP_IDAC      3

typedef enum {
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
    ADS1263_DELAY_8d8ms
} ADS1263_DELAY;

#define ADS1263_MODE1_FILTER(x)     (((x) << 5) & GENMASK(7, 5))

typedef enum {
    ADS1263_SINC1,
    ADS1263_SINC2,
    ADS1263_SINC3,
    ADS1263_SINC4,
    ADS1263_FIR
} ADS1263_FILTER;

#define ADS1263_MODE1_SBADC         BIT(4)
#define ADS1263_MODE1_SBPOL         BIT(3)
#define ADS1263_MODE1_SBMAG(x)      (((x) << 0) & GENMASK(2, 0))

#define ADS1263_MODE2_BYPASS        BIT(7)
#define ADS1263_MODE2_GAIN(x)       (((x) << 4) & GENMASK(6, 4))
#define ADS1263_MODE2_DR(x)         (((x) << 0) & GENMASK(3, 0))

typedef enum {
    ADS1263_GAIN_1,
    ADS1263_GAIN_2,
    ADS1263_GAIN_4,
    ADS1263_GAIN_8,
    ADS1263_GAIN_16,
    ADS1263_GAIN_32,
    ADS1263_GAIN_64,
    ADS1263_GAIN_128
} ADS1263_GAIN;

typedef enum {
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
    ADS1263_38400SPS
} ADS1263_DRATE;

#define ADS1263_INPMUX_MUXP(x)      (((x) << 4) & GENMASK(7, 4))
#define ADS1263_INPMUX_MUXN(x)      (((x) << 0) & GENMASK(3, 0))

typedef enum {
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
    ADS1263_FLOAT
} ADS1263_INPMUX;

#define ADS1263_IDACMUX_MUX2(x)     (((x) << 4) & GENMASK(7, 4))
#define ADS1263_IDACMUX_MUX1(x)     (((x) << 0) & GENMASK(3, 0))

#define ADS1263_IDACMAG_MAG2(x)     (((x) << 4) & GENMASK(7, 4))
#define ADS1263_IDACMAG_MAG1(x)     (((x) << 0) & GENMASK(3, 0))

typedef enum {
    ADS1263_DAC_VLOT_4_5        = 0b01001,
    ADS1263_DAC_VLOT_3_5        = 0b01000,
    ADS1263_DAC_VLOT_3          = 0b00111,
    ADS1263_DAC_VLOT_2_75       = 0b00110,
    ADS1263_DAC_VLOT_2_625      = 0b00101,
    ADS1263_DAC_VLOT_2_5625     = 0b00100,
    ADS1263_DAC_VLOT_2_53125    = 0b00011,
    ADS1263_DAC_VLOT_2_515625   = 0b00010,
    ADS1263_DAC_VLOT_2_5078125  = 0b00001,
    ADS1263_DAC_VLOT_2_5        = 0b00000,
    ADS1263_DAC_VLOT_2_4921875  = 0b10001,
    ADS1263_DAC_VLOT_2_484375   = 0b10010,
    ADS1263_DAC_VLOT_2_46875    = 0b10011,
    ADS1263_DAC_VLOT_2_4375     = 0b10100,
    ADS1263_DAC_VLOT_2_375      = 0b10101,
    ADS1263_DAC_VLOT_2_25       = 0b10110,
    ADS1263_DAC_VLOT_2          = 0b10111,
    ADS1263_DAC_VLOT_1_5        = 0b11000,
    ADS1263_DAC_VLOT_0_5        = 0b11001
} ADS1263_DAC_VOLT;

#define ADS1263_REFMUX_RMUXP(x)     (((x) << 3) & GENMASK(5, 3))
#define ADS1263_REFMUX_RMUXN(x)     (((x) << 0) & GENMASK(2, 0))

typedef enum {
    ADS1263_INTERNAL_REF,
    ADS1263_EXTERNAL_AIN0,
    ADS1263_EXTERNAL_AIN2,
    ADS1263_EXTERNAL_AIN4,
    ADS1263_INTERNAL_VAVDD
} ADS1263_REFMUX;

#define ADS1263_TDACP_OUTP          BIT(7)
#define ADS1263_TDACP_MAGP(x)       (((x) << 0) & GENMASK(4, 0))

#define ADS1263_TDACN_OUTN          BIT(7)
#define ADS1263_TDACN_MAGN(x)       (((x) << 0) & GENMASK(4, 0))

#define ADS1263_GPIOCON(x)          BIT(x)

#define ADS1263_GPIODIR(x)          BIT(x)

#define ADS1263_GPIODAT(x)          BIT(x)

#define ADS1263_ADC2CFG_DR2(x)      (((x) << 6) & GENMASK(7, 6))
#define ADS1263_ADC2CFG_REF2(x)     (((x) << 3) & GENMASK(5, 3))
#define ADS1263_ADC2CFG_GAIN2(x)    (((x) << 0) & GENMASK(2, 0))

typedef enum {
    ADS1263_ADC2_10SPS,
    ADS1263_ADC2_100SPS,
    ADS1263_ADC2_400SPS,
    ADS1263_ADC2_800SPS
} ADS1263_ADC2_DRATE;

typedef enum {
    ADS1263_ADC2_GAIN_1,
    ADS1263_ADC2_GAIN_2,
    ADS1263_ADC2_GAIN_4,
    ADS1263_ADC2_GAIN_8,
    ADS1263_ADC2_GAIN_16,
    ADS1263_ADC2_GAIN_32,
    ADS1263_ADC2_GAIN_64,
    ADS1263_ADC2_GAIN_128
} ADS1263_ADC2_GAIN;

#define ADS1263_ADC2MUX_MUXP2(x)    (((x) << 4) & GENMASK(7, 4))
#define ADS1263_ADC2MUX_MUXN2(x)    (((x) << 0) & GENMASK(3, 0))

typedef enum {
    CMD_NOP     = 0x00, // NOP
    CMD_RESET   = 0x06, // Reset the ADC, 0000 011x (06h or 07h)
    CMD_START1  = 0x08, // Start ADC1 conversions, 0000 100x (08h or 09h)
    CMD_STOP1   = 0x0A, // Stop ADC1 conversions, 0000 101x (0Ah or 0Bh)
    CMD_START2  = 0x0C, // Start ADC2 conversions, 0000 110x (0Ch or 0Dh)
    CMD_STOP2   = 0x0E, // Stop ADC2 conversions, 0000 111x (0Eh or 0Fh)
    CMD_RDATA1  = 0x12, // Read ADC1 data, 0001 001x (12h or 13h)
    CMD_RDATA2  = 0x14, // Read ADC2 data, 0001 010x (14h or 15h)
    CMD_SYOCAL1 = 0x16, // ADC1 system offset calibration, 0001 0110 (16h)
    CMD_SYGCAL1 = 0x17, // ADC1 system gain calibration, 0001 0111 (17h)
    CMD_SFOCAL1 = 0x19, // ADC1 self offset calibration, 0001 1001 (19h)
    CMD_SYOCAL2 = 0x1B, // ADC2 system offset calibration, 0001 1011 (1Bh)
    CMD_SYGCAL2 = 0x1C, // ADC2 system gain calibration, 0001 1100 (1Ch)
    CMD_SFOCAL2 = 0x1E, // ADC2 self offset calibration, 0001 1110 (1Eh)
    CMD_RREG    = 0x20, // Read registers 001r rrrr (20h+000r rrrr)
    CMD_RREG2   = 0x00, // number of registers to read minus 1, 000n nnnn
    CMD_WREG    = 0x40, // Write registers 010r rrrr (40h+000r rrrr)
    CMD_WREG2   = 0x00  // number of registers to write minus 1, 000n nnnn
} ADS1263_CMD;

struct ads1263_channel_config {
	unsigned int pga_gain;
};

struct ads1263 {
	struct spi_device *spi;

    struct ads1263_channel_config *channel_config;

	struct gpio_desc  *drdy_pin;
	struct gpio_desc  *reset_pin;

	unsigned int reset_delay_us;
};

#define ADC1263_VOLTAGE_CHANNEL(chan1, chan2, si)                   \
		{															\
			.type = IIO_VOLTAGE,									\
			.indexed = 1,											\
			.channel = (chan1),										\
			.channel2 = (chan2),									\
			.differential = (chan2) < ADS1263_AINCOM, 	    	    \
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |          \
                BIT(IIO_CHAN_INFO_HARDWAREGAIN),			        \
			.scan_index = (si),										\
			.scan_type = {											\
				.sign = 's',									    \
				.realbits = 24,						                \
				.storagebits = ADS1263_RESOLUTION,					\
                .shift = 8,                                         \
			},														\
		}

static inline void ads1263_reset(struct ads1263 *adc)
{
    gpiod_set_value(adc->reset_pin, 0);
    udelay(adc->reset_delay_us);
    gpiod_set_value(adc->reset_pin, 1);
}

static inline int ads1263_write_cmd(struct ads1263 *adc, u8 cmd)
{
    return spi_write(adc->spi, &cmd, 1);
}

static inline int ads1263_write_reg(struct ads1263 *adc, u8 reg, u8 data)
{
    const u8 txbuf[3] = { CMD_WREG | reg, 0, data };

    return spi_write(adc->spi, txbuf, 3);
}

static ssize_t ads1263_read_reg(struct ads1263 *adc, u8 reg)
{
    const u8 txbuf[2] = { CMD_RREG | reg, 0 };
    ssize_t	status;
	u16 result;

	status = spi_write_then_read(adc->spi, txbuf, 2, &result, 2);

	/* return negative errno or unsigned value */
	return (status < 0) ? status : result;
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
    
    const u8 MODE2 = ADS1263_MODE2_DR(ADS1263_400SPS);
    const u8 REFMUX = ADS1263_REFMUX_RMUXN(ADS1263_INTERNAL_VAVDD) |
        ADS1263_REFMUX_RMUXP(ADS1263_INTERNAL_VAVDD);
    const u8 MODE1 = ADS1263_MODE1_FILTER(ADS1263_SINC4);
    struct ads1263_channel_config *channel_config;
    size_t i = indio_dev->num_channels;

    ads1263_reset(adc);
    ads1263_write_cmd(adc, CMD_STOP1);
    
    ads1263_write_reg(adc, REG_MODE2, MODE2);
    if (ads1263_read_reg(adc, REG_MODE2) != MODE2) {
        printk("REG_MODE2 unsuccess\r\n");
        return 0;
    }

    ads1263_write_reg(adc, REG_REFMUX, REFMUX);
    if (ads1263_read_reg(adc, REG_REFMUX) != REFMUX) {
        printk("REG_REFMUX unsuccess\r\n");
        return 0;
    }

    ads1263_write_reg(adc, REG_MODE1, MODE1);
    if (ads1263_read_reg(adc, REG_MODE1) != MODE1) {
        printk("REG_MODE1 unsuccess\r\n");
        return 0;
    }

    ads1263_write_cmd(adc, CMD_START1);

    channel_config = devm_kcalloc(&adc->spi->dev, i, sizeof(*channel_config), GFP_KERNEL);
	if (!channel_config)
        return -ENOMEM;
    
    while(--i)
        channel_config[i].pga_gain = ADS1263_GAIN_1;

    adc->channel_config = channel_config;

    return 0;
}

static void ads1263_set_channel(struct ads1263 *adc, int chan1, int chan2)
{
    const u8 INPMUX = ADS1263_INPMUX_MUXP(chan1) | ADS1263_INPMUX_MUXN(chan2);
    
    ads1263_write_reg(adc, REG_INPMUX, INPMUX);
    // if (ads1263_read_reg(adc, REG_INPMUX) != INPMUX) {
    //     printk("ADS1263_ADC1_SetChannal unsuccess \r\n");
    //     return;
    // }
}

static u32 ads1263_read_adc1_data(struct ads1263 *adc)
{
    u8 buf[6];
    u32 read;

    spi_read(adc->spi, buf, 6);

    read  = (u32)buf[1] << 24;
    read |= (u32)buf[2] << 16;
    read |= (u32)buf[3] << 8;
    read |= (u32)buf[4];

    return ads1263_check_sum(read, buf[5]) == 0 ? read : 0;
}

static int ads1263_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int *val, int *val2, long mask) {
	struct ads1263 *adc = iio_priv(indio_dev);

    const u8 MODE2 = (adc->channel_config[chan->channel].pga_gain << 4) | ADS1263_400SPS;
	switch (mask) {
	case IIO_CHAN_INFO_RAW:

        ads1263_write_reg(adc, REG_MODE2, MODE2);
		ads1263_set_channel(adc, chan->channel, chan->channel2);

		while (!gpiod_get_value(adc->drdy_pin));
		while (gpiod_get_value(adc->drdy_pin));

		*val = ads1263_read_adc1_data(adc);
		
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

static const struct iio_info ads1263_info = {
	.read_raw = ads1263_read_raw,
    .write_raw = ads1263_write_raw,
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

	adc->drdy_pin = devm_gpiod_get(&spi->dev, "drdy", GPIOD_IN);
	if (IS_ERR(adc->drdy_pin))
		return PTR_ERR(adc->drdy_pin);

	adc->reset_pin = devm_gpiod_get(&spi->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(adc->reset_pin))
		return PTR_ERR(adc->reset_pin);
	
	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = "ads1263";
	indio_dev->info = &ads1263_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ads1263_channels;
	indio_dev->num_channels = ARRAY_SIZE(ads1263_channels);

	adc->reset_delay_us = DIV_ROUND_UP(
		ADS1263_WAIT_RESET_CYCLES * ADS1263_CLK_NS, NSEC_PER_USEC);
    
    ret = ads1263_setup(indio_dev);
    if (ret < 0) {
        dev_err(&spi->dev, "ADS1263 setup failed\r\n");   
        return ret;
    }

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id ads1263_of_match[] = {
	{
		.compatible	= "ti,ads1263",
	},
	{ }
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