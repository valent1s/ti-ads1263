#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

#define ADS1263_CLK_HZ				7372800
#define ADS1263_CLK_NS				(1 / ADS1263_CLK_HZ)

#define ADS1263_WAIT_RESET_CYCLES	4

typedef enum {
    ADS1263_GAIN_1,
    ADS1263_GAIN_2,
    ADS1263_GAIN_4,
    ADS1263_GAIN_8,
    ADS1263_GAIN_16,
    ADS1263_GAIN_32,
    ADS1263_GAIN_64
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
    ADS1263_DELAY_8d8ms,
} ADS1263_DELAY;

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

typedef enum {
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

struct ads1263 {
	struct spi_device *spi;

	struct gpio_desc  *drdy_pin;
	struct gpio_desc  *reset_pin;

	unsigned int reset_delay_us;
};

#define ADC1263_VOLTAGE_CHANNEL(chan1, chan2, si, s)				\
		{															\
			.type = IIO_VOLTAGE,									\
			.indexed = 1,											\
			.channel = (chan1),										\
			.channel2 = (chan2),									\
			.differential = 1,										\
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
			.scan_index = si,										\
			.scan_type = {											\
				.sign = s,											\
				.realbits = 32,										\
				.storagebits = 32,									\
			},														\
		}

static void ads1263_reset(struct ads1263 *adc)
{
    gpiod_set_value(adc->reset_pin, 0);
    udelay(adc->reset_delay_us);
    gpiod_set_value(adc->reset_pin, 1);
}

static void ads1263_write_cmd(struct ads1263 *adc, u8 cmd)
{
    spi_write(adc->spi, &cmd, 1);
}

static void ads1263_write_reg(struct ads1263 *adc, u8 reg, u8 data)
{
    const u8 txbuf[3] = { CMD_WREG | reg, 0x00, data };

    spi_write(adc->spi, txbuf, 3);
}

static u8 ads1263_read_reg(struct ads1263 *adc, u8 reg)
{
    const u8 txbuf[2] = { CMD_RREG | reg, 0x00 };
    u8 ret;

    spi_write_then_read(adc->spi, txbuf, 2, &ret, 1);
    
    return ret;
}

static u8 ads1263_check_sum(u32 val, u8 byt)
{
    u8 sum = 0x9b;

    while (val) {
        sum += val & 0xFF;  // only add the lower values
        val >>= 8;          // shift down
    }
    
    return sum ^ byt;       // if equal, this will be 0
}

static void ads1263_config_adc1(struct ads1263 *adc, ADS1263_GAIN gain, ADS1263_DRATE drate, ADS1263_DELAY delay)
{
    const u8 MODE2 = 0x80 | (gain << 4) | drate;   // 0x80:PGA bypassed, 0x00:PGA enabled
    const u8 REFMUX = 0x24;                        // 0x00:+-2.5V as REF, 0x24:VDD,VSS as REF
    const u8 MODE0 = delay;
    
    ads1263_write_reg(adc, REG_MODE2, MODE2);
    if (ads1263_read_reg(adc, REG_MODE2) != MODE2) {
        printk("REG_MODE2 unsuccess \r\n");
        return;
    }

    ads1263_write_reg(adc, REG_REFMUX, REFMUX);
    if (ads1263_read_reg(adc, REG_REFMUX) != REFMUX) {
        printk("REG_REFMUX unsuccess \r\n");
        return;
    }

    ads1263_write_reg(adc, REG_MODE0, MODE0);
    if (ads1263_read_reg(adc, REG_MODE0) != MODE0) {
        printk("REG_MODE0 unsuccess \r\n");
        return;
    }
}

static void ads1263_init_adc1(struct ads1263 *adc, ADS1263_DRATE rate)
{
    ads1263_write_cmd(adc, CMD_STOP1);
    ads1263_config_adc1(adc, ADS1263_GAIN_1, rate, ADS1263_DELAY_0s);
    ads1263_write_cmd(adc, CMD_START1);
}

static void ads1263_set_channel(struct ads1263 *adc, int chan1, int chan2)
{
    const u8 INPMUX = (chan1 << 4) | chan2;
    
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

    if (ads1263_check_sum(read, buf[5]) != 0)
        return 0;

    return read;
}

static int ads1263_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int *val, int *val2, long mask) {
	struct ads1263 *adc = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:

		ads1263_set_channel(adc, chan->channel, chan->channel2);

		while (!gpiod_get_value(adc->drdy_pin));
		while (gpiod_get_value(adc->drdy_pin));

		*val = ads1263_read_adc1_data(adc);
		
		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static const struct iio_chan_spec ads1263_channels[] = {
	ADC1263_VOLTAGE_CHANNEL(0, 0x0A, 0, 's'),
	ADC1263_VOLTAGE_CHANNEL(1, 0x0A, 1, 's'),
	ADC1263_VOLTAGE_CHANNEL(2, 0x0A, 2, 's'),
	ADC1263_VOLTAGE_CHANNEL(3, 0x0A, 3, 's'),
	ADC1263_VOLTAGE_CHANNEL(4, 0x0A, 4, 's'),
	ADC1263_VOLTAGE_CHANNEL(5, 0x0A, 5, 's'),
	ADC1263_VOLTAGE_CHANNEL(6, 0x0A, 6, 's'),
	ADC1263_VOLTAGE_CHANNEL(7, 0x0A, 7, 's'),
	ADC1263_VOLTAGE_CHANNEL(8, 0x0A, 8, 's'),
	ADC1263_VOLTAGE_CHANNEL(9, 0x0A, 9, 's'),
	ADC1263_VOLTAGE_CHANNEL(0, 1, 10, 's'),
	ADC1263_VOLTAGE_CHANNEL(2, 3, 11, 's'),
	ADC1263_VOLTAGE_CHANNEL(4, 5, 12, 's'),
	ADC1263_VOLTAGE_CHANNEL(6, 7, 13, 's'),
	ADC1263_VOLTAGE_CHANNEL(8, 9, 14, 's'),
};

static const struct iio_info ads1263_info = {
	.read_raw = ads1263_read_raw,
};

static const struct of_device_id ads1263_of_match[] = {
	{
		.compatible	= "ti,ads1263",
	},
	{ }
};
MODULE_DEVICE_TABLE(of, ads1263_of_match);

static int ads1263_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct ads1263 *adc;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(struct iio_dev));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	spi_set_drvdata(spi, indio_dev);
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

    ads1263_reset(adc);
    ads1263_init_adc1(adc, ADS1263_7200SPS);

	return devm_iio_device_register(&spi->dev, indio_dev);
}

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