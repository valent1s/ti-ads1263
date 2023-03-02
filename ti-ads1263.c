#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#define SPI_BUS_NUM 0

#define DRDY_PIN    17
#define RESET_PIN   18
#define CS_PIN      22

#define MAX_SINGLE_CHANNEL_NUM  10
#define MAX_DIFF_CHANNEL_NUM    4

typedef enum {
    ADS1263_SINGLE_MODE,
    ADS1263_DIF_MODE
} ADS1263_MODE;

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

unsigned int irq_number;
static struct spi_device *ads1263_dev;

static void ads1263_reset(void)
{
    gpio_set_value(RESET_PIN, 1);
    msleep(300);
    gpio_set_value(RESET_PIN, 0);
    msleep(300);
    gpio_set_value(RESET_PIN, 1);
    msleep(300);
}

static void ads1263_write_cmd(uint8_t cmd)
{
    gpio_set_value(CS_PIN, 0);
    spi_write(ads1263_dev, &cmd, 1);
    gpio_set_value(CS_PIN, 1);
}

static void ads1263_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t txbuf[3] = { CMD_WREG | reg, 0x00, data };
    gpio_set_value(CS_PIN, 0);
    spi_write(ads1263_dev, txbuf, 3);
    gpio_set_value(CS_PIN, 1);
}

static uint8_t ads1263_read_data(uint8_t reg)
{
    uint8_t txbuf[2] = { CMD_RREG | reg, 0x00 };
    uint8_t ret;

    gpio_set_value(CS_PIN, 0);
    spi_write_then_read(ads1263_dev, txbuf, 2, &ret, 1);
    gpio_set_value(CS_PIN, 1);
    
    return ret;
}

static uint8_t ads1263_check_sum(uint32_t val, uint8_t byt)
{
    uint8_t sum = 0x9b;

    while (val) {
        sum += val & 0xFF;  // only add the lower values
        val >>= 8;          // shift down
    }
    
    return sum ^ byt;       // if equal, this will be 0
}

void ads1263_config_adc1(ADS1263_GAIN gain, ADS1263_DRATE drate, ADS1263_DELAY delay)
{
    const uint8_t MODE2 = 0x80 | (gain << 4) | drate;   // 0x80:PGA bypassed, 0x00:PGA enabled
    const uint8_t REFMUX = 0x24;                        // 0x00:+-2.5V as REF, 0x24:VDD,VSS as REF
    const uint8_t MODE0 = delay;
    
    ads1263_write_reg(REG_MODE2, MODE2);
    msleep(1);
    if (ads1263_read_data(REG_MODE2) != MODE2) {
        printk("REG_MODE2 unsuccess \r\n");
        return;
    }

    ads1263_write_reg(REG_REFMUX, REFMUX);
    msleep(1);
    if (ads1263_read_data(REG_REFMUX) != REFMUX) {
        printk("REG_REFMUX unsuccess \r\n");
        return;
    }

    ads1263_write_reg(REG_MODE0, MODE0);
    msleep(1);
    if (ads1263_read_data(REG_MODE0) != MODE0) {
        printk("REG_MODE0 unsuccess \r\n");
        return;
    }
}

void ads1263_init_adc1(ADS1263_DRATE rate)
{
    ads1263_write_cmd(CMD_STOP1);
    ads1263_config_adc1(ADS1263_GAIN_1, rate, ADS1263_DELAY_35us);
    ads1263_write_cmd(CMD_START1);
}

static void ads1263_set_channel(int channel)
{
    uint8_t INPMUX = (channel << 4) | 0x0A; //  0x0A:VCOM as Negative Input
    
    if (channel > MAX_SINGLE_CHANNEL_NUM) {
        return;
    }

    ads1263_write_reg(REG_INPMUX, INPMUX);
    if (ads1263_read_data(REG_INPMUX) != INPMUX) {
        printk("ADS1263_ADC1_SetChannal unsuccess \r\n");
        return;
    }
}

static uint32_t ads1263_read_adc1_data(void)
{
    uint8_t status;
    const uint8_t cmd = CMD_RDATA1;
    uint8_t buf[5];
    uint32_t read;

    gpio_set_value(CS_PIN, 0);

    do {
        spi_write_then_read(ads1263_dev, &cmd, 1, &status, 1);
    } while(!(status & 0x40));
    spi_read(ads1263_dev, buf, 5);

    gpio_set_value(CS_PIN, 1);

    read  = (uint32_t)buf[0] << 24;
    read |= (uint32_t)buf[1] << 16;
    read |= (uint32_t)buf[2] << 8;
    read |= (uint32_t)buf[3];

    if (ads1263_check_sum(read, buf[4]) != 0) {
        printk("ADC1 Data read error! \r\n");
        return 0;
    }

    return read;
}

uint32_t ads1263_get_channel_value(int channel, ADS1263_MODE mode)
{
    // switch (mode) {
    // case ADS1263_SINGLE_MODE:
    //     if (channel > MAX_SINGLE_CHANNEL_NUM)
    //         return 0;
    //     ads1263_set_channel(channel);
    //     break;
    // // case ADS1263_DIF_MODE:
    // //     if (channel > MAX_DIFF_CHANNEL_NUM)
    // //         return 0;
    //     // ads1263_set_diff_channel(channel);
    // }

    return ads1263_read_adc1_data();
}

static irqreturn_t gpio_irq_handler(int irq, void *dev_id) {
    static uint8_t channel = 0;
    
    if (++channel >= 10) channel = 0;

    uint8_t tx[6] = { CMD_WREG | REG_INPMUX, 0x00, (channel << 4) | 0x0A, 0x00, 0x00, 0x00 };
    uint8_t rx[6];
    struct spi_transfer t = {
        .tx_buf = tx,
        .rx_buf = &rx,
        .len = 6
    };

    struct spi_message m;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);

    disable_irq(65);
    spi_bus_lock(ads1263_dev->controller);
    gpio_set_value(CS_PIN, 0);
    spi_sync_locked(ads1263_dev, &m);
    gpio_set_value(CS_PIN, 1);
    spi_bus_unlock(ads1263_dev->controller);
    enable_irq(65);

    return IRQ_HANDLED;
}

static int __init ModuleInit(void) {
    struct spi_board_info spi_device_info = {
        .modalias = "ads1263",
        .max_speed_hz = 8000000,
        .bus_num = SPI_BUS_NUM,
        .chip_select = 1,
        .mode = SPI_MODE_1,
    };

    /* Get access to spi bus */
    struct spi_master *master = spi_busnum_to_master(SPI_BUS_NUM);
    /* Check if we could get the master */
    if (!master) {
        printk("There is no spi bus with Nr. %d\n", SPI_BUS_NUM);
        return -1;
    }

    /* Create new SPI device */
    ads1263_dev = spi_new_device(master, &spi_device_info);
    if (!ads1263_dev) {
        spi_unregister_device(ads1263_dev);
        printk("Could not create device!\n");
        return -1;
    }

    ads1263_dev->bits_per_word = 8;

    /* Setup the bus for device's parameters */
    if (spi_setup(ads1263_dev) != 0) {
        printk("Could not change bus setup!\n");
        return -1;
    }

    if (gpio_request(CS_PIN, "rpi-gpio-4")) {
        printk("Can not allocate GPIO 4\n");
        return -1;
    }

    if (gpio_direction_output(CS_PIN, 1)) {
        printk("Can not set GPIO 4 to output!\n");
        return -1;
    }

    if (gpio_request(RESET_PIN, "rpi-gpio-4")) {
        printk("Can not allocate GPIO 4\n");
        return -1;
    }

    if (gpio_direction_output(RESET_PIN, 1)) {
        printk("Can not set GPIO 4 to output!\n");
        return -1;
    }

    if (gpio_request(DRDY_PIN, "DRDY_PIN")) {
        printk("Error!\nCan not allocate GPIO 17\n");
        return -1;
    }

    if (gpio_direction_input(DRDY_PIN)) {
        printk("Error!\nCan not set GPIO 17 to input!\n");
        return -1;
    }

    ads1263_reset();
    ads1263_init_adc1(ADS1263_38400SPS);
    ads1263_set_channel(0);

    /* Setup the interrupt */
    irq_number = gpio_to_irq(DRDY_PIN);
    if (request_threaded_irq(irq_number, NULL, gpio_irq_handler, IRQF_TRIGGER_LOW | IRQF_ONESHOT, "ads1263", NULL) != 0) {
        printk("Error!\nCan not request interrupt nr.: %d\n", irq_number);
        return -1;
    }
    
    return 0;
}

/**
 * @brief This function is called, when the module is removed from the kernel
 */
static void __exit ModuleExit(void) {
    if (ads1263_dev)
        spi_unregister_device(ads1263_dev);

    free_irq(irq_number, NULL);
    gpio_free(DRDY_PIN);
    gpio_free(CS_PIN);
    gpio_free(RESET_PIN);
        
    printk("Goodbye, Kernel\n");
}

module_init(ModuleInit);
module_exit(ModuleExit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Johannes 4 GNU/Linux");
MODULE_DESCRIPTION("A simple LKM to read and write some registers of a BMP280 sensor");
