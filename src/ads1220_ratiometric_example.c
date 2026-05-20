/*
 * Simple Zephyr example for using ADS1220 DOUT/!DRDY as both DRDY and MISO
 * and performing ratiometric measurements across AIN1/AIN2 and AIN3/AIN4.
 *
 * This file demonstrates the pattern: wait for DRDY (GPIO falling edge),
 * assert CS, perform a 3-byte SPI read of the conversion, deassert CS,
 * then switch the ADS1220 multiplexer to the next channel pair.
 *
 * NOTE: Fill the ADS1220 command/register opcodes as needed per your
 * ADS1220 datasheet. The example uses placeholders for register write
 * commands so the SPI/DRDY pattern and Zephyr API usage are clear.
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/spi.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include <sys/util.h>

/* Adjust these to match your board overlay */
#define SPI_DEV_LABEL "SPI_1"
#define CS_GPIO_LABEL "GPIO_0"
#define CS_PIN 14
#define DRDY_GPIO_LABEL "GPIO_0"
#define DRDY_PIN 4

/* ADS1220 command opcodes (common values - verify with your datasheet) */
#define ADS1220_CMD_RESET 0x06
#define ADS1220_CMD_START 0x08
#define ADS1220_CMD_RDATA 0x10
#define ADS1220_CMD_RREG 0x20
#define ADS1220_CMD_WREG 0x40

/* NOTE: The register layout (which register contains the MUX bits) must be
 * verified against the ADS1220 datasheet for correct mux writes. The helper
 * functions below use the common WREG/RREG command prefixes.
 */

static const struct device *spi_dev;
static const struct device *cs_dev;
static const struct device *drdy_dev;

static struct gpio_callback drdy_cb_data;

static struct spi_config spi_cfg = {
    .frequency = 1000000U,
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
    .slave = 0,
    .cs = NULL,
};

/* Simple helper: assert/deassert CS via GPIO */
static inline void cs_assert(void)
{
    gpio_pin_set(cs_dev, CS_PIN, 0);
}

static inline void cs_deassert(void)
{
    gpio_pin_set(cs_dev, CS_PIN, 1);
}

/* Read 3-byte conversion result from ADS1220 (MSB first). */
static int ads1220_read_conversion(int32_t *out_sample)
{
    uint8_t tx[3] = {0x00, 0x00, 0x00};
    uint8_t rx[3];
    const struct spi_buf tx_buf = {.buf = tx, .len = sizeof(tx)};
    const struct spi_buf rx_buf = {.buf = rx, .len = sizeof(rx)};
    const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
    const struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};

    cs_assert();
    int rc = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
    cs_deassert();
    if (rc)
    {
        return rc;
    }

    /* 24-bit signed two's complement -> sign-extend to 32-bit */
    int32_t val = (rx[0] << 16) | (rx[1] << 8) | rx[2];
    if (val & 0x800000)
    {
        val |= 0xFF000000;
    }
    *out_sample = val;
    return 0;
}

/* Write one register byte (placeholder implementation) */
static int ads1220_write_register(uint8_t reg_addr, uint8_t value)
{
    uint8_t tx[2];
    /* Build WREG command: base | reg_addr (datasheet-specific) */
    tx[0] = ADS1220_WREG | (reg_addr & 0x07);
    tx[1] = value;

    const struct spi_buf tx_buf = {.buf = tx, .len = 2};
    const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};

    cs_assert();
    int rc = spi_write(spi_dev, &spi_cfg, &tx_set);
    cs_deassert();
    return rc;
}

/* Configure MUX for differential pair: ainp, ainn (1-based AIN numbers) */
static int ads1220_set_mux(uint8_t ainp, uint8_t ainn)
{
    /* Translate AIN numbers to ADS1220 MUX bits per datasheet. This is
     * device-specific; replace the following example with the correct
     * encoding from the ADS1220 register map.
     */
    uint8_t mux_bits = 0;
    /* placeholder mapping - user must replace with real mapping */
    mux_bits = ((ainp & 0x07) << 3) | (ainn & 0x07);

    /* Example: write to CONFIG register 0 (reg 0) with mux bits shifted
     * into the proper position. Replace with correct register layout.
     */
    uint8_t reg0 = (mux_bits & 0x3F);
    return ads1220_write_register(0x00, reg0);
}

/* Work item triggered from DRDY GPIO callback to read a conversion */
static void ads1220_work_handler(struct k_work *work);
K_WORK_DEFINE(ads1220_read_work, ads1220_work_handler);
/* Initialize ADS1220 example: configure CS and DRDY GPIO, set initial MUX and
 * trigger a conversion. The DRDY callback will schedule reads asynchronously.
 */
int ads1220_ratiometric_example_init(void)
{
    printk("ADS1220 ratiometric example init\n");

    spi_dev = device_get_binding(SPI_DEV_LABEL);
    if (!spi_dev)
    {
        printk("SPI device %s not found\n", SPI_DEV_LABEL);
        return -ENODEV;
    }

    cs_dev = device_get_binding(CS_GPIO_LABEL);
    if (!cs_dev)
    {
        printk("CS gpio %s not found\n", CS_GPIO_LABEL);
        return -ENODEV;
    }

    drdy_dev = device_get_binding(DRDY_GPIO_LABEL);
    if (!drdy_dev)
    {
        printk("DRDY gpio %s not found\n", DRDY_GPIO_LABEL);
        return -ENODEV;
    }

    gpio_pin_configure(cs_dev, CS_PIN, GPIO_OUTPUT_HIGH);

    /* Configure DRDY as input with pull-up and falling-edge trigger */
    gpio_pin_configure(drdy_dev, DRDY_PIN, GPIO_INPUT | GPIO_PULL_UP);
    gpio_init_callback(&drdy_cb_data, drdy_gpio_callback, BIT(DRDY_PIN));
    gpio_add_callback(drdy_dev, &drdy_cb_data);
    gpio_pin_interrupt_configure(drdy_dev, DRDY_PIN, GPIO_INT_EDGE_TO_ACTIVE);

    /* Set initial MUX to AIN1/AIN2 */
    ads1220_set_mux(1, 2);

    /* Start a conversion (common START opcode) */
    uint8_t cmd = ADS1220_CMD_START;
    const struct spi_buf tx_buf = {.buf = &cmd, .len = 1};
    const struct spi_buf_set tx_set = {.buffers = &tx_buf, .count = 1};
    cs_assert();
    spi_write(spi_dev, &spi_cfg, &tx_set);
    cs_deassert();

    return 0;
}
return;
}

gpio_pin_configure(cs_dev, CS_PIN, GPIO_OUTPUT_HIGH);

/* Configure DRDY as input with falling-edge trigger */
gpio_pin_configure(drdy_dev, DRDY_PIN, GPIO_INPUT | GPIO_PULL_UP);
gpio_init_callback(&drdy_cb_data, drdy_gpio_callback, BIT(DRDY_PIN));
gpio_add_callback(drdy_dev, &drdy_cb_data);
gpio_pin_interrupt_configure(drdy_dev, DRDY_PIN, GPIO_INT_EDGE_TO_INACTIVE);

/* Example sequence: measure differential AIN1-AIN2, then AIN3-AIN4 */
while (1)
{
    /* Set MUX to AIN1/AIN2 */
    ads1220_set_mux(1, 2);
    /* Start a conversion if required by your device */
    /* e.g., send START command: */
    /* cs_assert(); spi_write(... START ...); cs_deassert(); */

    /* Wait for DRDY callback to read result (or poll) */
    k_sleep(K_MSEC(200));

    /* Then switch to AIN3/AIN4 */
    ads1220_set_mux(3, 4);
    /* trigger conversion as needed and wait */
    k_sleep(K_MSEC(200));
}
}
