/**
 * @file max17048.c
 * @brief Driver for MAX17048 Battery monitor. Written for use in Bike Power Meter project.
 * @version 0.1
 * @date 2026-05-24
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "max17048.h"

// #include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

// LOG_MODULE_REGISTER(mon, LOG_LEVEL_INF);
LOG_MODULE_REGISTER(mon, LOG_LEVEL_DBG);

int battery_monitor_init(batt_mon_t *mon)
{
    LOG_INF("MAX17048 WHO_AM_I test starting...\n");

    // Confirm i2c bus is available
    if (!device_is_ready(mon->bus))
    {
        LOG_ERR("I2C bus device not ready");
        return 1;
    }

    LOG_INF("I2C bus ready, target address: 0x%02x", mon->addr);

    /* Optional: give the sensor time after power-up */
    k_msleep(100);

    // Read and confirm battery monitor version
    uint8_t buf[2];
    uint8_t reg = MAX17048_REG_VERSION;
    int ret = i2c_write_read_dt(mon, &reg, 1, &buf, 2);

    if (ret)
    {
        LOG_WRN("Failed to write to MAX17048 Battery Monitor. Is the battery connected?");
        return ret;
    }

    uint16_t version = (buf[0] << 8) | buf[1];

    // Confirm version
    LOG_INF("Read battery monitor version: 0x%04x", version);

    if (version != MAX17048_VERSION_VAL)
    {
        LOG_ERR("Battery monitor version does not match");

        return -1;
    }

    LOG_INF("Battery monitor connected!");

    // Show status upon connection
    uint8_t pct = battery_monitor_read_soc(mon);

    LOG_INF("Battery SOC: %d%%", pct);

    return 0;
}

/**
 * @brief Read battery level in voltage (mV)
 *
 * @param mon Battery monitor device pointer
 * @return uint16_t Battery level in millivolts
 */
uint16_t battery_monitor_read_voltage(batt_mon_t *mon)
{
    uint16_t val;

    val = battery_monitor_write_read(mon, MAX17048_REG_VCELL);

    if (val == 0xFFFF)
    {
        return val;
    }

    /* Val returned is 16-bit where LSB = 78.125 uV (0.078125 mV) */
    /* mv = val * 0.078125 mV = val * 625 / 8000 */
    uint32_t mv = ((uint32_t)val * 625) / 8000;

    return (uint16_t)mv;
}

/**
 * @brief Read battery level in percent
 *
 * @param mon Battery monitor device pointer
 * @return uint8_t Battery level in percent
 */
uint8_t battery_monitor_read_soc(batt_mon_t *mon)
{
    uint16_t val;
    val = battery_monitor_write_read(mon, MAX17048_REG_SOC);

    if (val == 0xFFFF)
    {
        return 0xFF;
    }

    uint8_t batt_percent = val / 256;

    return batt_percent;
}

uint16_t battery_monitor_read_config(batt_mon_t *mon)
{
    return -1;
}

uint16_t battery_monitor_read_status(batt_mon_t *mon)
{
    return -1;
}

uint16_t battery_monitor_read_mode(batt_mon_t *mon)
{
    return -1;
}

/**
 * @brief Read from a register. Only for 2 byte write, 2 byte return (required)
 *
 * @param mon Battery monitor device pointer
 * @param reg Target register
 * @return uint16_t Return value. Bad read returns 0xFFFF.
 */
uint16_t battery_monitor_write_read(batt_mon_t *mon, uint8_t reg)
{
    uint8_t buf[2];
    /* Write single-byte register address, then read 2-byte register value */
    if (i2c_write_read_dt(mon, &reg, 1, &buf, 2))
    {
        LOG_INF("Failed to write 0x%02x to battery monitor", reg);
        return 0xFFFF;
    }

    LOG_DBG("Received 0x%02x 0x%02x", buf[0], buf[1]);

    return (uint16_t)((buf[0] << 8) | buf[1]);
}
