#include "icm40609.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_REGISTER(icm, LOG_LEVEL_INF);

int icm_init(icm_t *icm)
{
    int ret;
    uint8_t whoami = 0;

    printk("ICM-40609 WHO_AM_I test starting...\n");

    if (!device_is_ready(icm->bus))
    {
        LOG_ERR("I2C bus device not ready");
        return 1;
    }

    LOG_INF("I2C bus ready, target address: 0x%02x", icm->addr);

    /* Optional: give the sensor time after power-up */
    k_msleep(100);

    ret = icm_reg_read(icm, ICM_REG_WHO_AM_I, &whoami);
    if (ret)
    {
        LOG_ERR("Failed to read WHO_AM_I (err %d)", ret);
        return 2;
    }

    LOG_INF("WHO_AM_I = 0x%02x", whoami);

    if (whoami == ICM40609_WHO_AM_I_VAL)
    {
        LOG_INF("ICM-40609 detected");
    }
    else
    {
        LOG_ERR("Unexpected WHO_AM_I value");
        return 3;
    }

    ret = icm_init_gyro(icm);

    if (ret)
    {
        LOG_ERR("Failed to configure gyro! (%u)", ret);
        return 4;
    }

    return 0;
}

int icm_init_gyro(icm_t *icm)
{
    int ret;

    // Move to bank 0 for system configuration
    ret = icm_select_bank(icm, 0);

    // Set IMU awake, auto clock
    ret = i2c_reg_write_byte_dt(icm, ICM_REG_PWR_MGMT_1, 0x01);
    if (ret)
        return 1;

    // All accel/gyro axes on
    ret = i2c_reg_write_byte_dt(icm, ICM_REG_PWR_MGMT_2, 0x00);
    if (ret)
        return 2;

    // Move to bank 2 for gyro configuration
    ret = icm_select_bank(icm, 2);
    if (ret)
        return 3;

    uint8_t gyro_config = 0x00;
    gyro_config |= ICM_GYRO_RANGE_2000;

    ret = i2c_reg_write_byte_dt(icm, ICM_REG_GYRO_CONFIG_1, gyro_config);
    if (ret)
        return 4;

    // Reset selected bank
    return icm_select_bank(icm, 0);
}

int icm_reg_read(icm_t *icm, uint8_t reg, uint8_t *value)
{
    return i2c_write_read_dt(icm, &reg, sizeof(reg), value, 1);
}

int icm_read_gyro_raw(icm_t *icm, raw_gyro_t *g)
{
    uint8_t buffer[6];
    uint8_t reg = ICM_GYRO_XOUT_H;

    int ret = i2c_write_read_dt(icm, &reg, sizeof(reg), &buffer, ICM_GYRO_SIZE);

    if (ret)
    {
        LOG_ERR("Failed to write/read from gyro registers");
        return 1;
    }

    g->x = (buffer[0] << 8) | buffer[1];
    g->y = (buffer[2] << 8) | buffer[3];
    g->z = (buffer[4] << 8) | buffer[5];

    return 0;
}

int icm_select_bank(icm_t *icm, uint8_t bank)
{
    return i2c_reg_write_byte_dt(icm, ICM_REG_BANK_SEL, (bank << 4));
}
