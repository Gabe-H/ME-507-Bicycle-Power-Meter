#include "icm20948.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(icm, LOG_LEVEL_INF);

static const struct i2c_dt_spec icm = I2C_DT_SPEC_GET(ICM20948_NODE);

int icm20948_init(void)
{
    int ret;
    uint8_t whoami = 0;

    LOG_DBG("ICM-20948 WHO_AM_I test starting...\n");

    if (!device_is_ready(icm.bus))
    {
        LOG_ERR("I2C bus device not ready");
        return 1;
    }

    LOG_INF("I2C bus ready, target address: 0x%02x", icm.addr);

    /* Optional: give the sensor time after power-up */
    k_msleep(100);

    ret = icm20948_reg_read(ICM20948_REG_WHO_AM_I, &whoami);
    if (ret)
    {
        LOG_ERR("Failed to read WHO_AM_I (err %d)", ret);
        return 2;
    }

    LOG_INF("WHO_AM_I = 0x%02x", whoami);

    if (whoami == ICM20948_WHO_AM_I_VAL)
    {
        LOG_INF("ICM-20948 detected");
    }
    else
    {
        LOG_ERR("Unexpected WHO_AM_I value");
        return 3;
    }

    ret = icm20948_init_gyro();

    if (ret)
    {
        LOG_ERR("Failed to configure gyro! (%u)", ret);
        return 4;
    }

    return 0;
}

int icm20948_init_gyro(void)
{
    int ret;

    // Move to bank 0 for system configuration
    ret = icm20948_select_bank(0);

    // Set IMU awake, auto clock
    ret = i2c_reg_write_byte_dt(&icm, ICM20948_REG_PWR_MGMT_1, 0x01);
    if (ret)
        return 1;

    // All accel/gyro axes on
    ret = i2c_reg_write_byte_dt(&icm, ICM20948_REG_PWR_MGMT_2, 0x00);
    if (ret)
        return 2;

    // Move to bank 2 for gyro configuration
    ret = icm20948_select_bank(2);
    if (ret)
        return 3;

    uint8_t gyro_config = 0x00;
    gyro_config |= ICM20948_GYRO_RANGE_2000;

    ret = i2c_reg_write_byte_dt(&icm, ICM20948_REG_GYRO_CONFIG_1, gyro_config);
    if (ret)
        return 4;

    // Reset selected bank
    return icm20948_select_bank(0);
}

int icm20948_reg_read(uint8_t reg, uint8_t *value)
{
    return i2c_write_read_dt(&icm, &reg, sizeof(reg), value, 1);
}

int icm20948_read_gyro_raw(raw_gyro_t *g)
{
    uint8_t buffer[6];
    uint8_t reg = ICM20948_GYRO_XOUT_H;

    int ret = i2c_write_read_dt(&icm, &reg, sizeof(reg), &buffer, ICM20948_GYRO_SIZE);

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

int icm20948_select_bank(uint8_t bank)
{
    return i2c_reg_write_byte_dt(&icm, ICM20948_REG_BANK_SEL, (bank << 4));
}
