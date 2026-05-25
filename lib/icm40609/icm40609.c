#include "icm40609.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

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
    k_msleep(50);

    ret = icm_reg_read(icm, ICM_REG_WHO_AM_I, &whoami);
    if (ret)
    {
        LOG_ERR("Failed to read WHO_AM_I (err %d)", ret);
        return 2;
    }

    LOG_INF("WHO_AM_I = 0x%02x (expect 0x%02x)", whoami, ICM40609_WHO_AM_I_VAL);

    if (whoami == ICM40609_WHO_AM_I_VAL)
    {
        LOG_INF("ICM-40609 detected");
    }
    else
    {
        LOG_ERR("Unexpected WHO_AM_I value");
        return 3;
    }

    ret = icm_reset(icm);
    if (ret)
    {
        LOG_WRN("Failed to reset IMU %d]", ret);
        return ret;
    }

    ret = icm_init_power(icm);
    if (ret)
    {
        LOG_WRN("Failed to init IMU power [%d]", ret);
        return ret;
    }

    ret = icm_init_gyro(icm);
    if (ret)
    {
        LOG_WRN("Failed to init IMU gyro [%d]", ret);
        return ret;
    }

    ret = icm_init_accel(icm);
    if (ret)
    {
        LOG_WRN("Failed to init IMU accel [%d]", ret);
        return ret;
    }

    return 0;
}

int icm_reset(icm_t *icm)
{
    int ret;

    ret = icm_select_bank(icm, 0);
    if (ret)
        return 1;

    ret = icm_reg_write(icm, ICM_REG_DEVICE_CONFIG, 0x01);
    if (ret)
        return 2;

    LOG_INF("Triggered IMU reset");
    k_msleep(500); // Datasheet says 1ms, but due to AD0 error, IMU needs more time to decide to use 0x69 address

    return ret;
}

int icm_init_power(icm_t *icm)
{
    int ret;

    ret = icm_select_bank(icm, 0);
    if (ret)
        return 1;

    ret = icm_reg_write(icm, ICM_REG_PWR_MGMT0, 0b00001111); // Temp. enabled. RC auto. Gyro LN. Accel LN.
    if (ret)
        return 2;

    LOG_INF("Configured power");

    k_usleep(300); // Must wait at least 200us after turning on sensors

    return ret;
}

int icm_init_gyro(icm_t *icm)
{
    int ret;

    ret = icm_select_bank(icm, 0);
    if (ret)
        return 1;

    // Set gyro scale range (+/- 1000dps, 1kHz)
    ret = icm_reg_write(icm, ICM_REG_GYRO_CONFIG0, 0b00100110);
    if (ret)
        return 2;

    LOG_INF("Configured gyro");

    return ret;
}

int icm_init_accel(icm_t *icm)
{
    int ret;

    ret = icm_select_bank(icm, 0);
    if (ret)
        return 1;

    // Set accel scale range (+/- 8g, 1kHz)
    ret = icm_reg_write(icm, ICM_REG_ACCEL_CONFIG0, 0b01000110);
    if (ret)
        return 2;

    LOG_INF("Configured accel.");

    return ret;
}

int icm_reg_read(icm_t *icm, uint8_t reg, uint8_t *value)
{
    return i2c_write_read_dt(icm, &reg, sizeof(reg), value, 1);
}

int icm_reg_write(icm_t *icm, uint8_t reg, uint8_t value)
{
    return i2c_reg_write_byte_dt(icm, reg, value);
}

int icm_read_gyro(icm_t *icm, float *x, float *y, float *z)
{
    raw_gyro_t g;

    // Populate g with reading
    int ret = icm_read_gyro_raw(icm, &g);
    if (ret)
        return 1;

    *x = ((float)g.x * ICM_GYRO_SCALE_FACTOR_1000DPS);
    *y = ((float)g.y * ICM_GYRO_SCALE_FACTOR_1000DPS);
    *z = ((float)g.z * ICM_GYRO_SCALE_FACTOR_1000DPS);

    return ret;
}

int icm_read_gyro_raw(icm_t *icm, raw_gyro_t *g)
{
    uint8_t buffer[6];
    uint8_t reg = ICM_REG_GYRO_DATA_X1;

    int ret = i2c_write_read_dt(icm, &reg, sizeof(reg), &buffer, ICM_GYRO_DATA_SIZE);

    if (ret)
    {
        LOG_ERR("Failed to write/read from gyro registers");
        return 1;
    }

    g->x = (int16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
    g->y = (int16_t)(((uint16_t)buffer[2] << 8) | buffer[3]);
    g->z = (int16_t)(((uint16_t)buffer[4] << 8) | buffer[5]);

    return ret;
}

int icm_read_accel(icm_t *icm, float *x, float *y, float *z)
{
    raw_accel_t a;

    // Populate a with reading
    int ret = icm_read_accel_raw(icm, &a);
    if (ret)
        return 1;

    *x = ((float)a.x * ICM_ACCEL_SCALE_FACTOR_8G);
    *y = ((float)a.y * ICM_ACCEL_SCALE_FACTOR_8G);
    *z = ((float)a.z * ICM_ACCEL_SCALE_FACTOR_8G);

    return ret;
}

int icm_read_accel_raw(icm_t *icm, raw_accel_t *a)
{
    uint8_t buffer[6];
    uint8_t reg = ICM_REG_ACCEL_DATA_X1;

    // int ret = i2c_write_read_dt(icm, &reg, sizeof(reg), &buffer, ICM_GYRO_DATA_SIZE);
    int ret = i2c_burst_read_dt(icm, ICM_REG_ACCEL_DATA_X1, &buffer, ICM_ACCEL_DATA_SIZE);

    if (ret)
    {
        LOG_ERR("Failed to write/read from gyro registers");
        return 1;
    }

    a->x = (int16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
    a->y = (int16_t)(((uint16_t)buffer[2] << 8) | buffer[3]);
    a->z = (int16_t)(((uint16_t)buffer[4] << 8) | buffer[5]);

    return ret;
}

int icm_select_bank(icm_t *icm, uint8_t bank)
{
    return icm_reg_write(icm, ICM_REG_BANK_SEL, (bank & 0b111));
}
