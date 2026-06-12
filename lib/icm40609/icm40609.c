/**
 * @file icm40609.c
 * @brief Driver for ICM40609 IMU. Written for Bike Power Meter Project.
 * @version 0.1
 * @date 2026-05-24
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "icm40609.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(icm, LOG_LEVEL_WRN);

/**
 * @brief Initialize the IMU.
 * 1) Checks that databus is ready
 * 2) Confirms WHO_AM_I value
 * 3) Resets IMU (waits 500ms)
 * 4) Configures power settings
 * 5) Configures gyro settings
 * 6) Configures accel settings
 *
 * @param icm
 * @return int
 */
int icm_init(icm_t *icm)
{
    int ret;
    uint8_t whoami = 0;

    LOG_DBG("ICM-40609 WHO_AM_I test starting...\n");

    if (!device_is_ready(icm->bus))
    {
        LOG_ERR("I2C bus device not ready");
        return 1;
    }

    LOG_DBG("I2C bus ready, target address: 0x%02x", icm->addr);

    /* Optional: give the sensor time after power-up */
    k_msleep(50);

    ret = icm_reg_read(icm, ICM_REG_WHO_AM_I, &whoami);
    if (ret)
    {
        LOG_ERR("Failed to read WHO_AM_I (err %d)", ret);
        return 2;
    }

    LOG_DBG("WHO_AM_I = 0x%02x (expect 0x%02x)", whoami, ICM40609_WHO_AM_I_VAL);

    if (whoami == ICM40609_WHO_AM_I_VAL)
    {
        LOG_INF("ICM-40609 detected");
    }
    else
    {
        LOG_ERR("Unexpected WHO_AM_I value");
        return 3;
    }

    ret = icm_select_bank(icm, 0);
    if (ret)
    {
        LOG_WRN("Failed to select bank before resetting board");
        return ret;
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

/**
 * @brief Resets the IMU. Sleeps for 500ms before exiting to allow for wake-up
 * NOTE: 500ms was found via trial and error.
 *
 * @param icm
 * @return int
 */
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

/**
 * @brief Configures power settings on IMU
 * (At time of writing): 0b00001111);
 * 7:6: Reserved - (00)
 *   5: TEMP_DIS - (0) Temperature sensor enabled
 *   4: IDLE - (0) Entire chip goes to sleep if gyro/accel turned off
 * 3:2: GYRO_MODE - (11) Gyro in Low-Noise mode
 * 1:0: ACCEL_MODE - (11) Accel in Low-Noise mode
 *
 * @param icm
 * @return int
 */
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

/**
 * @brief Configures the gyro on the IMU
 * (At time of writing): 0b00100110
 * 7:5: GYRO_FS_SEL - (001) +/-1000dps
 *   4: Reserved - (0)
 * 3:0: GYRO_ODR - (0110) (default) 1kHz
 *
 * @param icm
 * @return int
 */
int icm_init_gyro(icm_t *icm)
{
    int ret;

    ret = icm_select_bank(icm, 0);
    if (ret)
        return 1;

    ret = icm_reg_write(icm, ICM_REG_GYRO_CONFIG0, ICM_GYRO_CONFIG0_VALUE);
    if (ret)
        return 2;

    LOG_INF("Configured gyro");

    return ret;
}

/**
 * @brief Configures the accelerometer on the IMU
 * (At time of writing): 0b01000110
 * 7:5: ACCEL_FS_SEL - (010) +/-8g
 *   4: Reserved
 * 3:0: ACCEL_ODR - (0110) (default) 1kHz
 *
 * @param icm
 * @return int
 */
int icm_init_accel(icm_t *icm)
{
    int ret;

    ret = icm_select_bank(icm, 0);
    if (ret)
        return 1;

    // Set accel scale range (+/- 8g, 1kHz)
    ret = icm_reg_write(icm, ICM_REG_ACCEL_CONFIG0, ICM_ACCEL_CONFIG0_VALUE);
    if (ret)
        return 2;

    LOG_INF("Configured accel.");

    return ret;
}

/**
 * @brief Read value from a register on IMU. Result is always 1 byte and placed in `&value`.
 * Register bank *must* already be selected
 *
 * @param icm
 * @param reg Register to be accessed (1 byte)
 * @param value Memory location to store result in (1 byte)
 * @return int
 */
int icm_reg_read(icm_t *icm, uint8_t reg, uint8_t *value)
{
    return i2c_write_read_dt(icm, &reg, sizeof(reg), value, 1);
}

/**
 * @brief Write to a register on the IMU. Register bank *must* already be selected.
 *
 * @param icm
 * @param reg Register to write to (1 byte)
 * @param value Value to write to register (1 byte)
 * @return int
 */
int icm_reg_write(icm_t *icm, uint8_t reg, uint8_t value)
{
    return i2c_reg_write_byte_dt(icm, reg, value);
}

/**
 * @brief Read gyro data scaled properly. Each axis is returned into the `x` `y` and `z`
 * float pointers in units of [dps].
 *
 * @param icm
 * @param x Angular velocity in X-axis [dps]
 * @param y Angular velocity in Y-axis [dps]
 * @param z Angular velocity in Z-axis [dps]
 * @return int
 */
int icm_read_gyro(icm_t *icm, float *x, float *y, float *z)
{
    raw_gyro_t g;

    // Populate g with reading
    int ret = icm_read_gyro_raw(icm, &g);
    if (ret)
        return 1;

    *x = ((float)g.x * ICM_GYRO_SCALE_FACTOR);
    *y = ((float)g.y * ICM_GYRO_SCALE_FACTOR);
    *z = ((float)g.z * ICM_GYRO_SCALE_FACTOR);

    return ret;
}

/**
 * @brief Reads all gyro data (6 bytes total, 2 bytes per axis). Result goes into `&g`.
 *
 * @param icm
 * @param g raw_gyro_t memory location to store result into.
 * @return int
 */
int icm_read_gyro_raw(icm_t *icm, raw_gyro_t *g)
{
    uint8_t buffer[6];
    uint8_t reg = ICM_REG_GYRO_DATA_X1;

    int ret = i2c_write_read_dt(icm, &reg, sizeof(reg), buffer, ICM_GYRO_DATA_SIZE);

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

/**
 * @brief Read gyro data scaled properly. Each axis is returned into the `x` `y` and `z`
 * float pointers in units of [g]'s.
 *
 * @param icm
 * @param x Acceleration in X-axis [g]
 * @param y Acceleration in Y-axis [g]
 * @param z Acceleration in Z-axis [g]
 * @return int
 */
int icm_read_accel(icm_t *icm, float *x, float *y, float *z)
{
    raw_accel_t a;

    // Populate a with reading
    int ret = icm_read_accel_raw(icm, &a);
    if (ret)
        return 1;

    *x = ((float)a.x * ICM_ACCEL_SCALE_FACTOR);
    *y = ((float)a.y * ICM_ACCEL_SCALE_FACTOR);
    *z = ((float)a.z * ICM_ACCEL_SCALE_FACTOR);

    return ret;
}

/**
 * @brief Reads all accel data (6 bytes total, 2 bytes per axis). Result goes into `&a`.
 *
 * @param icm
 * @param g raw_accel_t memory location to store result into.
 * @return int
 */
int icm_read_accel_raw(icm_t *icm, raw_accel_t *a)
{
    uint8_t buffer[6];

    int ret = i2c_burst_read_dt(icm, ICM_REG_ACCEL_DATA_X1, buffer, ICM_ACCEL_DATA_SIZE);

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

/**
 * @brief Select which register bank to access during next read/write command(s).
 *
 * @param icm
 * @param bank Register bank (0-4)
 * @return int
 */
int icm_select_bank(icm_t *icm, uint8_t bank)
{
    if (bank > 4)
    {
        return 1;
    }

    return icm_reg_write(icm, ICM_REG_BANK_SEL, (bank & 0b111));
}
