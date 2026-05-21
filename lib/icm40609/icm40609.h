#ifndef __ICM40609_H
#define __ICM40609_H

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>

/* ICM-40609 registers */
#define ICM40609_WHO_AM_I_VAL 0xEA

#define ICM_REG_WHO_AM_I 0x00
#define ICM_GYRO_XOUT_H 0x33
#define ICM_GYRO_SIZE 6
#define ICM_REG_BANK_SEL 0x7F
#define ICM_REG_PWR_MGMT_1 0x06
#define ICM_REG_PWR_MGMT_2 0x07
#define ICM_REG_GYRO_CONFIG_1 0x01

#define ICM_GYRO_RANGE_250 0x00 << 1
#define ICM_GYRO_RANGE_500 0x01 << 1
#define ICM_GYRO_RANGE_1000 0x02 << 1
#define ICM_GYRO_RANGE_2000 0x03 << 1

/* Devicetree node for the child device under i2c0 */
#define ICM40609_NODE DT_NODELABEL(icm40609)

/**
 * @brief Struct containing relevant raw data from gyro
 *
 * @param x signed 16-bit angular acceleration in X dir
 * @param y signed 16-bit angular acceleration in Y dir
 * @param z signed 16-bit angular acceleration in Z dir
 * @param scale scale factor to multiply raw int16_t to
 * get actual measured value (dps).
 *
 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
    float scale;
} raw_gyro_t;

/**
 * @brief Struct containing relevant raw data from accelerometer
 *
 * @param x signed 16-bit acceleration in X dir
 * @param y signed 16-bit acceleration in Y dir
 * @param z signed 16-bit acceleration in Z dir
 * @param scale scale factor to multiply raw int16_t to
 * get actual measured acceleration (m/s).
 *
 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
    float scale;
} raw_accel_t;

// static const struct i2c_dt_spec icm_t;
typedef const struct i2c_dt_spec icm_t;

int icm_init(icm_t *icm);

int icm_init_gyro(icm_t *icm);

int icm_reg_read(icm_t *icm, uint8_t reg, uint8_t *value);

int icm_read_gyro_raw(icm_t *icm, raw_gyro_t *g);

int icm_select_bank(icm_t *icm, uint8_t bank);

#endif /* __ICM40609_H*/