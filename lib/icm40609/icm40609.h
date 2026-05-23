#ifndef __ICM40609_H
#define __ICM40609_H

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>

/* ICM-40609 registers */
#define ICM40609_WHO_AM_I_VAL 0x3B

// Registers in Bank 0
#define ICM_REG_DEVICE_CONFIG 0x11 // 7:5 Reserved
                                   // 4   SPI_MODE (leave default)
                                   // 3:1 Reserved
                                   // 0   SOFT_RESET_CONFIG (0: normal [dflt], 1: enable reset (wait 1ms after write))
#define ICM_REG_ACCEL_DATA_X1 0x1F // Starting register of 6 accel data bytes (X1 X0 Y1 Y0 Z1 Z0)
                                   //
#define ICM_REG_GYRO_DATA_X1 0x25  // Starting register of 6 gyro data bytes (X1 X0 Y1 Y0 Z1 Z0)
                                   //
#define ICM_REG_PWR_MGMT0 0x4E     // 7:6 Reserved
                                   // 5:  TEMP_DIS (0: enabled, 1: disabled) [1]
                                   // 4:  IDLE (1: RC-oscll. always on, 0: RC-oscll. turns off with gyro/accel pwr) [0]
                                   // 3:2 GYRO_MODE (00: off, 01: standby, 10: rsvd, 11: low-noise mode) [11]
                                   // 1:0 ACCEL_MODE (00: off, 01: off, 10: low-power, 11: low-noise) [11]
#define ICM_REG_GYRO_CONFIG0 0x4F  // 7:5 GYRO_FS_SEL (000-111: 2000, 1000, 500, 250, 125, 62.5, 31.25, 15.625dps) [001 - 1000dps]
                                   // 4:  Reserved
                                   // 3:0 GYRO_ODR (0000-1111: Rsrvd, 32k, 16k, 8k, 4k, 2k, 1k, 200, 100, 50, 25, 12.5, R, R, R, 500) [0110 - 100Hz]
#define ICM_REG_ACCEL_CONFIG0 0x50 // 7:5 ACCEL_FS_SEL (000-111: 32g 16g, 8g, 4g, R, R, R, R) [010 - 8g]
                                   // 4 Reserved
                                   // 3:0 ACCEL_ODR (0000-1111 Rsrvd, 32k, 16k, 8k, 4k, 2k, 1k, 200, 100, 50, 25, 12.5, 6.25, 3.125, 1.5625, 500) [0110 - 1k]
#define ICM_REG_WHO_AM_I 0x75
#define ICM_REG_BANK_SEL 0x76

#define ICM_ACCEL_DATA_SIZE 6
#define ICM_GYRO_DATA_SIZE 6

#define ICM_GYRO_SCALE_FACTOR_1000DPS (0.03049f) // 1/32.8
#define ICM_ACCEL_SCALE_FACTOR_8G (0.00024414f)  // 1/4096 LSB/g

// Registers in Bank 1
#define ICM_REG_SENSOR_CONFIG0 0x03      // 7:6 Reserved
                                         // 5 ZG_DISABLE (0: on, 1: disabled)
                                         // 4 YG_DISABLE (0: on, 1: disabled)
                                         // 3 ZG_DISABLE (0: on, 1: disabled)
                                         // 2 ZA_DISABLE (0: on, 1: disabled)
                                         // 1 YA_DISABLE (0: on, 1: disabled)
                                         // 0 XA_DISABLE (0: on, 1: disabled)
#define ICM_REG_GYRO_CONFIG_STATIC2 0x0B // 7:2 Reserved
                                         // 1   GYRO_AAG_DIS (0: enable anti-alias/LP filter, 1: disable [dflt])
                                         // 0   GYRO_NF_DIS (0: enable notch filter, 1: disable [dflt])

// Registers in Bank 2
// Accelerometer filtering configuration

// NOTE: NO REGISTERS IN BANK 3

// Registers in Bank 4
// Gyro offsets - Max is +/- 64dps. Resolution is 1/32 dps
// Accel offsets - Max is +/- 1g. Resolution is 0.5mg
#define OFFSET_USER0 0x77 // 7:0 (low bits of X-gyro offset)
                          //
#define OFFSET_USER1 0x78 // 7:4 (upper bits of Y-gyro offset)
                          // 3:0 (upper bits of X-gyro offset)
#define OFFSET_USER2 0x79 // 7:0 (low bits of Y-gyro offset)
                          //
#define OFFSET_USER3 0x7A // 7:0 (low bits of Z-gyro offset)
                          //
#define OFFSET_USER4 0x7B // 7:4 (upper bits of X-accel offset)
                          // 3:0 (upper bits of Z-gyro offset)
#define OFFSET_USER5 0x7C // 7:0 (lower bits of X-accel offset)
                          //
#define OFFSET_USER6 0x7D // 7:0 (lower bits of Y-accel offset)
                          //
#define OFFSET_USER7 0x7E // 7:4 (upper bits of Z-accel offset)
                          // 3:0 (upper bits of Y-accel offset)
#define OFFSET_USER8 0x7F // 7:0 (lower bits of Z-accel offset)

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
} raw_accel_t;

typedef const struct i2c_dt_spec icm_t;

int icm_init(icm_t *icm);

int icm_reset(icm_t *icm);

int icm_init_power(icm_t *icm);

int icm_init_gyro(icm_t *icm);

int icm_init_accel(icm_t *icm);

int icm_set_gyro_offset(icm_t *icm, float x, float y, float z);

int icm_set_accel_offset(icm_t *icm, float x, float y, float z);

int icm_reg_write(icm_t *icm, uint8_t reg, uint8_t value);

int icm_reg_read(icm_t *icm, uint8_t reg, uint8_t *value);

int icm_read_gyro(icm_t *icm, float *x, float *y, float *z);

int icm_read_gyro_raw(icm_t *icm, raw_gyro_t *g);

int icm_read_accel(icm_t *icm, float *x, float *y, float *z);

int icm_read_accel_raw(icm_t *icm, raw_accel_t *a);

// Select from register banks 0-4
int icm_select_bank(icm_t *icm, uint8_t bank);

#endif /* __ICM40609_H*/