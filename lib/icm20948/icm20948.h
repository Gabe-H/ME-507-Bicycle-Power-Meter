#ifndef __ICM20948_H
#define __ICM20948_H

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>

/* ICM-20948 registers */
#define ICM20948_REG_WHO_AM_I 0x00
#define ICM20948_WHO_AM_I_VAL 0xEA
#define ICM20948_GYRO_XOUT_H 0x33
#define ICM20948_GYRO_SIZE 6
#define ICM20948_REG_BANK_SEL 0x7F
#define ICM20948_REG_PWR_MGMT_1 0x06
#define ICM20948_REG_PWR_MGMT_2 0x07
#define ICM20948_REG_GYRO_CONFIG_1 0x01

#define ICM20948_GYRO_RANGE_250 0x00 << 1
#define ICM20948_GYRO_RANGE_500 0x01 << 1
#define ICM20948_GYRO_RANGE_1000 0x02 << 1
#define ICM20948_GYRO_RANGE_2000 0x03 << 1

/* Devicetree node for the child device under i2c0 */
#define ICM20948_NODE DT_NODELABEL(icm20948)

#if !DT_NODE_EXISTS(ICM20948_NODE)
#error "No icm20948 node found in devicetree"
#endif

/**
 * @brief Struct containing relevant raw data from gyro
 *
 * @param x signed 16-bit angular acceleration in X dir
 * @param y signed 16-bit angular acceleration in X dir
 * @param z signed 16-bit angular acceleration in X dir
 * @param scale scale factor to multiply raw int16_t to
 * get actual measured value.
 *
 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
    float scale;
} raw_gyro_t;

int icm20948_init(void);

int icm20948_init_gyro(void);

int icm20948_reg_read(uint8_t reg, uint8_t *value);

int icm20948_read_gyro_raw(raw_gyro_t *g);

int icm20948_select_bank(uint8_t bank);

#endif /* __ICM20948_H*/