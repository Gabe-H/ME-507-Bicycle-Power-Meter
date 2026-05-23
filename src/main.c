/**
 * @file main.c
 * @author Gabe Haarberg
 * @brief Main source file for Bicycle Power Meter project for ME-507 Spring 2026 at Cal Poly SLO.
 * @version 0.1
 * @date 2026-05-20
 *
 * @copyright Copyright (c) 2026
 *
 *
 * Written to be run on custom nRF52832-based PCB.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

#define USE_IMU
// #define USE_ADC
#define USE_BATT_MON

#ifdef USE_IMU
// #include "icm20948.h"
// static const struct i2c_dt_spec icm = I2C_DT_SPEC_GET(ICM20948_NODE);
// #if !DT_NODE_EXISTS(ICM20948_NODE)
// #error "No icm20948 node found in devicetree"
// #endif

#include "icm40609.h"
static const struct i2c_dt_spec icm = I2C_DT_SPEC_GET(ICM40609_NODE);
#if !DT_NODE_EXISTS(ICM40609_NODE)
#error "No icm40609 node found in devicetree"
#endif

#endif /* USE_IMU */

#ifdef USE_BATT_MON
#include "max17048.h"
batt_mon_t batt_mon = I2C_DT_SPEC_GET(MAX17048_NODE);
#endif /* USE_BATT_MON */

#ifdef USE_ADC
#include "ads1220.h"
#endif /* USE_ADC */

#define DPS_TO_RPM(x) (x * 0.1666)

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

/**
 * @brief Entry function
 *
 * @return ignored by microcontroller
 */
int main(void)
{
#ifdef USE_ADC
    /* initialize ADS1220 example (non-blocking) */
    if (ads1220_ratiometric_example_init())
    {
        LOG_ERR("ADS1220 example init failed");
    }
#endif /* USE_ADC */

#ifdef USE_IMU
    if (icm_init(&icm)) // return 0 when properly configured
        return 0;
    LOG_INF("IMU configured.");

    raw_gyro_t g = {0.0, 0.0, 0.0};
#endif /* USE_IMU */

#ifdef USE_BATT_MON
    if (battery_monitor_init(&batt_mon)) // return 0 when properly configured
        return 0;
    LOG_INF("Battery monitor configured.");
#endif

    while (1)
    {
        int ret;

#ifdef USE_IMU
        float gx, gy, gz;
        ret = icm_read_gyro(&icm, &gx, &gy, &gz);

        if (ret)
        {
            LOG_WRN("Error reading gyro values");
        }
        else
        {
            LOG_INF("Read gyro values:  x: %0.3f, y: %0.3f, z: %0.3f dps", gx, gy, gz);
        }

        float ax, ay, az;
#endif /* USE_IMU */

        k_sleep(K_SECONDS(1));
    }

    return 0;
}
