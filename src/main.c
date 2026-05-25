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

/** BEGIN INCLUDES **/
// System
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

// Peripherals
#include "icm40609.h"
#include "max17048.h"
// #include "ads1220.h"
/** END INCLUDES **/

/** BEGIN PERIPHERAL CONFIGURATION **/

#if !DT_NODE_EXISTS(ICM40609_NODE)
#error "No icm40609 node found in devicetree"
#endif
icm_t icm = I2C_DT_SPEC_GET(ICM40609_NODE);

#if !DT_NODE_EXISTS(MAX17048_NODE)
#error "No MAX17048 battery monitor node found in devicetree"
#endif
batt_mon_t batt_mon = I2C_DT_SPEC_GET(MAX17048_NODE);
/** END PERIPHERAL CONFIGURATION */

#define DPS_TO_RPM(x) (x * 0.1666)

/** Logger configuration **/
LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

/** Threading configuration**/
#define STACKSIZE 1024 // Stack area used by each thread

K_FIFO_DEFINE(printk_fifo);

/** Structs */
// struct imu_data_t
// {
//     float gyro_z;     // Gyro angular vel in Z-dir [ dps ]
//     float accel_x;    // Acceleration in X-dir [ g ]
//     float accel_y;    // Acceleration in Y-dir [ g ]
//     uint64_t boot_ms; // Milliseconds since boot of measurement [ ms ]
// };

// struct torque_data_t
// {
//     float torque;     // Torque measured by loadcell [ N*m ]
//     uint64_t boot_ms; // Milliseconds since boot of measurement [ ms ]
// };

/**
 * @brief RTOS Task for the IMU
 *
 */
void imu_task(void)
{
    if (icm_init(&icm)) // return 0 when properly configured
    {
        return;
    }

    LOG_INF("IMU configured.");

    while (1)
    {
        int ret;

        float gx, gy, gz;
        float ax, ay, az;

        ret = icm_read_gyro(&icm, &gx, &gy, &gz);
        ret = icm_read_accel(&icm, &ax, &ay, &az);

        if (ret)
        {
            LOG_WRN("Error reading gyro/accel values");
        }
        else
        {
            char *mem_ptr = k_malloc(100);
            sprintf(mem_ptr, "Read values:  x: %0.2f, y: %0.2f, z: %0.2f dps | x: %0.2f, y: %0.2f, z: %0.2f g",
                    (double)gx, (double)gy, (double)gz,
                    (double)ax, (double)ay, (double)az);
            k_fifo_put(&printk_fifo, mem_ptr);

            // LOG_INF("Read values:  x: %0.2f, y: %0.2f, z: %0.2f dps | x: %0.2f, y: %0.2f, z: %0.2f g", gx, gy, gz, ax, ay, az);
        }

        k_sleep(K_SECONDS(1));
    }
}

/**
 * @brief RTOS Task for battery monitoring
 *
 */
void battery_monitor_task(void)
{

    if (battery_monitor_init(&batt_mon)) // return 0 when properly configured
    {
        // return 0;
        while (1) // Do nothing else in this task
            k_sleep(K_FOREVER);
    }

    while (1)
    {
        uint8_t pct = battery_monitor_read_soc(&batt_mon);

        char *mem_ptr = k_malloc(32);
        sprintf(mem_ptr, "Battery SOC: %d%%", pct);
        k_fifo_put(&printk_fifo, mem_ptr);

        k_sleep(K_SECONDS(5));
    }
}

/**
 * @brief RTOS Task for sending messages to RTT terminal
 *
 * Messages are sent as a char array pointer to the FIFO buffer
 */
void rtt_task(void)
{
    while (1)
    {
        char *rx_data = k_fifo_get(&printk_fifo, K_FOREVER);

        puts(rx_data);
        k_free(rx_data);
    }
}

/** Thread creation **/
K_THREAD_DEFINE(imu_task_id, STACKSIZE, imu_task, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(battery_task_id, STACKSIZE, battery_monitor_task, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(rtt_task_id, STACKSIZE, rtt_task, NULL, NULL, NULL, 7, 0, 0);
