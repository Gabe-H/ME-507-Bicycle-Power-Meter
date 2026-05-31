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
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/input/input.h>

// Peripherals
#include "icm40609.h"
#include "max17048.h"

#include <math.h>
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
#define IMU_DATA_SLAB_NUM_BLOCKS 20
#define IMU_DATA_SLAB_ALIGNMENT 8 // Need 8 because data is dealing with int64_t

K_FIFO_DEFINE(printk_fifo);

/** Axis data FIFO for queuing mapped axis values from callback **/
struct axis_data
{
    int32_t value;
    int channel;
};
K_FIFO_DEFINE(axis_fifo);

/** Structs */
struct imu_data_t
{
    float gyro_z;  // Gyro angular vel in Z-dir [ dps ]
    float accel_x; // Acceleration in X-dir [ g ]
    float accel_y; // Acceleration in Y-dir [ g ]
    int64_t ts;    // Timestamp. Millis since boot [ ms ]
};

K_MEM_SLAB_DEFINE(imu_data_slab,
                  sizeof(struct imu_data_t),
                  IMU_DATA_SLAB_NUM_BLOCKS,
                  IMU_DATA_SLAB_ALIGNMENT);

K_FIFO_DEFINE(imu_fifo);

// struct torque_data_t
// {
//     float torque;     // Torque measured by loadcell [ N*m ]
//     int64_t ts; // Timestamp. Millis since boot [ ms ]
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
        static int ret;
        static float gx, gy, gz;
        static float ax, ay, az;

        ret = icm_read_gyro(&icm, &gx, &gy, &gz);
        ret = icm_read_accel(&icm, &ax, &ay, &az);

        if (ret)
        {
            LOG_WRN("Error reading gyro/accel values");
        }
        else
        {
            struct imu_data_t *data;

            if (k_mem_slab_alloc(&imu_data_slab, (void **)&data, K_MSEC(100)) == 0)
            {
                data->accel_x = ax;
                data->accel_y = ay;
                data->gyro_z = gz;
                data->ts = k_uptime_get();

                k_fifo_put(&imu_fifo, data);
            }
            else
            {
                LOG_WRN("Warning: Couldn't allocate space on slab");
            }
            // char *mem_ptr = k_malloc(100);
            // sprintf(mem_ptr, "Read values:  x: %0.2f, y: %0.2f, z: %0.2f dps | x: %0.2f, y: %0.2f, z: %0.2f g",
            //         (double)gx, (double)gy, (double)gz,
            //         (double)ax, (double)ay, (double)az);
            // k_fifo_put(&printk_fifo, mem_ptr);

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
        return;
    }

    while (1)
    {
        // uint8_t pct = battery_monitor_read_soc(&batt_mon);

        // char *mem_ptr = k_malloc(32);
        // sprintf(mem_ptr, "Battery SOC: %d%%", pct);
        // k_fifo_put(&printk_fifo, mem_ptr);

        uint16_t volts = battery_monitor_read_voltage(&batt_mon); // Value is returned in millivolts

        char *mem_ptr = k_malloc(32);
        sprintf(mem_ptr, "Battery voltage: %dmV", volts);
        k_fifo_put(&printk_fifo, mem_ptr);

        k_sleep(K_SECONDS(5));
    }
}

/**
 * @brief Input event callback (called by the Zephyr input subsystem)
 *
 * Receives fully-processed events from the analog-axis driver.
 * The driver already handles all scaling, deadzone, and clamping via DTS.
 * We just queue the mapped value for printing.
 */
static void input_evt_cb(struct input_event *evt, void *user_data)
{
    // ARG_UNUSED(dev);
    ARG_UNUSED(user_data);

    /* Filter for Y axis absolute position events */
    if (evt->type == INPUT_EV_ABS && evt->code == INPUT_ABS_Y)
    {
        struct axis_data *axis_msg = k_malloc(sizeof(struct axis_data));
        if (axis_msg)
        {
            axis_msg->channel = 0;
            axis_msg->value = evt->value; /* Already scaled/clamped by driver */
            k_fifo_put(&axis_fifo, axis_msg);
        }
    }
}

/**
 * @brief RTOS Task for reading values from the ADC
 *
 * Registers the analog-axis callback, then continuously reads mapped axis
 * values from the axis_fifo queue and forwards them to RTT via printk_fifo.
 */
void adc_task(void)
{

    /* Main loop: read axis values from callback queue and print */
    while (1)
    {
        /* Get axis data from FIFO with timeout to keep responsiveness */
        struct axis_data *axis_msg = k_fifo_get(&axis_fifo, K_FOREVER);
        if (axis_msg)
        {
            char *mem_ptr = k_malloc(64);
            if (mem_ptr)
            {
                sprintf(mem_ptr, "Axis ch%d: %d", axis_msg->channel, (int)axis_msg->value);
                k_fifo_put(&printk_fifo, mem_ptr);
            }
            k_free(axis_msg);
        }
    }
}

/**
 * @brief Angular velocity and angle position processing and filtering
 *
 */
void angle_task(void)
{
    while (1)
    {
        struct imu_data_t *data = k_fifo_get(&imu_fifo, K_FOREVER);

        k_mem_slab_free(&imu_data_slab, (void *)data);

        float angle = 0;

        if (data->accel_x == 0)
        {
            angle = 1.5708;
        }
        else
        {
            angle = atanf(data->accel_y / data->accel_x);

            // angle += 1.5708; // Offset due to IMU placement

            // Depending on signage of x component, add 180deg offset
            // to ensure output angle goes from 0-360 (rather than 0-180)
            // if (data->accel_x < 0)
            //     angle += 3.1415;
        }

        char *mem_ptr = k_malloc(50);

        // sprintf(mem_ptr, "Angle: %f", angle);
        sprintf(mem_ptr, "X: %.2f, Y: %.2f", data->accel_x, data->accel_y);

        k_fifo_put(&printk_fifo, mem_ptr);
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
K_THREAD_DEFINE(adc_task_id, STACKSIZE, adc_task, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(angle_task_id, STACKSIZE, angle_task, NULL, NULL, NULL, 6, 0, 0);
K_THREAD_DEFINE(rtt_task_id, STACKSIZE, rtt_task, NULL, NULL, NULL, 7, 0, 0);

/** Register ADC axis callback **/
INPUT_CALLBACK_DEFINE(
    // DEVICE_DT_GET(DT_NODELABEL(anin0)),
    NULL,
    input_evt_cb,
    NULL);