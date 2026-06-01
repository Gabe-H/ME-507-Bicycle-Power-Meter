#ifndef APP_IPC_H
#define APP_IPC_H

#include <zephyr/kernel.h>

/** Threading configuration**/
#define STACKSIZE 1024              // Stack area used by each thread
#define IMU_DATA_SLAB_NUM_BLOCKS 20 // Number of blocks to allocate for IMU memory slab
#define IMU_DATA_SLAB_ALIGNMENT 8   // Need 8 because data is dealing with int64_t
#define AXIS_DATA_SLAB_NUM_BLOCKS 16
#define AXIS_DATA_SLAB_ALIGNMENT 8
#define BLE_DATA_SLAB_NUM_BLOCKS 16
#define BLE_DATA_SLAB_ALIGNMENT 4

/** Thread coordination */
#define ADC_THREAD_READY BIT(0)
#define BATTERY_THREAD_READY BIT(1)
#define BLE_THREAD_READY BIT(2)
#define FILTER_THREAD_READY BIT(3)
#define IMU_THREAD_READY BIT(4)
#define RTT_THREAD_READY BIT(5)

#define ALL_READY (ADC_THREAD_READY | BATTERY_THREAD_READY | \
                   BLE_THREAD_READY | FILTER_THREAD_READY |  \
                   IMU_THREAD_READY | RTT_THREAD_READY)

#define START_BIT BIT(6) // START BIT is 1 higher than highest thread bit

/** Axis data FIFO for queuing mapped axis values from callback **/

/**
 * @brief Struct containing value of strain gauge
 *
 */
struct axis_data
{
    int32_t value;
    int channel;
};

/**
 * @brief Struct containing relevant info from IMU
 *
 * @param gyro_z  Gyro angular vel in Z-dir [ dps ]
 * @param accel_x Acceleration in X-dir [ g ]
 * @param accel_y Acceleration in Y-dir [ g ]
 * @param ts Timestamp. Millis since boot [ ms ]
 */
struct imu_data_t
{
    float gyro_z;  // Gyro angular vel in Z-dir [ dps ]
    float accel_x; // Acceleration in X-dir [ g ]
    float accel_y; // Acceleration in Y-dir [ g ]
    int64_t ts;    // Timestamp. Millis since boot [ ms ]
};

/**
 * @brief Struct containing data to be sent via BLE
 *
 * @param power Estimated power output at cranks [W]
 * @param rpm   Estimated crank speed [rpm]
 */
struct ble_data_t
{
    uint16_t power; // Estimated power output [W]
    uint16_t rpm;   // Estimated crank speed [RPM]
};

extern struct k_event thread_sync_event;
extern struct k_fifo axis_fifo;
extern struct k_mem_slab axis_data_slab;
extern struct k_fifo imu_fifo;
extern struct k_mem_slab imu_data_slab;
extern struct k_fifo ble_fifo;
extern struct k_mem_slab ble_data_slab;
extern struct k_fifo printk_fifo;

#endif /* APP_IPC_H */