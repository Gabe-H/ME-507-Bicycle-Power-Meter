#ifndef APP_IPC_H
#define APP_IPC_H

#include <zephyr/kernel.h>

/** Threading configuration**/
#define STACKSIZE 1024             // Stack area used by each thread
#define IMU_DATA_SLAB_NUM_BLOCKS 5 // Number of blocks to allocate for IMU memory slab
#define IMU_DATA_SLAB_ALIGNMENT 8  // Need 8 because data is dealing with int64_t
#define AXIS_DATA_SLAB_NUM_BLOCKS 5
#define AXIS_DATA_SLAB_ALIGNMENT 8
#define BLE_DATA_SLAB_NUM_BLOCKS 5
#define BLE_DATA_SLAB_ALIGNMENT 8

/** Thread coordination */
#define ADC_THREAD_READY BIT(0)
#define BATTERY_THREAD_READY BIT(1)
#define BLE_THREAD_READY BIT(2)
#define FILTER_THREAD_READY BIT(3)
#define IMU_THREAD_READY BIT(4)
#define RTT_THREAD_READY BIT(5)
#define PWR_THREAD_READY BIT(6)

#define ALL_READY (/* ADC_THREAD_READY | */                 \
                   BATTERY_THREAD_READY |                   \
                   BLE_THREAD_READY | FILTER_THREAD_READY | \
                   IMU_THREAD_READY | RTT_THREAD_READY |    \
                   PWR_THREAD_READY)

#define START_BIT BIT(7) // START BIT is 1 higher than highest thread bit

/** IMU Timing */
#define IMU_PERIOD 50 // IMU data update period in milliseconds

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
 * @param accel_x Acceleration in X-dir [ g ]
 * @param accel_y Acceleration in Y-dir [ g ]
 * @param accel_z Acceleration in Z-dir [ g ]
 * @param gyro_x  Gyro angular vel in X-dir [ dps ]
 * @param gyro_y  Gyro angular vel in Y-dir [ dps ]
 * @param gyro_z  Gyro angular vel in Z-dir [ dps ]
 * @param ts Timestamp. Millis since boot [ ms ]
 */
struct imu_data_t
{
    float accel_x; // Acceleration in X-dir [ g ]
    float accel_y; // Acceleration in Y-dir [ g ]
    float accel_z; // Acceleration in Z-dir [ g ]
    float gyro_x;  // Gyro angular vel in X-dir [ dps ]
    float gyro_y;  // Gyro angular vel in Y-dir [ dps ]
    float gyro_z;  // Gyro angular vel in Z-dir [ dps ]
    int64_t ts;    // Timestamp. Millis since boot [ ms ]
};

/**
 * @brief Struct containing data to be sent via BLE
 *
 * @param power Estimated power output at cranks [W]
 */
struct ble_data_t
{
    uint16_t initialized; // Honestly just a place holder to make this struct of size 8U for slab alignment
    uint16_t power;       // Estimated power output [W]
    uint16_t crank_index;
    uint16_t crank_event_time;
};

/**
 * @brief Struct containing outputs from Kalman filter
 *
 */
struct filter_data_t
{
    float theta;               // Estimated angle [rad]
    float omega;               // Estimated omega [rad/s]
    uint16_t crank_index;      // Rotation index (used for CPS)
    uint16_t crank_event_time; // Rotation time (used for CPS)
    int64_t ts;                // Uptime timestamps [ms]
};

extern struct k_event thread_sync_event;
extern struct k_fifo imu_fifo;
extern struct k_mem_slab imu_data_slab;
extern struct k_fifo ble_fifo;
extern struct k_mem_slab ble_data_slab;
extern struct k_fifo filter_fifo;
extern struct k_mem_slab filter_data_slab;
extern struct k_fifo printk_fifo;

extern struct k_mutex axis_latest_lock;
// extern struct k_sem axis_latest_ready;
extern int32_t axis_latest_value;

#endif /* APP_IPC_H */