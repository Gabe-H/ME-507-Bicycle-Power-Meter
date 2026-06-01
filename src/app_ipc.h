#ifndef APP_IPC_H
#define APP_IPC_H

#include <zephyr/kernel.h>

/** Threading configuration**/
#define STACKSIZE 1024              // Stack area used by each thread
#define IMU_DATA_SLAB_NUM_BLOCKS 20 // Number of blocks to allocate for IMU memory slab
#define IMU_DATA_SLAB_ALIGNMENT 8   // Need 8 because data is dealing with int64_t
#define AXIS_DATA_SLAB_NUM_BLOCKS 16
#define AXIS_DATA_SLAB_ALIGNMENT 8

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
 * @param gyro_z Gyro angular vel in Z-dir [ dps ]
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

extern struct k_fifo axis_fifo;
extern struct k_mem_slab axis_data_slab;
extern struct k_fifo imu_fifo;
extern struct k_fifo printk_fifo;
extern struct k_mem_slab imu_data_slab;

#endif /* APP_IPC_H */