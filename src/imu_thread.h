#ifndef IMU_THREAD_H
#define IMU_THREAD_H

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "icm40609.h"
#include "app_ipc.h"

#if !DT_NODE_EXISTS(ICM40609_NODE)
#error "No icm40609 node found in devicetree"
#endif

#define IMU_PERIOD 25 // IMU data update period in milliseconds

static void imu_thread(void);

#endif /* IMU_THREAD_H */