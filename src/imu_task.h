#ifndef IMU_TASK_H
#define IMU_TASK_H

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
// #include <zephyr/device.h>
// #include <zephyr/devicetree.h>
#include "icm40609.h"
#include "app_ipc.h"

#if !DT_NODE_EXISTS(ICM40609_NODE)
#error "No icm40609 node found in devicetree"
#endif

void imu_task(void);

#endif /* IMU_TASK_H */