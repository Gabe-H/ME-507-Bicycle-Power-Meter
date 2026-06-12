#ifndef POWER_MEAS_H
#define POWER_MEAS_H

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "app_ipc.h"

// -- Fake-a-crank calibration results
#define AXIS_SCALE_FACTOR 1.0F
#define AXIS_SLOPE AXIS_SCALE_FACTOR * 0.00096662F    // Slope for N*m estimate
#define AXIS_INTERCEPT -(AXIS_SCALE_FACTOR * 2.3005F) // Intercept for N*m estimate

#define NUM_SAMPLES 10

void power_measure_thread(void);

#endif /* POWER_MEAS_H */