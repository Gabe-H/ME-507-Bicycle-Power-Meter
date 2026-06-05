#ifndef POWER_MEAS_H
#define POWER_MEAS_H

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "app_ipc.h"

// -- Fake-a-crank calibration results
#define AXIS_SLOPE 10 * 0.00096662 // Slope for N*m estimate
#define AXIS_INTERCEPT -2.3005     // Intercept for N*m estimate

#define NUM_SAMPLES 10

void power_measure_thread(void);

#endif /* POWER_MEAS_H */