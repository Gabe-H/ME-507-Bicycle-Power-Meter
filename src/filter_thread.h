#ifndef FILTER_THREAD_H
#define FILTER_THREAD_H

#include <zephyr/kernel.h>
#include "app_ipc.h"

#include <eekf/eekf.h>
#include <math.h>
#include <stdio.h>

#define NX 5 // State dimension
#define NZ 3 // Measurement dimension
#define NU 1 // Input dimension; use it to pass dt

#define M_PI 3.1415926536F
#define M_TWO_PI_F 6.2831853071F

#define MIN_FORWARD_OMEGA 0.5 // Min [rad/s] that crank must be going

#define M_GRAVITY 9.80665F

#define DPS_TO_RAD_S 0.017453F

/* Keep the EKF angle locally continuous, but do not let the float grow forever.
 * Recenter by whole revolutions so sin/cos physics and CPS revolution counting
 * remain unchanged.
 */
#ifndef THETA_RECENTER_REVS
#define THETA_RECENTER_REVS (100.0F)
#endif

#define THETA_RECENTER_RAD ((eekf_value)THETA_RECENTER_REVS * M_TWO_PI_F)

/**
 *  State vector, x
 *
 *  x[0] = theta     - Crank angle [rad]
 *  x[1] = omega     - Crank angular velocity [rad/s]
 *  x[2] = bg        - Gyro bias [rad/s]
 *  x[3] = bax       - Accel x bias [m/s^2]
 *  x[4] = bay       - Accel y bias [m/s^2]
 *
 */

/**
 * Measurement vector, z
 *
 *  z[0] = gyro_z     - Measured crank-axis gyro rate [rad/s]
 *  z[1] = accel_x    - measured in-plane accel x [m/s^2]
 *  z[2] = accel_y    - measured in-plane accel y [m/s^2]
 */

typedef struct
{
    eekf_value g; // gravity [m/s^2]

    eekf_value rx; // IMU x-position from crank axis [m]
    eekf_value ry; // IMU y-position from crank axis [m]

    eekf_value sigma_alpha; // Angular acceleration process noise [rad/s^2]
    eekf_value sigma_bg;    // Gyro bias random walk [rad/s/sqrt(s)]
    eekf_value sigma_ba;    // Accel bias random walk [m/s^2/sqrt(s)]
} crank_ekf_params_t;

/**
 * @brief External state of the system. Used to calculate crank-rev index
 * and crank_event_time for CPS BLE Service
 *
 */
static struct cps_crank_state
{
    bool initialized;

    float theta_prev_wrapped;
    float theta_prev_unwrapped;
    float theta_unwrapped;

    uint64_t t_prev_us;

    int32_t rev_index; // internal signed revolution index

    uint16_t crank_revs;       // CPS cumulative crank revolutions
    uint16_t crank_event_time; // CPS event time, 1/1024 s units
} crank_state;

static eekf_context ekf_ctx;

static crank_ekf_params_t ekf_params;

static uint64_t previous_ts; // Previous uptime in ms

static void crank_ekf_init(void);

static void crank_ekf_update(eekf_value, eekf_value, eekf_value, eekf_value);

static void filter_thread(void);

static eekf_return transition(eekf_mat *xp, eekf_mat *Jf, eekf_mat const *x,
                              eekf_mat const *u, void *userData);

static eekf_return measurement(eekf_mat *zp, eekf_mat *Jh, eekf_mat const *x,
                               void *userData);

static void mat_zero(eekf_mat *);

static eekf_value wrap_pi(eekf_value);

static void set_diag(eekf_mat *, uint8_t, const eekf_value *);

static void crank_update_Q(eekf_mat *, const crank_ekf_params_t *, eekf_value);

static void crank_set_R(eekf_mat *);

static eekf_value crank_get_angle_rad(void);

static eekf_value crank_get_omega_rad_s(void);

static eekf_value crank_get_cadence_rpm(void);

/**
 * @brief Calculate event time in 1/1024s as required by CPS service
 *
 */
static uint16_t cps_event_time_from_us(uint64_t t_us);

static void crank_ekf_recenter_angle(struct cps_crank_state *s);

/**
 * @brief Update external system states (for CPS service)
 *
 */
static void cps_crank_update(struct cps_crank_state *s,
                             float theta_wrapped,
                             float omega,
                             uint64_t t_us);

#endif /* FILTER_THREAD_H */