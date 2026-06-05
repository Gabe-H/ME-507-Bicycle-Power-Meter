#include "filter_thread.h"

K_MEM_SLAB_DEFINE(filter_data_slab,
                  sizeof(struct filter_data_t),
                  10,
                  4);

/**
 * @brief Physical parameters of the system (constants)
 *
 */
static crank_ekf_params_t ekf_params = {
    .g = M_GRAVITY,
    // .rx = 0.051, // Example x offset 51mm
    // .ry = 0.001, // Example y offset 1mm
    .rx = 0.173, // Fake-a-crank x offset
    .ry = 0.012, // Fake-a-crank y offset
    .sigma_alpha = 15.0,
    .sigma_bg = 0.003,
    .sigma_ba = 0.02,
};

/* State Vector */
static EEKF_DECL_MAT_INIT(x, NX, 1,
                          0.0, // Theta
                          0.0, // Omega
                          0.0, // Bg
                          0.0, // Bax
                          0.0  // Bay
);

/* Initial covariance, P*/
/* Start fairly uncertain about angle/omega, less uncertain about biases */
static EEKF_DECL_MAT_INIT(P, NX, NX,
                          1.0, 0, 0, 0, 0,
                          0, 10, 0, 0, 0,
                          0, 0, 0.01, 0, 0,
                          0, 0, 0, 0.25, 0,
                          0, 0, 0, 0, 0.25);

/* Input u (only dt) */
static EEKF_DECL_MAT_INIT(u, NU, 1, 0.05); // Begin with dt=0.05s

/* Process nosie Q, filled each sample because it depends on dt */
static EEKF_DECL_MAT_INIT(Q, NX, NX);

/* Measurement z = [gyro, ax, ay]^T */
static EEKF_DECL_MAT(z, NZ, 1);

/* Measurement noise R */
static EEKF_DECL_MAT(R, NZ, NZ);

static void crank_ekf_init(void)
{
    // Initial state guess
    *EEKF_MAT_EL(x, 0, 0) = 0.0; // theta
    *EEKF_MAT_EL(x, 1, 0) = 0.0; // omega
    *EEKF_MAT_EL(x, 2, 0) = 0.0; // gyro bias
    *EEKF_MAT_EL(x, 3, 0) = 0.0; // ax bias
    *EEKF_MAT_EL(x, 4, 0) = 0.0; // ay bias

    crank_update_Q(&Q, &ekf_params, (eekf_value)IMU_PERIOD * (eekf_value)0.001);

    // Initial covariance
    mat_zero(&P);

    *EEKF_MAT_EL(P, 0, 0) = 1.0;  // theta uncertainty
    *EEKF_MAT_EL(P, 1, 1) = 10.0; // omega uncertainty
    *EEKF_MAT_EL(P, 2, 2) = 0.01; // gyro bias uncertainty
    *EEKF_MAT_EL(P, 3, 3) = 0.25; // ax bias uncertainty
    *EEKF_MAT_EL(P, 4, 4) = 0.25; // ay bias uncertainty

    crank_set_R(&R);

    eekf_init(&ekf_ctx, &x, &P, transition, measurement, &ekf_params);
}

static void crank_ekf_update(
    eekf_value dt,
    eekf_value gyro_rad_s,
    eekf_value ax_m_s2,
    eekf_value ay_m_s2)
{
    if (dt <= 0.0F)
    {
        return;
    }

    // Input u[0] = dt
    *EEKF_MAT_EL(u, 0, 0) = dt;

    // Measurement vector
    *EEKF_MAT_EL(z, 0, 0) = gyro_rad_s;
    *EEKF_MAT_EL(z, 1, 0) = ax_m_s2;
    *EEKF_MAT_EL(z, 2, 0) = ay_m_s2;

    /** Disable to test if constant dT assumption is good enough */
    // // Process covariance for this timestep
    crank_update_Q(&Q, &ekf_params, dt);

    // Predict to current sample time
    eekf_predict(&ekf_ctx, &u, &Q);

    // Correct using current gyro + accel measurement
    eekf_correct(&ekf_ctx, &z, &R);

    /* Do not wrap theta here. Keep the EKF angle locally continuous so
     * downstream crank revolution detection can use direct 2*pi crossings.
     * The measurement model wraps theta locally before sin/cos.
     */
}

/**
 * @brief Angular velocity and angle position processing and filtering
 *
 */
static void filter_thread(void)
{
    // Init static variables for each loop
    eekf_value ax;
    eekf_value ay;
    eekf_value gz;
    eekf_value dt;

    // Kalman setup
    crank_ekf_init();

    // Post READY bit to sync event share
    k_event_post(&thread_sync_event, FILTER_THREAD_READY);

    // Continue to main loop after START bit received
    k_event_wait(&thread_sync_event, START_BIT, false, K_FOREVER);

    previous_ts = k_uptime_get();

    while (1)
    {
        int64_t loop_start_ms = k_uptime_get();

        struct imu_data_t *data = k_fifo_get(&imu_fifo, K_FOREVER);
        if (data)
        {
            // -- OPTIMAL CONFIGURATION: Board is actually in line with crank axis
            // ax = data->accel_x;
            // ay = data->accel_y;
            // gz = data->gyro_z;

            // -- INTENDED CONFIGURATION: board top faces out from crank axis
            //    (Gyro Z is rotation direction)
            // Translate IMU axes such that +x faces outward from crank
            // and +y faces upwards when crank is horizontal
            // ax = data->accel_y;
            // ay = -data->accel_x;
            // gz = data->gyro_z;

            // -- ALTERNATE CONFIGURATION: board on side of crank
            //    (-Gyro X is rotation direction)
            ax = data->accel_y;
            ay = -data->accel_z;
            gz = -data->gyro_x;

            // Unit conversions
            ax *= M_GRAVITY;    // g -> m/s^2
            ay *= M_GRAVITY;    // g -> m/s^2
            gz *= DPS_TO_RAD_S; // dps -> rad/s

            // Get dt from readings
            uint32_t dt_ms = (uint32_t)(data->ts - previous_ts);
            previous_ts = data->ts;
            dt = (eekf_value)dt_ms / (eekf_value)1000.0F;

            // Release location on slab now that data has been processed
            k_mem_slab_free(&imu_data_slab, (void *)data);
        }
        else
            continue; // Do not process if NULL data returned

        // Run an iteration of the kalman filter
        crank_ekf_update(
            dt,
            gz,
            ax,
            ay);

        eekf_value angle_unwrapped = crank_get_angle_rad();
        eekf_value omega_rad_s = crank_get_omega_rad_s();
        eekf_value cadence_rpm = crank_get_cadence_rpm();

        uint64_t k_uptime = k_ticks_to_us_floor64(k_uptime_ticks());

        cps_crank_update(
            &crank_state,
            angle_unwrapped,
            omega_rad_s,
            k_uptime);

        /* Recenter only after CPS has consumed the current sample. This keeps
         * the EKF state numerically small without losing crank revolution
         * continuity.
         */
        crank_ekf_recenter_angle(&crank_state);

        eekf_value angle_local = crank_get_angle_rad();

        struct filter_data_t *data_out;

        if (k_mem_slab_alloc(&filter_data_slab, (void **)&data_out, K_MSEC(10)) == 0)
        {
            data_out->theta = angle_local;
            data_out->omega = cadence_rpm;
            data_out->crank_index = crank_state.crank_revs;
            data_out->crank_event_time = crank_state.crank_event_time;
            data_out->ts = loop_start_ms;

            k_fifo_put(&filter_fifo, data_out);
        }
        else
        {
            // LOG_WRN("Warning: Couldn't allocate space on slab");
        }

        /**
         * Only ONE task should be the master of update timing.
         * If the IMU sends data at a fixed rate, DO NOT wait here.
         * If the IMU does not send data at a fixed rate, OD wait here.
         */
        // int64_t remaining_ms = FILTER_PERIOD - k_uptime_delta(&loop_start_ms);
        // k_sleep(K_MSEC(remaining_ms));
    }
}

/**
 * @brief The state prediction function
 *
 * @param xp
 * @param Jf
 * @param x
 * @param u
 * @param userData
 * @return eekf_return
 */
static eekf_return transition(eekf_mat *xp, eekf_mat *Jf, eekf_mat const *x,
                              eekf_mat const *u, void *userData)
{
    (void)userData;

    const eekf_value dt = *EEKF_MAT_EL(*u, 0, 0);
    const eekf_value theta = *EEKF_MAT_EL(*x, 0, 0);
    const eekf_value omega = *EEKF_MAT_EL(*x, 1, 0);
    const eekf_value bg = *EEKF_MAT_EL(*x, 2, 0);
    const eekf_value bax = *EEKF_MAT_EL(*x, 3, 0);
    const eekf_value bay = *EEKF_MAT_EL(*x, 4, 0);

    // Predicted state
    *EEKF_MAT_EL(*xp, 0, 0) = theta + dt * omega;
    *EEKF_MAT_EL(*xp, 1, 0) = omega;
    *EEKF_MAT_EL(*xp, 2, 0) = bg;
    *EEKF_MAT_EL(*xp, 3, 0) = bax;
    *EEKF_MAT_EL(*xp, 4, 0) = bay;

    // Jacobian F = df/dx
    mat_zero(Jf);

    *EEKF_MAT_EL(*Jf, 0, 0) = 1.0;
    *EEKF_MAT_EL(*Jf, 0, 1) = dt;

    *EEKF_MAT_EL(*Jf, 1, 1) = 1.0;
    *EEKF_MAT_EL(*Jf, 2, 2) = 1.0;
    *EEKF_MAT_EL(*Jf, 3, 3) = 1.0;
    *EEKF_MAT_EL(*Jf, 4, 4) = 1.0;

    return eEekfReturnOk;
}

/**
 * @brief The measurement prediction function
 *
 * @param zp
 * @param Jh
 * @param x
 * @param userData
 * @return eekf_return
 */
static eekf_return measurement(eekf_mat *zp, eekf_mat *Jh, eekf_mat const *x,
                               void *userData)
{
    crank_ekf_params_t *p = (crank_ekf_params_t *)userData;

    /* Keep trig arguments bounded, but do not modify the EKF state angle. */
    const eekf_value theta = wrap_pi(*EEKF_MAT_EL(*x, 0, 0));
    const eekf_value omega = *EEKF_MAT_EL(*x, 1, 0);
    const eekf_value bg = *EEKF_MAT_EL(*x, 2, 0);
    const eekf_value bax = *EEKF_MAT_EL(*x, 3, 0);
    const eekf_value bay = *EEKF_MAT_EL(*x, 4, 0);

    const eekf_value s = EEKF_MAT_SIN(theta);
    const eekf_value c = EEKF_MAT_COS(theta);
    const eekf_value w2 = omega * omega;

    // Predicted measurement h(x)
    *EEKF_MAT_EL(*zp, 0, 0) = omega + bg;
    *EEKF_MAT_EL(*zp, 1, 0) = p->g * s - w2 * p->rx + bax;
    *EEKF_MAT_EL(*zp, 2, 0) = p->g * c - w2 * p->ry + bay;

    // Measurement Jacobian H = dh/dx
    mat_zero(Jh);

    // gyro row
    *EEKF_MAT_EL(*Jh, 0, 1) = 1.0F;
    *EEKF_MAT_EL(*Jh, 0, 2) = 1.0F;

    // accel x row
    *EEKF_MAT_EL(*Jh, 1, 0) = p->g * c;
    *EEKF_MAT_EL(*Jh, 1, 1) = -2.0F * omega * p->rx;
    *EEKF_MAT_EL(*Jh, 1, 3) = 1.0F;

    // accel y row
    *EEKF_MAT_EL(*Jh, 2, 0) = -p->g * s;
    *EEKF_MAT_EL(*Jh, 2, 1) = -2.0F * omega * p->ry;
    *EEKF_MAT_EL(*Jh, 2, 4) = 1.0F;

    return eEekfReturnOk;
}

/*** HELPER FUNCTIONS ***/

/**
 * @brief Set all elements of matrix to 0.
 *
 * @param m
 */
static void mat_zero(eekf_mat *m)
{
    for (uint8_t c = 0; c < m->cols; c++)
    {
        for (uint8_t r = 0; r < m->rows; r++)
        {
            *EEKF_MAT_EL(*m, r, c) = 0.0F;
        }
    }
}

/**
 * @brief Ensure eekf_value is between -pi to +pi
 *
 * @param a value to be wrapped
 * @return eekf_value
 */
static eekf_value wrap_pi(eekf_value a)
{
    while (a > (eekf_value)M_PI)
    {
        a -= M_TWO_PI_F;
    }
    while (a < -(eekf_value)M_PI)
    {
        a += M_TWO_PI_F;
    }
    return a;
}

/**
 * @brief Set the diagonal of a matrix to some diag array
 *
 * @param m eekf_mat matrix
 * @param n num diagonals to be set
 * @param diag eekf_vector array of values to set to each diag. element
 * @return * void
 */
static void set_diag(eekf_mat *m, uint8_t n, const eekf_value *diag)
{
    mat_zero(m);
    for (uint8_t i = 0; i < n; i++)
    {
        *EEKF_MAT_EL(*m, i, i) = diag[i];
    }
}

/**
 * @brief Fill out Q after each sample because it depends on dt
 *
 * @param Q
 * @param p
 * @param dt
 */
static void crank_update_Q(eekf_mat *Q, const crank_ekf_params_t *p, eekf_value dt)
{
    mat_zero(Q);

    const eekf_value sa2 = p->sigma_alpha * p->sigma_alpha;
    const eekf_value sbg2 = p->sigma_bg * p->sigma_bg;
    const eekf_value sba2 = p->sigma_ba * p->sigma_ba;

    const eekf_value dt2 = dt * dt;
    const eekf_value dt3 = dt2 * dt;
    const eekf_value dt4 = dt2 * dt2;

    // angle/omega process noise from angular acceleration uncertainty
    *EEKF_MAT_EL(*Q, 0, 0) = 0.25F * sa2 * dt4;
    *EEKF_MAT_EL(*Q, 0, 1) = 0.5F * sa2 * dt3;
    *EEKF_MAT_EL(*Q, 1, 0) = 0.5F * sa2 * dt3;
    *EEKF_MAT_EL(*Q, 1, 1) = sa2 * dt2;

    // bias random walks
    *EEKF_MAT_EL(*Q, 2, 2) = sbg2 * dt;
    *EEKF_MAT_EL(*Q, 3, 3) = sba2 * dt;
    *EEKF_MAT_EL(*Q, 4, 4) = sba2 * dt;
}

static void crank_set_R(eekf_mat *R)
{
    mat_zero(R);

    const eekf_value sigma_g = 0.02F; // rad/s; start conservative (more uncertain that IMU docs)
    const eekf_value sigma_a = 0.50F; // m/s^2; start conservative (more uncertain that IMU docs)

    *EEKF_MAT_EL(*R, 0, 0) = sigma_g * sigma_g;
    *EEKF_MAT_EL(*R, 1, 1) = sigma_a * sigma_a;
    *EEKF_MAT_EL(*R, 2, 2) = sigma_a * sigma_a;
}

static eekf_value crank_get_angle_rad(void)
{
    return *EEKF_MAT_EL(x, 0, 0);
}

static eekf_value crank_get_omega_rad_s(void)
{
    return *EEKF_MAT_EL(x, 1, 0);
}

static eekf_value crank_get_cadence_rpm(void)
{
    const eekf_value omega = crank_get_omega_rad_s();
    return omega * (eekf_value)60.0F / (M_TWO_PI_F);
}

static uint16_t cps_event_time_from_us(uint64_t t_us)
{
    /*
     * CPS event time is uint16_t in 1/1024 second units.
     * Natural uint16_t wrap is correct.
     */
    return (uint16_t)((t_us * 1024ULL) / 1000000ULL);
}

static void crank_ekf_recenter_angle(struct cps_crank_state *s)
{
    eekf_value theta = *EEKF_MAT_EL(x, 0, 0);

    if ((theta < THETA_RECENTER_RAD) && (theta > -THETA_RECENTER_RAD))
    {
        return;
    }

    /* Use truncation toward zero so the recentered angle keeps the same sign
     * and remains close to zero. Only subtract whole revolutions so the
     * physical angle, sin(theta), and cos(theta) are unchanged.
     */
    int32_t revs_to_remove = (int32_t)(theta / M_TWO_PI_F);

    if (revs_to_remove == 0)
    {
        return;
    }

    eekf_value theta_offset = (eekf_value)revs_to_remove * M_TWO_PI_F;

    *EEKF_MAT_EL(x, 0, 0) = theta - theta_offset;

    if ((s != NULL) && s->initialized)
    {
        /* Keep the CPS detector's internal angle frame aligned with the EKF
         * frame. Do not alter s->crank_revs; that is the actual BLE CPS
         * cumulative count.
         */
        s->theta_unwrapped -= theta_offset;
        s->theta_prev_unwrapped -= theta_offset;
        s->theta_prev_wrapped -= theta_offset;
        s->rev_index -= revs_to_remove;
    }
}

static void cps_crank_update(struct cps_crank_state *s,
                             eekf_value theta_unwrapped,
                             eekf_value omega_rad_s,
                             uint64_t t_us)
{
    /* If the physical positive direction is opposite of the CPS crank-count
     * direction, flip both theta_unwrapped and omega_rad_s before calling this
     * function.
     */

    if (!s->initialized)
    {
        s->initialized = true;

        s->theta_prev_wrapped = theta_unwrapped; /* Kept for struct compatibility. */
        s->theta_unwrapped = theta_unwrapped;
        s->theta_prev_unwrapped = theta_unwrapped;
        s->t_prev_us = t_us;

        s->rev_index = (int32_t)floorf(theta_unwrapped / M_TWO_PI_F);

        return;
    }

    eekf_value theta_old = s->theta_prev_unwrapped;
    eekf_value theta_new = theta_unwrapped;
    eekf_value dtheta = theta_new - theta_old;

    int32_t old_rev_index = s->rev_index;
    int32_t new_rev_index = (int32_t)floorf(theta_new / M_TWO_PI_F);

    /* Count forward crank revolutions only. A revolution is counted when the
     * locally continuous EKF theta crosses one or more 2*pi boundaries in the
     * positive direction. omega_rad_s is used only as a sanity gate against
     * low-speed jitter near a boundary.
     */
    if ((new_rev_index > old_rev_index) &&
        (dtheta > 0.0F) &&
        (omega_rad_s > (eekf_value)MIN_FORWARD_OMEGA))
    {
        for (int32_t k = old_rev_index + 1; k <= new_rev_index; k++)
        {
            eekf_value crossing_angle = (eekf_value)k * M_TWO_PI_F;
            eekf_value frac = (crossing_angle - theta_old) / dtheta;

            if (frac < 0.0F)
            {
                frac = 0.0F;
            }
            else if (frac > 1.0F)
            {
                frac = 1.0F;
            }

            uint64_t dt_us = t_us - s->t_prev_us;
            uint64_t t_cross_us = s->t_prev_us + (uint64_t)(frac * (eekf_value)dt_us);

            s->crank_revs++;
            s->crank_event_time = cps_event_time_from_us(t_cross_us);
        }

        s->rev_index = new_rev_index;
    }
    else if (new_rev_index > old_rev_index)
    {
        /* The angle crossed a boundary, but the motion sanity checks failed.
         * Treat it as consumed so jitter cannot repeatedly count it later.
         * Keep MIN_FORWARD_OMEGA low enough that genuine very-slow pedaling is
         * not rejected.
         */
        s->rev_index = new_rev_index;
    }

    s->theta_prev_wrapped = theta_new; /* Kept for struct compatibility. */
    s->theta_prev_unwrapped = theta_new;
    s->theta_unwrapped = theta_new;
    s->t_prev_us = t_us;
}

K_THREAD_DEFINE(filter_thread_id, STACKSIZE, filter_thread, NULL, NULL, NULL, 6, 0, 0);