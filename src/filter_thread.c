#include "filter_thread.h"

static crank_ekf_params_t ekf_params = {
    .g = 9.80665,
    .rx = 0.05, // Example x offset 50mm
    .ry = 0.0,  // Example y offset 0mm
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
static EEKF_DECL_MAT_INIT(u, NU, 1, EEKF_DT);

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
    if (dt <= 0.0)
    {
        return;
    }

    // Input u[0] = dt
    *EEKF_MAT_EL(u, 0, 0) = dt;

    // Measurement vector
    *EEKF_MAT_EL(z, 0, 0) = gyro_rad_s;
    *EEKF_MAT_EL(z, 1, 0) = ax_m_s2;
    *EEKF_MAT_EL(z, 2, 0) = ay_m_s2;

    // Process covariance for this timestep
    crank_update_Q(&Q, &ekf_params, dt);

    // Predict to current sample time
    eekf_predict(&ekf_ctx, &u, &Q);

    // Correct using current gyro + accel measurement
    eekf_correct(&ekf_ctx, &z, &R);

    // Wrap angle after correction
    *EEKF_MAT_EL(x, 0, 0) = wrap_pi(*EEKF_MAT_EL(x, 0, 0));
}

/**
 * @brief Angular velocity and angle position processing and filtering
 *
 */
static void filter_thread(void)
{
    // Kalman setup

    // Post READY bit to sync event share
    k_event_post(&thread_sync_event, FILTER_THREAD_READY);

    // Continue to main loop after START bit received
    k_event_wait(&thread_sync_event, START_BIT, false, K_FOREVER);

    while (1)
    {
        struct imu_data_t *data = k_fifo_get(&imu_fifo, K_FOREVER);
        float ax = data->accel_x;
        float ay = data->accel_y;

        k_mem_slab_free(&imu_data_slab, (void *)data);

        float angle = 0;
        int state = 0;

        // Check if atan denominator is 0 first
        if (ax == 0.0F)
        {
            if (ay < 0.0F)
                angle = 4.7124;
            else
                angle = 1.5708;
        }
        else
        {
            angle = atanf(ay / ax);

            // Arctan truth table.
            // Theta=0 when IMU x axis is facing upwards
            if (ax > 0.0F && ay > 0.0F)
            {
                state = 1;
                angle = 3.1415F - angle;
            }

            else if (ax > 0.0F && ay <= 0.0F)
            {
                state = 2;
                angle = 3.1415F - angle;
            }
            else if (ax < 0.0F && ay >= 0.0F)
            {
                state = 3;
                angle = -angle;
            }
            else if (ax < 0.0F && ay < 0.0F)
            {
                angle = 6.2831F - angle;
            }

            // angle += 1.5708; // Offset due to IMU placement
        }

        char *mem_ptr = k_malloc(50);

        // sprintf(mem_ptr, "Angle: %f", angle);
        sprintf(mem_ptr, "X: %.2f, Y: %.2f, theta: %.2f, [%d]", (double)data->accel_x, (double)data->accel_y, (double)angle, state);

        k_fifo_put(&printk_fifo, mem_ptr);
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
    *EEKF_MAT_EL(*xp, 0, 0) = wrap_pi(theta + dt * omega);
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

    const eekf_value theta = *EEKF_MAT_EL(*x, 0, 0);
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
    *EEKF_MAT_EL(*Jh, 0, 1) = 1.0;
    *EEKF_MAT_EL(*Jh, 0, 2) = 1.0;

    // accel x row
    *EEKF_MAT_EL(*Jh, 1, 0) = p->g * c;
    *EEKF_MAT_EL(*Jh, 1, 1) = -2.0 * omega * p->rx;
    *EEKF_MAT_EL(*Jh, 1, 3) = 1.0;

    // accel y row
    *EEKF_MAT_EL(*Jh, 2, 0) = -p->g * s;
    *EEKF_MAT_EL(*Jh, 2, 1) = -2.0 * omega * p->ry;
    *EEKF_MAT_EL(*Jh, 2, 4) = 1.0;

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
            *EEKF_MAT_EL(*m, r, c) = 0.0;
        }
    }
}

/**
 * @brief Ensure eekf_value is between 0-2pi
 *
 * @param a value to be wrapped
 * @return eekf_value
 */
static eekf_value wrap_pi(eekf_value a)
{
    while (a > M_PI)
    {
        a -= 2.0 * M_PI;
    }
    while (a < -M_PI)
    {
        a += 2.0 * M_PI;
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
    *EEKF_MAT_EL(*Q, 0, 0) = 0.25 * sa2 * dt4;
    *EEKF_MAT_EL(*Q, 0, 1) = 0.5 * sa2 * dt3;
    *EEKF_MAT_EL(*Q, 1, 0) = 0.5 * sa2 * dt3;
    *EEKF_MAT_EL(*Q, 1, 1) = sa2 * dt2;

    // bias random walks
    *EEKF_MAT_EL(*Q, 2, 2) = sbg2 * dt;
    *EEKF_MAT_EL(*Q, 3, 3) = sba2 * dt;
    *EEKF_MAT_EL(*Q, 4, 4) = sba2 * dt;
}

static void crank_set_R(eekf_mat *R)
{
    mat_zero(R);

    const eekf_value sigma_g = 0.02; // rad/s; start conservative (more uncertain that IMU docs)
    const eekf_value sigma_a = 0.50; // m/s^2; start conservative (more uncertain that IMU docs)

    *EEKF_MAT_EL(*R, 0, 0) = sigma_g * sigma_g;
    *EEKF_MAT_EL(*R, 1, 1) = sigma_a * sigma_a;
    *EEKF_MAT_EL(*R, 2, 2) = sigma_a * sigma_a;
}

K_THREAD_DEFINE(filter_thread_id, STACKSIZE, filter_thread, NULL, NULL, NULL, 6, 0, 0);