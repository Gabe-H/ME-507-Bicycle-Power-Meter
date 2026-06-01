#include "filter_thread.h"

/**
 * @brief Angular velocity and angle position processing and filtering
 *
 */
static void filter_thread(void)
{
    // Kalman setup
    eekf_context ctx;
    // state of the filter
    EEKF_DECL_MAT_INIT(x, 2, 1, 0);
    EEKF_DECL_MAT_INIT(P, 2, 2,
                       pow(s_w, 2) * pow(dT, 4) / 4, pow(s_w, 2) * pow(dT, 3) / 2,
                       pow(s_w, 2) * pow(dT, 3) / 2, pow(s_w, 2) * pow(dT, 2));
    // input and process noise variables
    EEKF_DECL_MAT_INIT(u, 1, 1, 0.1);
    EEKF_DECL_MAT_INIT(Q, 2, 2,
                       pow(s_w, 2) * pow(dT, 4) / 4, pow(s_w, 2) * pow(dT, 3) / 2,
                       pow(s_w, 2) * pow(dT, 3) / 2, pow(s_w, 2) * pow(dT, 2));
    // measurement and measurement noise variables
    EEKF_DECL_MAT_INIT(z, 1, 1, 0);
    EEKF_DECL_MAT_INIT(R, 1, 1, s_z * s_z);

    int k;
    eekf_value v = 0, p = 0;

    for (k = 0; k < 1000; k++)
    {
    }

    // initialize the filter context
    eekf_init(&ctx, &x, &P, transition, measurement, NULL);

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
    return eEekfReturnOk;
}

K_THREAD_DEFINE(filter_thread_id, STACKSIZE, filter_thread, NULL, NULL, NULL, 6, 0, 0);