/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_CSCS_H_
#define ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_CSCS_H_

/**
 * @brief Cycling Speed and Cadence Service (CSCS)
 * @defgroup bt_cscs Cycling Speed and Cadence Service (CSCS)
 * @ingroup bluetooth
 * @{
 */

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/sys/slist.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /** @brief Cycling Speed and Cadence callback structure */
    struct bt_cscs_cb
    {
        /** @brief CSCS notifications changed
         *
         * @param enabled Flag that is true if notifications were enabled, false
         *                if they were disabled.
         */
        void (*ntf_changed)(bool enabled);

        /** Internal member to form a list of callbacks */
        sys_snode_t _node;
    };

    /** @brief CSCS callback register
     *
     * @param cb Pointer to callbacks structure. Must point to memory that remains valid
     * until unregistered.
     *
     * @return 0 on success
     * @return -EINVAL in case @p cb is NULL
     */
    int bt_cscs_cb_register(struct bt_cscs_cb *cb);

    /** @brief CSCS callback unregister
     *
     * @param cb Pointer to callbacks structure
     *
     * @return 0 on success
     * @return -EINVAL in case @p cb is NULL
     * @return -ENOENT in case the @p cb was not found in registered callbacks
     */
    int bt_cscs_cb_unregister(struct bt_cscs_cb *cb);

    /** @brief Notify cycling speed and cadence measurement.
     *
     * This will send a GATT notification to all current subscribers.
     *
     *  @param crank_revolutions Cumulative crank revolutions since sensor startup.
     *  @param crank_event_time Crank event time in 1/1024 second units.
     *
     *  @return Zero in case of success and error code in case of error.
     */
    int bt_cscs_notify(uint16_t crank_revolutions, uint16_t crank_event_time);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_CSCS_H_ */
