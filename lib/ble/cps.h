/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_CPS_H_
#define ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_CPS_H_

/**
 * @brief Cycling Power Service (CPS)
 * @defgroup bt_cps Cycling Power Service (CPS)
 * @ingroup bluetooth
 * @{
 *
 * [Experimental] Users should note that the APIs can change
 * as a part of ongoing development.
 */

#include <stdint.h>

#include <stdbool.h>

#include <zephyr/sys/slist.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Server shall restart the accumulation of energy expended from zero
 */
#define BT_CPS_CONTROL_POINT_RESET_ENERGY_EXPANDED_REQ 0x01

/**
 * @brief Server shall set the crank length
 */
#define BT_CPS_CONTROL_POINT_SET_CRANK_LENGTH_REQ 0x05

/** @brief Cycling power service response codes */
#define BT_CPS_CONTROL_POINT_RESPONSE_SUCCESS 0x20
#define BT_CPS_ATT_ERR_CONTROL_POINT_NOT_SUPPORTED 0x80

	/** @brief Cycling power service callback structure */
	struct bt_cps_cb
	{
		/** @brief Cycling power notifications changed
		 *
		 * @param enabled Flag that is true if notifications were enabled, false
		 *                if they were disabled.
		 */
		void (*ntf_changed)(bool enabled);

		/**
		 * @brief Cycling power control point write callback
		 *
		 * @note if Server supports the Energy Expended feature then application
		 * shall implement and support @ref BT_CPS_CONTROL_POINT_RESET_ENERGY_EXPANDED_REQ
		 * request code. For crank length adjustment, the callback will be called
		 * with the request code and buffer containing the crank length in mm (2 bytes).
		 *
		 * @param request control point request code
		 * @param buf pointer to request data (if any). For crank length: 2-byte uint16_t in little-endian
		 * @param len length of buf data
		 *
		 * @return 0 on successful handling of control point request
		 * @return -ENOTSUP if not supported. It can be used to pass handling to other
		 *         listeners in case of multiple listeners
		 * @return other negative error codes will result in immediate error response
		 */
		int (*ctrl_point_write)(uint8_t request, const uint8_t *buf, uint16_t len);

		/** Internal member to form a list of callbacks */
		sys_snode_t _node;
	};

	/** @brief Cycling power service callback register
	 *
	 * This function will register callbacks that will be called in
	 * certain events related to Cycling Power service.
	 *
	 * @param cb Pointer to callbacks structure. Must point to memory that remains valid
	 * until unregistered.
	 *
	 * @return 0 on success
	 * @return -EINVAL in case @p cb is NULL
	 */
	int bt_cps_cb_register(struct bt_cps_cb *cb);

	/** @brief Cycling power service callback unregister
	 *
	 * This function will unregister callback from Cycling Power service.
	 *
	 * @param cb Pointer to callbacks structure
	 *
	 * @return 0 on success
	 * @return -EINVAL in case @p cb is NULL
	 * @return -ENOENT in case the @p cb was not found in registered callbacks
	 */
	int bt_cps_cb_unregister(struct bt_cps_cb *cb);

	/** @brief Notify cycling power measurement.
	 *
	 * This will send a GATT notification to all current subscribers.
	 *
	 *  @param power_watts The instantaneous power measurement in watts (0-65535).
	 *  @param crank_revolutions Cumulative crank revolutions since sensor startup.
	 *  @param crank_event_time Crank event time in 1/1024 second units.
	 *
	 *  @return Zero in case of success and error code in case of error.
	 */
	int bt_cps_notify(int16_t power_watts, uint16_t crank_revolutions, uint16_t crank_event_time);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_BLUETOOTH_SERVICES_CPS_H_ */
