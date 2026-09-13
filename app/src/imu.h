/*
 * Copyright (c) 2026 Nicolas Goualard
 * SPDX-License-Identifier: Apache-2.0
 *
 * IMU significant motion detection for the LSM6DS3TR-C on XIAO BLE Sense.
 *
 * Detects significant motion and logs the event.
 * Starts automatically at boot when APP_IMU is enabled.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/** Called from the system work queue when significant motion is confirmed. */
typedef void (*imu_motion_cb_t)(void);

#if CONFIG_APP_IMU
/** Initialise the IMU motion-event module.
 *
 *  Called automatically by SYS_INIT; exposed here so callers can check
 *  the return value during testing or manual bringup.
 *
 *  @return 0 on success, negative errno on failure.
 */
int imu_init(void);

/** Register a callback invoked on every confirmed significant-motion event. */
void imu_set_motion_cb(imu_motion_cb_t cb);
#else
static inline int imu_init(void) { return 0; }
static inline void imu_set_motion_cb(imu_motion_cb_t cb) { ARG_UNUSED(cb); }
#endif

#ifdef __cplusplus
}
#endif
