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

#if CONFIG_APP_IMU
/** Initialise the IMU motion-event module.
 *
 *  Called automatically by SYS_INIT; exposed here so callers can check
 *  the return value during testing or manual bringup.
 *
 *  @return 0 on success, negative errno on failure.
 */
int imu_init(void);
#else
static inline int imu_init(void) { return 0; }
#endif

#ifdef __cplusplus
}
#endif
