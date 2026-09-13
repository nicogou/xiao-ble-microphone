/*
 * Copyright (c) 2026 Nicolas Goualard
 * SPDX-License-Identifier: Apache-2.0
 *
 * WAV recorder public C API.
 */

#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Returns true if a WAV recording is currently in progress. */
#if CONFIG_APP_REC
bool rec_is_active(void);
#else
static inline bool rec_is_active(void) { return false; }
#endif

#ifdef __cplusplus
}
#endif
