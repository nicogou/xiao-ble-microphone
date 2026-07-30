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
bool rec_is_active(void);

#ifdef __cplusplus
}
#endif
