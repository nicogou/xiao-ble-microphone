/*
 * Copyright (c) 2026 Nicolas Goualard
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pitch Detection public C API.
 * Include this header from any C/C++ file that needs to query pitch state.
 */

#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Returns true if pitch detection is currently running. */
bool pitch_is_active(void);

#ifdef __cplusplus
}
#endif
