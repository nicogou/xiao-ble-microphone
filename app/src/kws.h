/*
 * Copyright (c) 2026 Nicolas Goualard
 * SPDX-License-Identifier: Apache-2.0
 *
 * Keyword Spotting (KWS) public C API.
 * Include this header from C files that need to query KWS state.
 */

#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Returns true if keyword spotting is currently running.
 */
bool kws_is_active(void);

#ifdef __cplusplus
}
#endif
