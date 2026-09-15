#pragma once

#include <stdbool.h>
#include <errno.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Callback fired from the pitch thread on each detected note (e.g. note="A", octave=4).
 *  cents is the deviation from the equal-tempered note, in [-50, +50].
 */
typedef void (*pitch_note_cb_t)(const char *note, int octave, float cents);

/** Callback fired from the pitch thread once per analysis hop (~16 ms) with the
 *  RMS level of the latest samples, in [0, 1]. Fires for silent frames too.
 */
typedef void (*pitch_level_cb_t)(float rms);

#if CONFIG_APP_PITCH
bool pitch_is_active(void);
int  pitch_start(void);
int  pitch_stop(void);
void pitch_set_note_cb(pitch_note_cb_t cb);
void pitch_set_level_cb(pitch_level_cb_t cb);
#else
static inline bool pitch_is_active(void)                 { return false; }
static inline int  pitch_start(void)                     { return -ENODEV; }
static inline int  pitch_stop(void)                      { return 0; }
static inline void pitch_set_note_cb(pitch_note_cb_t cb) { (void)cb; }
static inline void pitch_set_level_cb(pitch_level_cb_t cb) { (void)cb; }
#endif

#ifdef __cplusplus
}
#endif
