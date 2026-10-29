/*
 * SPDX-FileCopyrightText: 2026 Avery Levitt
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file tone_gen.h
 * @brief Simple sine tone generator for audio testing
 *
 * Generates interleaved PCM sine waves at a specified frequency.
 * Used to verify playback through the UAC2 driver.
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t sample_rate;       // e.g. 48000
    uint8_t  channels;          // e.g. 2
    uint8_t  bit_depth;         // 16 or 24
    float    frequency;         // tone frequency in Hz (e.g. 1000.0)
    float    amplitude;         // 0.0 to 1.0
} tone_gen_config_t;

typedef struct {
    tone_gen_config_t config;
    float phase;                // current phase in radians
    float phase_inc;            // phase increment per sample
    float env;                  // gain envelope, 0..1
    float env_step;             // envelope change per sample frame
    float env_target;           // 1 while playing, 0 after tone_gen_fade_out()
} tone_gen_t;

/**
 * Initialize a tone generator.
 */
void tone_gen_init(tone_gen_t *gen, const tone_gen_config_t *config);

/**
 * Fill a buffer with interleaved PCM samples.
 *
 * @param gen       Tone generator state
 * @param buf       Output buffer
 * @param buf_size  Buffer size in bytes
 * @return          Number of bytes written (always buf_size, rounded down to frame boundary)
 */
uint32_t tone_gen_fill(tone_gen_t *gen, uint8_t *buf, uint32_t buf_size);

/**
 * Ramp the tone down to silence over the next 20 ms of samples. Tones always
 * start with a 20 ms ramp up, so neither edge clicks.
 */
void tone_gen_fade_out(tone_gen_t *gen);

#ifdef __cplusplus
}
#endif
