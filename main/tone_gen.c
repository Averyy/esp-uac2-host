/**
 * @file tone_gen.c
 * @brief Simple sine tone generator for audio testing
 *
 * SPDX-License-Identifier: MIT
 */

#include <math.h>
#include <string.h>
#include "tone_gen.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void tone_gen_init(tone_gen_t *gen, const tone_gen_config_t *config)
{
    gen->config = *config;
    gen->phase = 0.0f;
    gen->phase_inc = (2.0f * (float)M_PI * config->frequency) / (float)config->sample_rate;
}

uint32_t tone_gen_fill(tone_gen_t *gen, uint8_t *buf, uint32_t buf_size)
{
    uint8_t bytes_per_sample = gen->config.bit_depth / 8;
    uint8_t frame_size = bytes_per_sample * gen->config.channels;
    uint32_t num_frames = buf_size / frame_size;
    uint32_t bytes_written = num_frames * frame_size;

    for (uint32_t i = 0; i < num_frames; i++) {
        float sample = gen->config.amplitude * sinf(gen->phase);

        gen->phase += gen->phase_inc;
        if (gen->phase >= 2.0f * (float)M_PI) {
            gen->phase -= 2.0f * (float)M_PI;
        }

        if (gen->config.bit_depth == 24) {
            // 24-bit signed, packed in 3 bytes little-endian
            int32_t val = (int32_t)(sample * 8388607.0f);  // 2^23 - 1
            if (val > 8388607) val = 8388607;
            if (val < -8388608) val = -8388608;

            for (uint8_t ch = 0; ch < gen->config.channels; ch++) {
                uint32_t offset = (i * frame_size) + (ch * 3);
                buf[offset + 0] = (uint8_t)(val & 0xFF);
                buf[offset + 1] = (uint8_t)((val >> 8) & 0xFF);
                buf[offset + 2] = (uint8_t)((val >> 16) & 0xFF);
            }
        } else {
            // 16-bit signed, little-endian
            int16_t val = (int16_t)(sample * 32767.0f);

            for (uint8_t ch = 0; ch < gen->config.channels; ch++) {
                uint32_t offset = (i * frame_size) + (ch * 2);
                buf[offset + 0] = (uint8_t)(val & 0xFF);
                buf[offset + 1] = (uint8_t)((val >> 8) & 0xFF);
            }
        }
    }

    return bytes_written;
}
