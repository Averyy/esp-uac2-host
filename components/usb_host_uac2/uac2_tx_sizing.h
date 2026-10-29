/*
 * SPDX-License-Identifier: MIT
 *
 * Playback packet sizing for full-speed UAC2 (1 ms frames), free of driver
 * state so test/host/test_tx_sizing.c can run it on the host.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/** Samples per frame in 16.16 for sample_freq Hz. */
static inline uint32_t uac2_tx_nominal_q16(uint32_t sample_freq)
{
    return (uint32_t)(((uint64_t)sample_freq << 16) / 1000);
}

/**
 * Samples per frame (16.16) to send. An async device's feedback reports its
 * real clock (e.g. 48.0009 at "48 kHz"); following it keeps the device FIFO
 * from drifting into underrun/overrun. Before the first report (fb_q16 == 0),
 * or if the report is more than 1 sample/frame off nominal, use nominal.
 */
static inline uint32_t uac2_tx_rate_q16(uint32_t nominal_q16, uint32_t fb_q16)
{
    if (fb_q16 == 0) {
        return nominal_q16;
    }
    uint32_t delta = fb_q16 > nominal_q16 ? fb_q16 - nominal_q16 : nominal_q16 - fb_q16;
    return delta <= (1u << 16) ? fb_q16 : nominal_q16;
}

/**
 * Sample frames for the next packet at rate_q16, carrying the fractional part
 * in *accumulator (kept below 1 << 16). Clamped to max_samples, the endpoint's
 * whole-frame capacity; frames above it are dropped, not carried.
 */
static inline uint32_t uac2_tx_next_packet_samples(uint32_t rate_q16, uint32_t *accumulator,
                                                   uint32_t max_samples)
{
    *accumulator += rate_q16 & 0xFFFF;
    uint32_t samples = (rate_q16 >> 16) + (*accumulator >> 16);
    *accumulator &= 0xFFFF;
    return samples > max_samples ? max_samples : samples;
}

/**
 * Sample frames for the next packet at exactly sample_freq / 1000 per frame,
 * carrying the remainder in *accumulator (kept below 1000). For endpoints
 * without feedback (adaptive, synchronous), which run off the host's frame
 * timing: 44.1 kHz gives nine 44-frame packets and one 45-frame packet per
 * 10 ms, where 16.16 rounding would fall about 0.2 ppm short. Clamped to
 * max_samples like uac2_tx_next_packet_samples().
 */
static inline uint32_t uac2_tx_next_nominal_samples(uint32_t sample_freq, uint32_t *accumulator,
                                                    uint32_t max_samples)
{
    *accumulator += sample_freq % 1000;
    uint32_t samples = sample_freq / 1000 + *accumulator / 1000;
    *accumulator %= 1000;
    return samples > max_samples ? max_samples : samples;
}

/**
 * Decode a feedback packet into 16.16: 4 bytes are 16.16 (what XMOS devices
 * such as the miniDSP send at full speed), 3 bytes are the USB spec's 10.14.
 * Returns false for any other length and leaves *out_q16 unchanged.
 */
static inline bool uac2_feedback_decode_q16(const uint8_t *buf, int len, uint32_t *out_q16)
{
    if (len == 4) {
        *out_q16 = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) |
                   ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
        return true;
    }
    if (len == 3) {
        *out_q16 = ((uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16)) << 2;
        return true;
    }
    return false;
}
