/*
 * SPDX-License-Identifier: MIT
 *
 * Host-side tests for the playback packet sizing in uac2_tx_sizing.h: over
 * long simulated streams the samples sent must match the rate the device asks
 * for (its feedback, or the nominal rate), packets must stay whole frames
 * within the endpoint's max packet size, and feedback must decode correctly.
 */

#include <stdio.h>
#include <stdlib.h>
#include "uac2_tx_sizing.h"

static int s_failures;

#define CHECK(cond) do { \
    if (!(cond)) { \
        printf("    FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        s_failures++; \
    } \
} while (0)

/** 16.16 for a rate given in millihertz, e.g. 48000900 -> 48.0009 samples/frame. */
static uint32_t q16_from_mhz(uint64_t mhz)
{
    return (uint32_t)((mhz << 16) / 1000000);
}

typedef struct {
    uint64_t samples;
    uint32_t min_pkt;
    uint32_t max_pkt;
    uint32_t max_acc;
} stream_result_t;

/**
 * Simulate frames packets the way stream_tx_xfer_submit() sizes them, with
 * the feedback reported from the first frame on.
 */
static stream_result_t simulate(uint32_t sample_freq, uint32_t fb_q16, uint32_t max_samples,
                                uint64_t frames)
{
    stream_result_t r = { .min_pkt = UINT32_MAX };
    uint32_t nominal = uac2_tx_nominal_q16(sample_freq);
    uint32_t rate = uac2_tx_rate_q16(nominal, fb_q16);
    uint32_t acc = 0;
    for (uint64_t i = 0; i < frames; i++) {
        uint32_t n = uac2_tx_next_packet_samples(rate, &acc, max_samples);
        r.samples += n;
        if (n < r.min_pkt) r.min_pkt = n;
        if (n > r.max_pkt) r.max_pkt = n;
        if (acc > r.max_acc) r.max_acc = acc;
    }
    return r;
}

/** Samples a rate of rate_q16 asks for over frames 1 ms frames, rounded down. */
static uint64_t expected_samples(uint32_t rate_q16, uint64_t frames)
{
    return (uint64_t)rate_q16 * frames >> 16;
}

static void check_stream(const char *name, uint32_t sample_freq, uint32_t fb_q16,
                         uint32_t max_samples, uint64_t frames, uint32_t want_min, uint32_t want_max)
{
    printf("[%s]\n", name);
    stream_result_t r = simulate(sample_freq, fb_q16, max_samples, frames);
    uint32_t rate = uac2_tx_rate_q16(uac2_tx_nominal_q16(sample_freq), fb_q16);
    uint64_t want = expected_samples(rate, frames);
    printf("    %llu frames: %llu samples (want %llu), packets %u..%u\n",
           (unsigned long long)frames, (unsigned long long)r.samples,
           (unsigned long long)want, r.min_pkt, r.max_pkt);
    // The accumulator carries every fractional sample, so the total never
    // drifts from the rate by more than the one sample still being carried.
    CHECK(r.samples == want || r.samples == want + 1);
    CHECK(r.min_pkt == want_min && r.max_pkt == want_max);
    CHECK(r.max_acc < (1u << 16));
}

#define ONE_HOUR_FRAMES (3600ull * 1000)

static void test_rates(void)
{
    // miniDSP 2x4 HD: 24-bit stereo, MPS 294 bytes = 49 whole frames
    const uint32_t mps_frames = 294 / 6;

    check_stream("48 kHz, feedback exactly 48.0 (real miniDSP report 0x00300000)",
                 48000, 0x00300000, mps_frames, ONE_HOUR_FRAMES, 48, 48);
    check_stream("48 kHz before any feedback (nominal)",
                 48000, 0, mps_frames, ONE_HOUR_FRAMES, 48, 48);
    check_stream("44.1 kHz, feedback 44.1",
                 44100, uac2_tx_nominal_q16(44100), mps_frames, ONE_HOUR_FRAMES, 44, 45);
    check_stream("48 kHz, device clock 48.0009 (19 ppm fast)",
                 48000, q16_from_mhz(48000900), mps_frames, ONE_HOUR_FRAMES, 48, 49);
    check_stream("48 kHz, device clock 47.9991 (19 ppm slow)",
                 48000, q16_from_mhz(47999100), mps_frames, ONE_HOUR_FRAMES, 47, 48);
    check_stream("48 kHz, fb-drift simulator (+2 Hz)",
                 48000, uac2_tx_nominal_q16(48002), mps_frames, ONE_HOUR_FRAMES, 48, 49);
    check_stream("44.1 kHz, device clock 44.1004",
                 44100, q16_from_mhz(44100400), mps_frames, ONE_HOUR_FRAMES, 44, 45);
    check_stream("48 kHz on the 16-bit alt (MPS 196 = 49 frames), clock 48.0009",
                 48000, q16_from_mhz(48000900), 196 / 4, ONE_HOUR_FRAMES, 48, 49);
}

static void test_feedback_plausibility(void)
{
    printf("[feedback outside +-1 sample/frame falls back to nominal]\n");
    uint32_t nominal = uac2_tx_nominal_q16(48000);
    CHECK(uac2_tx_rate_q16(nominal, 0) == nominal);
    CHECK(uac2_tx_rate_q16(nominal, nominal + (1u << 16)) == nominal + (1u << 16));
    CHECK(uac2_tx_rate_q16(nominal, nominal - (1u << 16)) == nominal - (1u << 16));
    CHECK(uac2_tx_rate_q16(nominal, nominal + (1u << 16) + 1) == nominal);
    CHECK(uac2_tx_rate_q16(nominal, nominal - (1u << 16) - 1) == nominal);
    // 10.14 misread as 16.16 is 4x too small, 16.16 misread as 10.14 4x too big
    CHECK(uac2_tx_rate_q16(nominal, nominal / 4) == nominal);
    CHECK(uac2_tx_rate_q16(nominal, nominal * 4) == nominal);
}

static void test_clamp_never_exceeds_mps(void)
{
    printf("[packets never exceed the endpoint's whole-frame capacity]\n");
    // Endpoint sized exactly for nominal: feedback above nominal is clamped
    stream_result_t r = simulate(48000, q16_from_mhz(48500000), 48, 100000);
    CHECK(r.max_pkt == 48);
    CHECK(r.samples == 48ull * 100000);
}

static void test_feedback_decode(void)
{
    printf("[feedback decoding]\n");
    uint32_t q16 = 0;
    // 4-byte 16.16, as the miniDSP sends: 48.0 samples/frame
    const uint8_t fb4[4] = {0x00, 0x00, 0x30, 0x00};
    CHECK(uac2_feedback_decode_q16(fb4, 4, &q16) && q16 == 0x00300000);
    // 3-byte 10.14: 44.1 samples/frame = 0x0B0666 in 10.14
    const uint8_t fb3[3] = {0x66, 0x06, 0x0B};
    CHECK(uac2_feedback_decode_q16(fb3, 3, &q16) && q16 == 0x0B0666u << 2);
    CHECK((q16 >> 16) == 44);
    // Anything else leaves the last value alone
    q16 = 1234;
    CHECK(!uac2_feedback_decode_q16(fb4, 2, &q16) && q16 == 1234);
    CHECK(!uac2_feedback_decode_q16(fb4, 0, &q16) && q16 == 1234);
}

static void test_rate_change_keeps_fraction(void)
{
    printf("[switching from nominal to feedback mid-stream keeps the carried fraction]\n");
    uint32_t nominal = uac2_tx_nominal_q16(44100);
    uint32_t fb = q16_from_mhz(44100400);
    uint32_t acc = 0;
    uint64_t samples = 0;
    for (int i = 0; i < 8; i++) {  // first feedback arrives after ~8 frames
        samples += uac2_tx_next_packet_samples(uac2_tx_rate_q16(nominal, 0), &acc, 49);
    }
    for (int i = 0; i < 1000000; i++) {
        samples += uac2_tx_next_packet_samples(uac2_tx_rate_q16(nominal, fb), &acc, 49);
    }
    uint64_t want = ((uint64_t)nominal * 8 + (uint64_t)fb * 1000000) >> 16;
    CHECK(samples == want || samples == want + 1);
}

static void test_nominal_without_feedback(void)
{
    printf("[no feedback endpoint: exact nominal pattern, no long-run drift]\n");
    uint32_t acc = 0;
    int pattern[10];
    for (int i = 0; i < 10; i++) {
        pattern[i] = (int)uac2_tx_next_nominal_samples(44100, &acc, 49);
    }
    // Nine 44-frame packets, then one 45-frame packet, every 10 ms
    for (int i = 0; i < 9; i++) CHECK(pattern[i] == 44);
    CHECK(pattern[9] == 45);
    CHECK(acc == 0);

    static const uint32_t rates[] = {8000, 11025, 16000, 22050, 32000, 44100, 48000, 88200, 96000};
    const uint64_t frames = 3600ull * 1000;    // one hour
    for (size_t r = 0; r < sizeof(rates) / sizeof(rates[0]); r++) {
        uint32_t base = rates[r] / 1000;
        uint32_t lo = UINT32_MAX, hi = 0;
        uint64_t samples = 0;
        acc = 0;
        for (uint64_t i = 0; i < frames; i++) {
            uint32_t n = uac2_tx_next_nominal_samples(rates[r], &acc, 200);
            samples += n;
            if (n < lo) lo = n;
            if (n > hi) hi = n;
            CHECK(acc < 1000);
        }
        CHECK(samples == (uint64_t)rates[r] * 3600);
        CHECK(lo == base);
        CHECK(hi == base + (rates[r] % 1000 ? 1 : 0));
    }

    // Clamped to the endpoint's capacity like the feedback path
    acc = 0;
    CHECK(uac2_tx_next_nominal_samples(44100, &acc, 40) == 40);
}

int main(void)
{
    test_rates();
    test_feedback_plausibility();
    test_clamp_never_exceeds_mps();
    test_feedback_decode();
    test_rate_change_keeps_fraction();
    test_nominal_without_feedback();

    if (s_failures) {
        printf("%d check(s) FAILED\n", s_failures);
        return 1;
    }
    printf("All packet sizing checks passed\n");
    return 0;
}
