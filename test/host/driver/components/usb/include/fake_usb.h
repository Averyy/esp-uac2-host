/*
 * SPDX-License-Identifier: MIT
 *
 * Control and inspection API for the fake USB Host Library that this test
 * project builds in place of ESP-IDF's `usb` component. The fake implements
 * the subset of usb/usb_host.h the UAC2 driver uses, with one simulated
 * device (by default the captured miniDSP 2x4 HD) on a 1 ms "bus" task.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "usb/usb_types_stack.h"

typedef struct {
    // Playback (isochronous OUT data endpoint), counted for packets that completed
    uint64_t out_packets;
    uint64_t out_bytes;
    uint32_t out_min_packet;
    uint32_t out_max_packet;
    uint16_t first_out_packets[32];       // sizes of the first completed OUT data packets
    uint64_t out_nonzero_bytes;           // playback bytes other than 0 (silence padding)
    uint32_t out_pattern_breaks;          // nonzero playback bytes that do not continue the
                                          // writers' 1..255 counter (lost, repeated, corrupted)
    uint64_t out_silence_after_data;      // zero (padding) bytes after the first nonzero byte:
                                          // silence inserted while a writer keeps the ring full
    int first_out_count;
    // Control requests
    uint32_t ctrl_requests;
    uint32_t set_rate_calls;
    uint32_t last_set_rate;
    uint32_t set_rate_while_streaming;    // SET CUR sample rate with OUT URBs in flight
    uint32_t set_interface_calls;
    uint32_t alt_enabled_without_rate;    // SET_INTERFACE(alt != 0) with no sample-rate SET since
                                          // the previous one (the clock must be set first)
    // Contract violations by the driver (each one is a test failure)
    uint32_t free_while_in_flight;        // usb_host_transfer_free() on a pending transfer
    uint32_t close_with_ctrl_in_flight;   // last device close with EP0 busy (asserts in ESP-IDF)
    uint32_t submit_rejected;             // submits the fake refused (bad state / endpoint)
    uint32_t invalid_transfers;           // transfers the real library would reject or assert on
    uint32_t release_with_pending;        // usb_host_interface_release() while the interface's
                                          // transfers were in flight or not yet handed to their
                                          // callback (espressif/usb then sleeps 10 ticks)
    // Resources
    int transfers_allocated;
    int device_open_count;
    int interfaces_claimed;
    bool device_freed;                    // gone and fully closed (ALL_FREE)
    bool client_registered;
} fake_usb_stats_t;

/** Forget all state. Call before each test, with no client registered. */
void fake_usb_reset(void);

/**
 * Attach a device with this configuration descriptor (NULL: the captured
 * miniDSP). Registered clients get NEW_DEV.
 */
void fake_usb_attach(const uint8_t *config_desc, uint16_t len);

/*
 * Capture (isochronous IN data) packets carry 288 bytes of a running byte
 * counter (each byte is the previous one + 1, mod 256), placed in the transfer
 * buffer at the HCD's offsets.
 */

/** Unplug: pending transfers complete with NO_DEVICE, clients holding it get DEV_GONE. */
void fake_usb_detach(void);

/** Feedback reported on the feedback endpoint, 16.16 samples/frame (default 48.0). */
void fake_usb_set_feedback_q16(uint32_t q16);

/** The next feedback packets complete with this packet status (e.g. SKIPPED). */
void fake_usb_set_feedback_packet_status(usb_transfer_status_t status, int count);

/** The next OUT data URBs complete with every packet in this status. */
void fake_usb_fail_out_urbs(usb_transfer_status_t packet_status, int count);

/**
 * Leave the next class control request to entity/control selector unanswered
 * (bRequest 0x01 CUR / 0x02 RANGE, direction IN when get is true).
 */
void fake_usb_hang_ctrl(uint8_t entity, uint8_t control_selector, bool get);

/** Answer the hung control request now (a late reply). Returns false if none. */
bool fake_usb_release_hung_ctrl(void);

/** usb_host_interface_release() fails with ESP_ERR_INVALID_STATE this many times (ESP-IDF #17707). */
void fake_usb_set_release_busy(int count);

/**
 * usb_host_interface_claim() refuses (ESP_ERR_NOT_SUPPORTED) an alt whose
 * isochronous OUT endpoint has a max packet size above limit, as the HCD does
 * when it exceeds the configured periodic TX FIFO. 0 removes the limit.
 */
void fake_usb_set_max_iso_out_mps(uint16_t limit);

/**
 * Sample rates the clock source advertises in its RANGE, as discrete
 * (min == max) triplets; at most 21 fit the fake's reply. count 0 restores
 * the default 44.1 and 48 kHz.
 */
void fake_usb_set_clock_rates(const uint32_t *rates, int count);

/** Let n more usb_host_transfer_alloc() calls succeed, then fail one with ESP_ERR_NO_MEM. */
void fake_usb_fail_transfer_alloc_after(int n);

/** STALL every class request to entity/control selector/channel in this direction. */
void fake_usb_stall_ctrl(uint8_t entity, uint8_t control_selector, uint8_t channel, bool get);

/**
 * Copy of the captured miniDSP configuration descriptor with every feedback
 * endpoint removed and the playback data endpoints given sync_type
 * (1 async, 2 adaptive, 3 synchronous). Returns the length written to out.
 */
uint16_t fake_usb_no_feedback_config(uint8_t sync_type, uint8_t *out, uint16_t out_size);

/** Snapshot of counters and resource state. */
fake_usb_stats_t fake_usb_stats(void);

/** Zero the playback counters (e.g. to measure only steady state). */
void fake_usb_clear_out_stats(void);

/** Print internal state for diagnosing a hang; safe to call while the fake is wedged. */
void fake_usb_debug_dump(void);
