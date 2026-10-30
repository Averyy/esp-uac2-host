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
    uint32_t ctrl_bad_wlength;            // GET CUR requests whose wLength is not the control's size
    uint32_t ctrl_wrong_interface;        // class requests not addressed to the AC interface (IF0)
    uint32_t ctrl_in_unrounded;           // IN control transfers whose data stage room is not a
                                          // multiple of bMaxPacketSize0 (Espressif's class drivers
                                          // round it up; the HCD receives whole packets)
    uint32_t release_with_pending;        // usb_host_interface_release() while the interface's
                                          // transfers were in flight or not yet handed to their
                                          // callback (espressif/usb then sleeps 10 ticks)
    uint32_t halt_while_active;           // usb_host_endpoint_halt() on a live pipe with a transfer
                                          // executing: the HCD waits with no timeout, and a channel
                                          // error during the wait never wakes it (stop() would hang)
    uint32_t alt0_with_urbs_in_flight;    // SET_INTERFACE(alt 0) while the interface's transfers
                                          // were still with the host
    uint32_t submit_on_inactive_alt;      // isochronous transfer submitted while the device runs
                                          // another alt of that interface (alt 0 after a teardown)
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

/** fake_usb_hang_ctrl() for one bRequest only (0x01 CUR, 0x02 RANGE). */
void fake_usb_hang_ctrl_request(uint8_t entity, uint8_t control_selector, uint8_t request, bool get);

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

/** bMaxPacketSize0 the device descriptor reports (default 64, the miniDSP's). */
void fake_usb_set_ep0_mps(uint8_t mps);

/**
 * On ESP-IDF < 6, frames after a failed control transfer during which EP0
 * refuses submissions with ESP_ERR_INVALID_STATE, as usbh.c does until its
 * library task clears the halted pipe (default 2).
 */
void fake_usb_set_ep0_recovery_frames(int frames);

/** The next count usb_host_transfer_submit() calls on ep_addr fail with ESP_ERR_INVALID_STATE. */
void fake_usb_fail_submits(uint8_t ep_addr, int count);

/**
 * The next transfer completing on ep_addr fails with a channel error, as in
 * hcd_dwc.c: URB status ERROR, the pipe halts, and the transfers queued
 * behind it stay pending (submits are refused) until the pipe is flushed.
 */
void fake_usb_error_halt_endpoint(uint8_t ep_addr);

/** The single volume range feature units report, 1/256 dB (default -90..0 dB in 1 dB steps). */
void fake_usb_set_volume_range(int16_t min, int16_t max, int16_t res);

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

/** Undo fake_usb_stall_ctrl(). */
void fake_usb_stall_ctrl_clear(void);

/**
 * Copy of the captured miniDSP configuration descriptor with every feedback
 * endpoint removed and the playback data endpoints given sync_type
 * (1 async, 2 adaptive, 3 synchronous). Returns the length written to out.
 */
uint16_t fake_usb_no_feedback_config(uint8_t sync_type, uint8_t *out, uint16_t out_size);

/**
 * Copy of the captured miniDSP configuration descriptor with every feedback
 * endpoint's wMaxPacketSize set to fb_mps. A 3-byte endpoint reports
 * feedback in the USB spec's 10.14 format. Returns the length written to out.
 */
uint16_t fake_usb_feedback_mps_config(uint16_t fb_mps, uint8_t *out, uint16_t out_size);

/** Copy of the captured miniDSP configuration descriptor. Returns its length. */
uint16_t fake_usb_minidsp_config(uint8_t *out, uint16_t out_size);

/**
 * The captured miniDSP configuration followed by a USB-MIDI function whose
 * AudioControl interface (IF5) is UAC1 (protocol 0). Returns its length.
 */
uint16_t fake_usb_midi_function_config(uint8_t *out, uint16_t out_size);

/**
 * In a configuration descriptor, replace feature unit unit_id's bmaControls
 * for channel (0 = master). False if the unit or channel is not there.
 */
bool fake_usb_config_set_fu_controls(uint8_t *config, uint16_t len, uint8_t unit_id,
                                     uint8_t channel, uint32_t bma_controls);

/** Snapshot of counters and resource state. */
fake_usb_stats_t fake_usb_stats(void);

/** Zero the playback counters (e.g. to measure only steady state). */
void fake_usb_clear_out_stats(void);

/** Print internal state for diagnosing a hang; safe to call while the fake is wedged. */
void fake_usb_debug_dump(void);
