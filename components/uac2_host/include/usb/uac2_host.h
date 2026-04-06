/*
 * SPDX-FileCopyrightText: 2026 Avery Levitt
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file uac2_host.h
 * @brief USB Audio Class 2.0 host driver for ESP32-S3
 *
 * Provides device management, clock control, volume/mute, and
 * isochronous audio streaming (playback and capture) for UAC2 devices.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "usb/usb_host.h"
#include "usb/uac2_desc.h"

#ifdef __cplusplus
extern "C" {
#endif

// ── Version ───────────────────────────────────────────────────────

#define UAC2_HOST_VER_MAJOR  1
#define UAC2_HOST_VER_MINOR  0
#define UAC2_HOST_VER_PATCH  0

// ── Configuration defaults ─────────────────────────────────────────

// Configurable via Kconfig (menuconfig -> UAC2 Host Driver)
#ifdef CONFIG_UAC2_NUM_ISOC_URBS
#define UAC2_NUM_ISOC_URBS          CONFIG_UAC2_NUM_ISOC_URBS
#else
#define UAC2_NUM_ISOC_URBS          3
#endif

#define UAC2_NUM_PACKETS_PER_URB    1       // Packets per URB (1 = 1ms per URB at FS)

#ifdef CONFIG_UAC2_CTRL_XFER_TIMEOUT_MS
#define UAC2_CTRL_XFER_TIMEOUT_MS   CONFIG_UAC2_CTRL_XFER_TIMEOUT_MS
#else
#define UAC2_CTRL_XFER_TIMEOUT_MS   5000
#endif

#ifdef CONFIG_UAC2_CTRL_XFER_MAX_SIZE
#define UAC2_CTRL_XFER_MAX_SIZE     CONFIG_UAC2_CTRL_XFER_MAX_SIZE
#else
#define UAC2_CTRL_XFER_MAX_SIZE     256
#endif

#ifdef CONFIG_UAC2_MAX_CONSECUTIVE_ERRORS
#define UAC2_MAX_CONSECUTIVE_ERRORS CONFIG_UAC2_MAX_CONSECUTIVE_ERRORS
#else
#define UAC2_MAX_CONSECUTIVE_ERRORS 10
#endif

// ESP32-S3 DWC_OTG FIFO limits (1024 bytes total, bias-dependent):
//   PERIODIC_OUT bias: PTX=600, RX=128, NPTX=64
//   Playback (iso OUT ≤600) + feedback (iso IN ≤128) works fine.
//   Simultaneous capture (iso IN ~294) DOES NOT FIT in RX FIFO (128 max).
//   Full duplex audio requires ESP32-P4 (4KB FIFO) or alternating directions.

// ── Types ──────────────────────────────────────────────────────────

typedef struct uac2_host_device *uac2_host_device_handle_t;

/** Stream direction */
typedef enum {
    UAC2_STREAM_TX = 0,     /**< Playback: host -> device (isochronous OUT) */
    UAC2_STREAM_RX,         /**< Capture:  device -> host (isochronous IN) */
} uac2_stream_dir_t;

/** Device-level events */
typedef enum {
    UAC2_HOST_EVENT_TX_DONE = 0,        /**< Playback ringbuf needs more data */
    UAC2_HOST_EVENT_RX_DONE,            /**< Capture ringbuf has data ready */
    UAC2_HOST_EVENT_TRANSFER_ERROR,     /**< Isochronous transfer error */
    UAC2_HOST_EVENT_DISCONNECTED,       /**< Device disconnected */
} uac2_host_event_t;

/**
 * Event callback. Called from the USB Host client event task context.
 *
 * @warning Must not block. Must not call uac2_host_stream_stop(),
 *          uac2_host_device_close(), or any control request APIs
 *          (set/get sample rate, volume, mute) from this callback —
 *          doing so will deadlock the USB event task.
 */
typedef void (*uac2_host_event_cb_t)(uac2_host_device_handle_t dev,
                                     uac2_host_event_t event, void *arg);

/** Stream configuration */
typedef struct {
    uint32_t sample_rate;           /**< Sample rate in Hz (e.g. 48000) */
    uint8_t  channels;              /**< Number of channels */
    uint8_t  bit_resolution;        /**< Bits per sample (16 or 24) */
    uint32_t ringbuf_size;          /**< Ring buffer size in bytes (0 = auto) */
    uint32_t ringbuf_threshold;     /**< Threshold for TX_DONE / RX_DONE events (0 = half) */
} uac2_stream_config_t;

/** Sample rate range (from GET_RANGE) */
typedef struct {
    uint32_t min;
    uint32_t max;
    uint32_t res;
} uac2_sample_rate_range_t;

#define UAC2_MAX_SAMPLE_RATE_RANGES  16

// ── Device management ──────────────────────────────────────────────

/**
 * Open a UAC2 device. Parses descriptors, allocates control transfer,
 * and prepares the device for streaming and control requests.
 *
 * @param client    USB Host client handle (from usb_host_client_register)
 * @param dev       USB device handle (from usb_host_device_open)
 * @param cb        Event callback (may be NULL)
 * @param cb_arg    User argument passed to callback
 * @param out_dev   Output: UAC2 device handle
 */
esp_err_t uac2_host_device_open(usb_host_client_handle_t client,
                                usb_device_handle_t dev,
                                uac2_host_event_cb_t cb, void *cb_arg,
                                uac2_host_device_handle_t *out_dev);

/**
 * Close a UAC2 device. Stops any active streams, frees driver resources.
 *
 * @note Does NOT call usb_host_device_close() on the underlying USB device.
 *       The caller must close the USB device handle separately.
 */
esp_err_t uac2_host_device_close(uac2_host_device_handle_t dev);

/**
 * Get parsed descriptor info for the device.
 */
esp_err_t uac2_host_device_get_info(uac2_host_device_handle_t dev,
                                    uac2_device_info_t *info);

// ── Clock control ──────────────────────────────────────────────────

/**
 * Get current sample rate from the device's clock source.
 */
esp_err_t uac2_host_get_sample_rate(uac2_host_device_handle_t dev,
                                    uint32_t *sample_rate);

/**
 * Set sample rate on the device's clock source.
 */
esp_err_t uac2_host_set_sample_rate(uac2_host_device_handle_t dev,
                                    uint32_t sample_rate);

/**
 * Query supported sample rate ranges.
 *
 * @param ranges     Output array (caller provides UAC2_MAX_SAMPLE_RATE_RANGES)
 * @param num_ranges Output: number of ranges filled
 */
esp_err_t uac2_host_get_sample_rate_range(uac2_host_device_handle_t dev,
                                          uac2_sample_rate_range_t *ranges,
                                          uint8_t *num_ranges);

/**
 * Check if the clock source is valid (synced).
 */
esp_err_t uac2_host_get_clock_valid(uac2_host_device_handle_t dev,
                                    bool *valid);

// ── Streaming ──────────────────────────────────────────────────────

/**
 * Start an audio stream. Claims interface, sets alternate setting,
 * allocates URBs, and begins isochronous transfers.
 *
 * For TX (playback): transfers start silent, waiting for data via
 * uac2_host_stream_write(). The UAC2_HOST_EVENT_TX_DONE event fires
 * when the ring buffer needs data.
 *
 * For RX (capture): transfers start immediately. The
 * UAC2_HOST_EVENT_RX_DONE event fires when audio data is available.
 */
esp_err_t uac2_host_stream_start(uac2_host_device_handle_t dev,
                                 uac2_stream_dir_t dir,
                                 const uac2_stream_config_t *config);

/**
 * Stop a stream. Releases interface, frees URBs and ring buffer.
 */
esp_err_t uac2_host_stream_stop(uac2_host_device_handle_t dev,
                                uac2_stream_dir_t dir);

/**
 * Write audio data to the playback ring buffer.
 * Data format must match the stream config (interleaved PCM).
 */
esp_err_t uac2_host_stream_write(uac2_host_device_handle_t dev,
                                 const uint8_t *data, uint32_t size,
                                 uint32_t timeout_ms);

/**
 * Read audio data from the capture ring buffer.
 */
esp_err_t uac2_host_stream_read(uac2_host_device_handle_t dev,
                                uint8_t *data, uint32_t size,
                                uint32_t *bytes_read,
                                uint32_t timeout_ms);

/**
 * Get the hardware timestamp (microseconds) of when the first isochronous
 * URB was submitted for the active TX stream. Returns 0 if no stream is active.
 * Used by callers that need precise playback start timing (e.g., measurement sweeps).
 */
int64_t uac2_host_stream_get_start_time(uac2_host_device_handle_t dev);

// ── Volume / Mute ──────────────────────────────────────────────────

/**
 * Set mute on a feature unit channel.
 * @param channel  0 = master, 1+ = individual channels
 */
esp_err_t uac2_host_set_mute(uac2_host_device_handle_t dev,
                             uint8_t channel, bool mute);

/**
 * Get current mute state.
 */
esp_err_t uac2_host_get_mute(uac2_host_device_handle_t dev,
                             uint8_t channel, bool *mute);

/**
 * Set volume on a feature unit channel.
 * @param volume_db256  Volume in 1/256 dB units (e.g. 0x0100 = +1 dB)
 */
esp_err_t uac2_host_set_volume(uac2_host_device_handle_t dev,
                               uint8_t channel, int16_t volume_db256);

/**
 * Get current volume.
 */
esp_err_t uac2_host_get_volume(uac2_host_device_handle_t dev,
                               uint8_t channel, int16_t *volume_db256);

#ifdef __cplusplus
}
#endif
