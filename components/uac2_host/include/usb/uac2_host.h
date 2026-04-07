/*
 * SPDX-FileCopyrightText: 2026 Avery Levitt
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file uac2_host.h
 * @brief USB Audio Class 2.0 host driver for ESP32-S3
 *
 * Espressif-pattern class driver: install/uninstall lifecycle with internal
 * device discovery. Provides clock control, volume/mute, and isochronous
 * audio streaming (playback and capture) for UAC2 devices.
 *
 * Usage:
 *   1. usb_host_install()
 *   2. uac2_host_install(&driver_config)   — driver registers USB client
 *   3. Driver fires callback on connect: UAC2_HOST_DRIVER_EVENT_TX_CONNECTED
 *   4. uac2_host_device_open(&device_config, &handle)
 *   5. uac2_host_device_start(handle, &stream_config)
 *   6. uac2_host_device_write(handle, data, size, timeout)
 *   7. uac2_host_device_stop(handle)
 *   8. uac2_host_device_close(handle)
 *   9. uac2_host_uninstall()
 *
 * TX (playback) silence behavior: when the ring buffer is empty, isochronous
 * URBs are filled with zeros (silence) and never stop. The stream remains
 * active until explicitly stopped. UAC2_HOST_DEVICE_EVENT_TX_DONE fires
 * when the ring buffer needs more data.
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

#define UAC2_HOST_VER_MAJOR  0
#define UAC2_HOST_VER_MINOR  1
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

/**
 * @brief Flag that starts the stream in a suspended state.
 *
 * When set in uac2_host_stream_config_t::flags, uac2_host_device_start()
 * claims the interface and prepares resources without submitting stream
 * transfers. Call uac2_host_device_resume() later to start transfers.
 */
#define UAC2_FLAG_STREAM_SUSPEND_AFTER_START  (1 << 0)

// ── Types ──────────────────────────────────────────────────────────

typedef struct uac2_interface *uac2_host_device_handle_t;

/** Stream direction */
typedef enum {
    UAC2_STREAM_TX = 0,     /**< Playback: host -> device (isochronous OUT) */
    UAC2_STREAM_RX,         /**< Capture:  device -> host (isochronous IN) */
} uac2_stream_dir_t;

// ── Driver-level events and callback ───────────────────────────────

/** Driver-level events (fired during device discovery) */
typedef enum {
    UAC2_HOST_DRIVER_EVENT_TX_CONNECTED = 0, /**< UAC2 playback interface found */
    UAC2_HOST_DRIVER_EVENT_RX_CONNECTED,     /**< UAC2 capture interface found */
} uac2_host_driver_event_t;

/**
 * Driver-level event callback. Fired when a UAC2 device connects and
 * streaming interfaces are discovered. Use addr + iface_num to open
 * the interface via uac2_host_device_open().
 *
 * @warning Called from the USB Host client event task. Must not block.
 */
typedef void (*uac2_host_driver_event_cb_t)(uint8_t addr, uint8_t iface_num,
                                            const uac2_host_driver_event_t event,
                                            void *arg);

// ── Device-level events and callback ───────────────────────────────

/** Device/interface-level events (fired during streaming) */
typedef enum {
    UAC2_HOST_DEVICE_EVENT_TX_DONE = 0,     /**< Playback ringbuf needs more data */
    UAC2_HOST_DEVICE_EVENT_RX_DONE,         /**< Capture ringbuf has data ready */
    UAC2_HOST_DEVICE_EVENT_TRANSFER_ERROR,  /**< Isochronous transfer error */
    UAC2_HOST_DEVICE_EVENT_DISCONNECTED,    /**< Device disconnected. Caller MUST call
                                                 uac2_host_device_close() from a non-callback
                                                 context to release stream resources. */
    UAC2_HOST_DEVICE_EVENT_STREAM_ERROR,    /**< Stream dead after max consecutive errors */
} uac2_host_device_event_t;

/**
 * Device/interface event callback. Fired from USB Host client event task.
 *
 * @warning Must not block. Must not call uac2_host_device_stop(),
 *          uac2_host_device_close(), or any control request APIs
 *          from this callback — doing so will deadlock.
 */
typedef void (*uac2_host_device_event_cb_t)(uac2_host_device_handle_t dev,
                                            const uac2_host_device_event_t event,
                                            void *arg);

// ── Configuration structs ──────────────────────────────────────────

/** UAC2 driver configuration (passed to uac2_host_install) */
typedef struct {
    bool create_background_task;       /**< true: driver creates event task; false: caller pumps events */
    size_t task_priority;              /**< Priority of background task (if created) */
    size_t stack_size;                 /**< Stack size of background task (if created) */
    BaseType_t core_id;                /**< Core affinity of background task, or tskNO_AFFINITY */
    uac2_host_driver_event_cb_t callback; /**< Driver event callback. Must not be NULL. */
    void *callback_arg;                /**< User argument passed to driver callback */
} uac2_host_driver_config_t;

/** UAC2 device/interface open configuration */
typedef struct {
    uint8_t addr;                      /**< USB device address (from driver callback) */
    uint8_t iface_num;                 /**< Interface number (from driver callback) */
    uint32_t buffer_size;              /**< Ring buffer size in bytes (0 = auto ~100ms) */
    uint32_t buffer_threshold;         /**< Threshold for TX_DONE/RX_DONE events (0 = half) */
    uac2_host_device_event_cb_t callback; /**< Device event callback (may be NULL) */
    void *callback_arg;                /**< User argument passed to device callback */
} uac2_host_device_config_t;

/** Stream configuration (passed to uac2_host_device_start) */
typedef struct {
    uint8_t  channels;                 /**< Number of channels (0 = match any) */
    uint8_t  bit_resolution;           /**< Bits per sample: 16 or 24 (0 = match best) */
    uint32_t sample_freq;              /**< Sample rate in Hz (e.g. 48000) */
    uint16_t flags;                    /**< Control flags (e.g. UAC2_FLAG_STREAM_SUSPEND_AFTER_START) */
} uac2_host_stream_config_t;

/** Sample rate range (from GET_RANGE) */
typedef struct {
    uint32_t min;
    uint32_t max;
    uint32_t res;
} uac2_sample_rate_range_t;

#define UAC2_MAX_SAMPLE_RATE_RANGES  16

/** Volume range (from GET_RANGE on Feature Unit) */
typedef struct {
    int16_t min;    /**< Minimum volume in 1/256 dB */
    int16_t max;    /**< Maximum volume in 1/256 dB */
    int16_t res;    /**< Volume resolution in 1/256 dB */
} uac2_volume_range_t;

#define UAC2_MAX_VOLUME_RANGES  8

/** Alternate setting parameters (from uac2_host_get_device_alt_param) */
typedef struct {
    uint8_t  alt_setting;              /**< Alternate setting number */
    uint8_t  channels;                 /**< Number of channels */
    uint8_t  bit_resolution;           /**< Bits per sample */
    uint8_t  sub_slot_size;            /**< Bytes per sample slot */
    uint16_t ep_max_packet_size;       /**< Endpoint max packet size */
    uint8_t  ep_addr;                  /**< Endpoint address */
    uint8_t  fb_ep_addr;              /**< Feedback endpoint address (0 if none) */
} uac2_host_dev_alt_param_t;

// ── Driver lifecycle ───────────────────────────────────────────────

/**
 * Install the UAC2 host class driver.
 * Registers a USB Host client, optionally creates a background event task.
 * Call after usb_host_install().
 */
esp_err_t uac2_host_install(const uac2_host_driver_config_t *config);

/**
 * Uninstall the UAC2 host class driver.
 * All devices must be closed first.
 */
esp_err_t uac2_host_uninstall(void);

/**
 * Handle USB Host events for the UAC2 driver.
 * Call periodically when create_background_task = false.
 *
 * @return ESP_OK on success, ESP_FAIL when driver is being uninstalled
 */
esp_err_t uac2_host_handle_events(TickType_t timeout);

// ── Device management ──────────────────────────────────────────────

/**
 * Open a UAC2 audio interface. Obtain addr and iface_num from the
 * driver event callback (UAC2_HOST_DRIVER_EVENT_TX/RX_CONNECTED).
 *
 * If this is the first interface opened for the device address, the
 * USB device is opened and descriptors are parsed. Subsequent opens
 * for the same address share the physical device (reference counted).
 */
esp_err_t uac2_host_device_open(const uac2_host_device_config_t *config,
                                uac2_host_device_handle_t *out_handle);

/**
 * Close a UAC2 interface. Stops any active stream. When the last
 * interface for a device is closed, the USB device is released.
 */
esp_err_t uac2_host_device_close(uac2_host_device_handle_t dev);

/**
 * Get parsed descriptor info for the device (all interfaces, topology).
 */
esp_err_t uac2_host_device_get_info(uac2_host_device_handle_t dev,
                                    uac2_device_info_t *info);

/**
 * Get alternate setting parameters for this interface.
 *
 * @param alt  Alternate setting number (1-based: first alt is 1).
 */
esp_err_t uac2_host_get_device_alt_param(uac2_host_device_handle_t dev,
                                         uint8_t alt,
                                         uac2_host_dev_alt_param_t *param);

/**
 * Print full device info: topology, clock, feature unit capabilities,
 * stream state, endpoints, and cached volume range.
 */
void uac2_host_device_print_info(uac2_host_device_handle_t dev);

// ── Clock control ──────────────────────────────────────────────────

/**
 * Get current sample rate from the device's clock source.
 */
esp_err_t uac2_host_device_get_sample_rate(uac2_host_device_handle_t dev,
                                           uint32_t *sample_rate);

/**
 * Set sample rate on the device's clock source.
 */
esp_err_t uac2_host_device_set_sample_rate(uac2_host_device_handle_t dev,
                                           uint32_t sample_rate);

/**
 * Query supported sample rate ranges.
 */
esp_err_t uac2_host_device_get_sample_rate_range(uac2_host_device_handle_t dev,
                                                 uac2_sample_rate_range_t *ranges,
                                                 uint8_t *num_ranges);

/**
 * Check if the clock source is valid (synced).
 */
esp_err_t uac2_host_device_get_clock_valid(uac2_host_device_handle_t dev,
                                           bool *valid);

// ── Streaming ──────────────────────────────────────────────────────

/**
 * Start the audio stream. Claims interface, sets alternate setting,
 * allocates URBs and ring buffer, begins isochronous transfers.
 * Stream direction is determined by the interface (TX or RX).
 */
esp_err_t uac2_host_device_start(uac2_host_device_handle_t dev,
                                 const uac2_host_stream_config_t *config);

/**
 * Stop the stream. Releases interface, frees URBs and ring buffer.
 */
esp_err_t uac2_host_device_stop(uac2_host_device_handle_t dev);

/**
 * Suspend an active stream without freeing resources.
 * Stream transitions from ACTIVE to READY.
 */
esp_err_t uac2_host_device_suspend(uac2_host_device_handle_t dev);

/**
 * Resume a suspended stream. No reallocation needed.
 * Stream transitions from READY to ACTIVE.
 */
esp_err_t uac2_host_device_resume(uac2_host_device_handle_t dev);

/**
 * Write audio data to the playback ring buffer.
 *
 * @note This function takes a lightweight runtime reference to the handle.
 *       It returns `ESP_ERR_INVALID_STATE` if the stream is not active or if
 *       the handle is closing/closed/disconnected.
 */
esp_err_t uac2_host_device_write(uac2_host_device_handle_t dev,
                                 const uint8_t *data, uint32_t size,
                                 uint32_t timeout_ms);

/**
 * Read audio data from the capture ring buffer.
 *
 * @note This function takes a lightweight runtime reference to the handle.
 *       It returns `ESP_ERR_INVALID_STATE` if the stream is not active or if
 *       the handle is closing/closed/disconnected.
 */
esp_err_t uac2_host_device_read(uac2_host_device_handle_t dev,
                                uint8_t *data, uint32_t size,
                                uint32_t *bytes_read,
                                uint32_t timeout_ms);

/**
 * Get the hardware timestamp of when the first isochronous URB was submitted.
 * Returns 0 if no stream is active.
 *
 * @note Returns 0 if the handle is closing/closed/disconnected.
 */
int64_t uac2_host_device_get_start_time(uac2_host_device_handle_t dev);

/**
 * Get the current feedback value (async playback only).
 * Returns 16.16 fixed-point (samples per frame). 0 if no feedback.
 *
 * @note Returns 0 if the handle is closing/closed/disconnected.
 */
uint32_t uac2_host_device_get_feedback(uac2_host_device_handle_t dev);

// ── Volume / Mute ──────────────────────────────────────────────────

/**
 * Set mute on a feature unit channel.
 * @param channel  0 = master, 1+ = individual channels
 */
esp_err_t uac2_host_device_set_mute(uac2_host_device_handle_t dev,
                                    uint8_t channel, bool mute);

/**
 * Get current mute state.
 */
esp_err_t uac2_host_device_get_mute(uac2_host_device_handle_t dev,
                                    uint8_t channel, bool *mute);

/**
 * Set volume on a feature unit channel.
 * @param volume_db256  Volume in 1/256 dB units (e.g. 0x0100 = +1 dB)
 */
esp_err_t uac2_host_device_set_volume(uac2_host_device_handle_t dev,
                                      uint8_t channel, int16_t volume_db256);

/**
 * Get current volume.
 */
esp_err_t uac2_host_device_get_volume(uac2_host_device_handle_t dev,
                                      uint8_t channel, int16_t *volume_db256);

/**
 * Query supported volume ranges from the feature unit.
 */
esp_err_t uac2_host_device_get_volume_range(uac2_host_device_handle_t dev,
                                            uint8_t channel,
                                            uac2_volume_range_t *ranges,
                                            uint8_t *num_ranges);

/**
 * Set volume as a percentage (0-100). Maps linearly from min to max dB.
 */
esp_err_t uac2_host_device_set_volume_percent(uac2_host_device_handle_t dev,
                                              uint8_t channel, uint8_t percent);

/**
 * Get current volume as a percentage (0-100).
 */
esp_err_t uac2_host_device_get_volume_percent(uac2_host_device_handle_t dev,
                                              uint8_t channel, uint8_t *percent);

/**
 * Set volume on all channels that support volume control.
 * Iterates channels based on the feature unit's bmaControls bitmap.
 * Stops on first error and returns that error code.
 *
 * @param volume_db256  Volume in 1/256 dB units (applied to every channel)
 */
esp_err_t uac2_host_device_set_volume_all_channels(uac2_host_device_handle_t dev,
                                                   int16_t volume_db256);

#ifdef __cplusplus
}
#endif
