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
 *   4. Callback notifies application task with addr + iface_num
 *   5. Application task calls uac2_host_device_open(&device_config, &handle)
 *   6. Application task calls uac2_host_device_start(handle, &stream_config)
 *   7. uac2_host_device_write(handle, data, size, timeout)
 *   8. uac2_host_device_stop(handle)
 *   9. uac2_host_device_close(handle)
 *  10. uac2_host_uninstall()
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
#define UAC2_HOST_VER_PATCH  3

// ── Configuration defaults ─────────────────────────────────────────

// Configurable via Kconfig (menuconfig -> UAC2 Host Driver)
#ifdef CONFIG_UAC2_NUM_ISOC_URBS
#define UAC2_NUM_ISOC_URBS          CONFIG_UAC2_NUM_ISOC_URBS
#else
#define UAC2_NUM_ISOC_URBS          3
#endif

#ifdef CONFIG_UAC2_NUM_PACKETS_PER_URB
#define UAC2_NUM_PACKETS_PER_URB    CONFIG_UAC2_NUM_PACKETS_PER_URB
#else
#define UAC2_NUM_PACKETS_PER_URB    3
#endif

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

// ESP32-S3 DWC_OTG FIFO (1024 bytes physical, 800 usable; limits per endpoint):
//   PERIODIC_OUT bias: iso OUT <= 600, IN <= 128, control OUT <= 64.
//   Playback (iso OUT) + feedback (iso IN, 4 bytes) fits; capture (iso IN ~294)
//   does not. For playback and capture at once, install the USB Host Library
//   with UAC2_HOST_DUPLEX_FIFO_* (below) and enable CONFIG_UAC2_ALLOW_DUPLEX.

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
 * streaming interfaces are discovered. Use addr + iface_num to notify
 * an application task, then open the interface there via
 * uac2_host_device_open().
 *
 * Devices that were already attached when the driver was installed are
 * reported the same way on the first uac2_host_handle_events() call, so a
 * driver reinstall does not require the device to be replugged.
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
    size_t task_priority;              /**< Priority of background task (if created). Keep it above
                                            tasks that call write()/read() in a loop: this task
                                            refills/drains the ring buffer, and a higher-priority
                                            task that never blocks (e.g. write() with a 0 timeout in
                                            a loop) starves it, stalling the stream and leaving
                                            close()/uninstall() waiting on it. */
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

/**
 * USB controller FIFO split for one playback and one capture stream at once on
 * ESP32-S3, for usb_host_config_t.fifo_settings_custom (ESP-IDF 5.5+ or the
 * espressif/usb component) together with CONFIG_UAC2_ALLOW_DUPLEX.
 *
 * The S3 has 200 usable FIFO lines of 4 bytes. The host accepts an isochronous
 * OUT endpoint up to ptx_lines * 4 bytes and any IN endpoint up to
 * (rx_lines - 2) * 4 bytes, so this split takes playback endpoints up to 360
 * bytes and capture endpoints up to 368 (48 kHz 24-bit stereo uses 288-294),
 * plus 64-byte control transfers. Larger endpoints need a different split.
 *
 * The device must also support both directions at once; this is unverified
 * on the miniDSP 2x4 HD (see the repo README's Limitations).
 */
#define UAC2_HOST_DUPLEX_FIFO_NPTX_LINES    16
#define UAC2_HOST_DUPLEX_FIFO_PTX_LINES     90
#define UAC2_HOST_DUPLEX_FIFO_RX_LINES      94

/** Synchronization type of an isochronous data endpoint (bmAttributes bits 3..2) */
typedef enum {
    UAC2_EP_SYNC_NONE = 0,
    UAC2_EP_SYNC_ASYNC = 1,                /**< Device clock; playback follows its feedback */
    UAC2_EP_SYNC_ADAPTIVE = 2,             /**< Device follows the host's data rate */
    UAC2_EP_SYNC_SYNC = 3,                 /**< Device clock locked to the USB frame timing */
} uac2_ep_sync_type_t;

/** A started stream's format and endpoints (from uac2_host_device_get_stream_info) */
typedef struct {
    uint8_t  alt_setting;              /**< Alternate setting the stream runs on */
    uint8_t  channels;                 /**< Number of channels */
    uint8_t  bit_resolution;           /**< Valid bits per sample */
    uint8_t  sub_slot_size;            /**< Bytes per sample in the data written/read
                                            (can exceed bit_resolution / 8, e.g. 24 bits in 4) */
    uint32_t sample_freq;              /**< Sample rate in Hz */
    uint16_t ep_max_packet_size;       /**< Data endpoint max packet size */
    uint8_t  ep_addr;                  /**< Data endpoint address */
    uint8_t  fb_ep_addr;               /**< Feedback endpoint address (0 if none) */
    uac2_ep_sync_type_t sync_type;     /**< Data endpoint synchronization type */
    bool     active;                   /**< false while suspended */
} uac2_host_stream_info_t;

/** Audio buffered by the driver (from uac2_host_device_get_buffer_status) */
typedef struct {
    uint32_t ring_bytes;               /**< In the ring buffer: TX written and not yet
                                            sent, RX received and not yet read */
    uint32_t ring_capacity;            /**< Ring buffer size in bytes */
    uint32_t ring_us;                  /**< ring_bytes as duration at the stream's rate */
    uint32_t in_flight_us;             /**< Audio in submitted isochronous URBs, about
                                            CONFIG_UAC2_NUM_ISOC_URBS * CONFIG_UAC2_NUM_PACKETS_PER_URB ms */
} uac2_host_buffer_status_t;

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
 *
 * With create_background_task, waits for that task to exit before returning.
 *
 * @return ESP_OK if uninstalled or not installed; ESP_ERR_INVALID_STATE if
 *         interfaces are still open, another uninstall is in progress, or a
 *         control transfer that timed out on a hung device is still pending
 *         in the USB Host stack (it completes when the device responds or is
 *         disconnected; retry after that).
 */
esp_err_t uac2_host_uninstall(void);

/**
 * Handle USB Host events for the UAC2 driver.
 * Call periodically when create_background_task = false.
 *
 * @note uac2_host_uninstall() unblocks a call that is in progress and waits
 *       for it to return; it does not wait for calls that have not started.
 *       It may therefore be called from the pumping task between calls, or
 *       after pumping has stopped, but not from inside a driver callback
 *       (callbacks run within this call, which uninstall would then wait on).
 *
 * @note Only one call may run at a time (USB Host client events support one
 *       caller), and none while the background task is in use.
 *
 * @return ESP_OK on success; ESP_FAIL once uninstall has started (stop
 *         pumping); ESP_ERR_INVALID_STATE if the driver is not installed or
 *         another call is already running
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
 *
 * @return ESP_ERR_INVALID_STATE if the device was closed while a control
 *         transfer that timed out was still pending: it cannot be reopened
 *         until that transfer completes (the device responds or is unplugged)
 */
esp_err_t uac2_host_device_open(const uac2_host_device_config_t *config,
                                uac2_host_device_handle_t *out_handle);

/**
 * Close a UAC2 interface. Stops any active stream. When the last
 * interface for a device is closed, the USB device is released.
 *
 * On failure the interface stays open and the handle stays valid: keep it
 * and retry, or it remains open (blocking reopen and uninstall) until reboot.
 *
 * @return ESP_ERR_INVALID_ARG if the handle is not open (already closed);
 *         ESP_ERR_TIMEOUT if stream teardown timed out (the interface release
 *         kept being refused, or in-flight transfers did not finish), or if
 *         another call on the handle or a pending read/write/telemetry call
 *         did not finish in time; another stream teardown error if releasing
 *         the interface failed; ESP_ERR_INVALID_STATE if the driver is not
 *         installed or is uninstalling
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
 *
 * The clock is shared by all interfaces on the device, so this returns
 * ESP_ERR_INVALID_STATE while any stream on the device is started or
 * suspended. uac2_host_device_start() applies the stream's own sample_freq.
 *
 * @return ESP_ERR_NOT_SUPPORTED if the device's advertised sample-rate ranges
 *         (including their resolution) exclude sample_rate; see
 *         CONFIG_UAC2_ALLOW_UNADVERTISED_RATES. If the ranges cannot be read,
 *         or the device lists more than UAC2_MAX_SAMPLE_RATE_RANGES, a rate not
 *         found is sent anyway.
 */
esp_err_t uac2_host_device_set_sample_rate(uac2_host_device_handle_t dev,
                                           uint32_t sample_rate);

/**
 * Query supported sample rate ranges.
 *
 * @param ranges      Output array
 * @param num_ranges  In: capacity of ranges, or 0 for UAC2_MAX_SAMPLE_RATE_RANGES
 *                    (the array must then hold that many). Out: entries written;
 *                    extra ranges beyond the capacity are dropped.
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
 * Start the audio stream. Claims the interface, sets the device clock to
 * sample_freq, activates the alternate setting, allocates URBs and ring buffer,
 * begins isochronous transfers. Stream direction is determined by the
 * interface (TX or RX).
 *
 * Alternate settings matching channels/bit_resolution are tried best first
 * (highest bit depth first when bit_resolution is 0). If one cannot carry the
 * stream (packet size above its max packet size, endpoints the USB host cannot
 * fit, unsupported intervals, a fractional rate without the feedback it needs),
 * the next is tried. uac2_host_device_get_stream_info() reports the one used.
 *
 * Fractional rates (44.1 kHz) on playback need a feedback endpoint when the
 * data endpoint is asynchronous; adaptive and synchronous endpoints get the
 * exact nominal packet pattern without one.
 *
 * @return ESP_ERR_INVALID_STATE if a stream is already started, the device
 *         was disconnected (close the handle), a previous stream on this
 *         handle was not torn down (stop or close it first), or another
 *         interface of the same device has a started or suspended stream at a
 *         different sample_freq (the device has one clock);
 *         ESP_ERR_NOT_FOUND if no alternate setting has the requested format;
 *         ESP_ERR_NOT_SUPPORTED if the device's advertised sample-rate ranges
 *         exclude sample_freq (see uac2_host_device_set_sample_rate()), or on
 *         ESP32-S3 if a stream in the opposite direction exists (unless
 *         CONFIG_UAC2_ALLOW_DUPLEX). If no alt can carry the stream, the last
 *         one tried decides: ESP_ERR_NOT_SUPPORTED (descriptor checks, or
 *         endpoints the host cannot fit) or ESP_ERR_INVALID_SIZE (packet size
 *         above its max packet size)
 */
esp_err_t uac2_host_device_start(uac2_host_device_handle_t dev,
                                 const uac2_host_stream_config_t *config);

/**
 * Stop the stream. Releases interface, frees URBs and ring buffer.
 *
 * Audio accepted by write() that is still in the ring buffer is discarded;
 * audio already in submitted URBs may or may not play. To play everything
 * written, wait for both to drain before stopping: up to buffer_size bytes
 * in the ring buffer plus
 * CONFIG_UAC2_NUM_ISOC_URBS * CONFIG_UAC2_NUM_PACKETS_PER_URB ms in flight.
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
 *
 * @return ESP_ERR_INVALID_STATE if another stream on the device now runs at a
 *         different sample rate; ESP_ERR_NOT_SUPPORTED as for start()
 */
esp_err_t uac2_host_device_resume(uac2_host_device_handle_t dev);

/**
 * Write audio data to the playback ring buffer.
 *
 * Returns once the data is queued, not played (see uac2_host_device_stop()).
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
 * Get the esp_timer_get_time() timestamp (us since boot) of when start() or
 * resume() submitted the first isochronous URB. Returns 0 if no stream is active.
 *
 * For playback the initial URBs carry silence, so the first written sample
 * goes out about CONFIG_UAC2_NUM_ISOC_URBS * CONFIG_UAC2_NUM_PACKETS_PER_URB ms
 * (9 ms by default) after this time, later if the ring buffer was still
 * empty when they completed. Device output latency is not included.
 *
 * @note Returns 0 if the handle is closing/closed/disconnected, so read it
 *       while the stream is running; 0 means there is no valid start time.
 */
int64_t uac2_host_device_get_start_time(uac2_host_device_handle_t dev);

/**
 * Get the current feedback value (async playback only).
 * Returns 16.16 fixed-point (samples per frame). 0 if no feedback.
 *
 * @note Returns 0 if the handle is closing/closed/disconnected.
 */
uint32_t uac2_host_device_get_feedback(uac2_host_device_handle_t dev);

/**
 * Get the format and endpoints of a started (or suspended) stream: which
 * alternate setting start() chose, and the sample layout the data written or
 * read must use (sub_slot_size bytes per sample, channels interleaved).
 *
 * @return ESP_ERR_INVALID_STATE if the stream is not started
 */
esp_err_t uac2_host_device_get_stream_info(uac2_host_device_handle_t dev,
                                           uac2_host_stream_info_t *info);

/**
 * Get how much audio the driver holds for an active stream. For playback,
 * ring_us + in_flight_us approximates the delay from write() to the bus (add
 * the device's own output latency); to play everything written before
 * stop(), wait until ring_bytes reaches 0 and then in_flight_us.
 *
 * @note Takes a lightweight runtime reference, like write()/read().
 * @return ESP_ERR_INVALID_STATE if the stream is not active or the handle is
 *         closing/closed/disconnected
 */
esp_err_t uac2_host_device_get_buffer_status(uac2_host_device_handle_t dev,
                                             uac2_host_buffer_status_t *status);

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
 *
 * @param ranges      Output array
 * @param num_ranges  In: capacity of ranges, or 0 for UAC2_MAX_VOLUME_RANGES
 *                    (the array must then hold that many). Out: entries written;
 *                    extra ranges beyond the capacity are dropped.
 */
esp_err_t uac2_host_device_get_volume_range(uac2_host_device_handle_t dev,
                                            uint8_t channel,
                                            uac2_volume_range_t *ranges,
                                            uint8_t *num_ranges);

/**
 * Set volume as a percentage (0-100). Maps linearly from min to max dB,
 * rounded to the nearest step of the device's advertised resolution.
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
 * Stops on the first per-channel error and returns it. A failed master
 * (channel 0) write does not stop the per-channel writes, since some devices
 * advertise a master control that STALLs; it is returned only if the device
 * has no per-channel volume controls.
 *
 * @param volume_db256  Volume in 1/256 dB units (applied to every channel)
 */
esp_err_t uac2_host_device_set_volume_all_channels(uac2_host_device_handle_t dev,
                                                   int16_t volume_db256);

/**
 * Set mute on all channels that support mute, with the same master fallback
 * as uac2_host_device_set_volume_all_channels(). Muting before start() and
 * unmuting once audio flows avoids start-up noise on some DACs.
 */
esp_err_t uac2_host_device_set_mute_all_channels(uac2_host_device_handle_t dev, bool mute);

#ifdef __cplusplus
}
#endif
