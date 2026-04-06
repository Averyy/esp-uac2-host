/*
 * SPDX-FileCopyrightText: 2026 Avery Levitt
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file uac2_host.c
 * @brief USB Audio Class 2.0 host driver implementation
 */

#include <string.h>
#include <stdatomic.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/ringbuf.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_check.h"
#include "usb/usb_host.h"
#include "usb/uac2_host.h"

static const char *TAG = "uac2-host-drv";

// ── UAC2 control request constants ─────────────────────────────────
// UAC2_REQUEST_CUR, UAC2_REQUEST_RANGE, UAC2_CS_SAM_FREQ_CONTROL,
// UAC2_CS_CLOCK_VALID_CONTROL are defined in uac2_desc.h

// Feature Unit control selectors (not in uac2_desc.h)
#define UAC2_FU_MUTE_CONTROL            0x01
#define UAC2_FU_VOLUME_CONTROL          0x02

// bmRequestType values
#define UAC2_CTRL_SET   (USB_BM_REQUEST_TYPE_DIR_OUT | USB_BM_REQUEST_TYPE_TYPE_CLASS | USB_BM_REQUEST_TYPE_RECIP_INTERFACE)
#define UAC2_CTRL_GET   (USB_BM_REQUEST_TYPE_DIR_IN  | USB_BM_REQUEST_TYPE_TYPE_CLASS | USB_BM_REQUEST_TYPE_RECIP_INTERFACE)

// ── Internal types ─────────────────────────────────────────────────

typedef enum {
    UAC2_STREAM_STATE_IDLE = 0,
    UAC2_STREAM_STATE_READY,
    UAC2_STREAM_STATE_ACTIVE,
} uac2_stream_state_t;

typedef struct {
    uac2_stream_dir_t dir;
    uac2_stream_state_t state;
    portMUX_TYPE state_lock;        // per-stream spinlock for state transitions

    // Interface info (from descriptors)
    uint8_t iface_num;
    uint8_t alt_setting;

    // Audio parameters
    uint32_t sample_rate;
    uint8_t  channels;
    uint8_t  bit_resolution;
    uint8_t  sub_slot_size;     // bytes per sample slot
    uint16_t packet_size;       // bytes per 1ms frame

    // Data endpoint
    uint8_t  ep_addr;
    uint16_t ep_mps;

    // Feedback endpoint (async playback only)
    uint8_t  fb_ep_addr;        // 0 if no feedback
    usb_transfer_t *fb_xfer;
    atomic_uint fb_value;       // latest feedback in 16.16 format (written by fb callback, read by TX submit)
    uint32_t fb_accumulator;    // fractional sample accumulator (only accessed from USB callback task)

    // Isochronous URBs
    usb_transfer_t *xfer[UAC2_NUM_ISOC_URBS];
    int xfer_count;
    atomic_int urbs_in_flight;      // decremented by callbacks, waited on by stream_stop
    atomic_int consecutive_errors;  // reset on success, stops re-submitting after max

    // Timing
    _Atomic int64_t first_frame_us; // esp_timer_get_time() when first URB submitted (0 = not yet)

    // Ring buffer
    RingbufHandle_t ringbuf;
    uint32_t ringbuf_size;
    uint32_t ringbuf_threshold;
    _Atomic bool tx_done_pending; // single-shot flag: true after TX_DONE fired, cleared by stream_write

    // Synchronization for safe stream_free: signaled by stream_write/stream_read
    // when they return from a blocking ringbuf call, so stream_free can wait.
    SemaphoreHandle_t user_task_done;
    _Atomic bool user_task_blocked;  // true while a task is blocked in stream_write/stream_read
} uac2_stream_t;

struct uac2_host_device {
    // USB handles
    usb_host_client_handle_t client;
    usb_device_handle_t usb_dev;

    // Device info
    uac2_device_info_t desc_info;
    uint8_t ac_iface_num;       // Audio Control interface number

    // Clock info (resolved from descriptors)
    uint8_t clock_source_id;    // Primary clock source entity ID

    // Feature unit (first one found, for volume/mute)
    uint8_t feature_unit_id;
    bool has_feature_unit;

    // Control transfer (shared, protected by mutex + completion semaphore)
    usb_transfer_t *ctrl_xfer;
    SemaphoreHandle_t ctrl_xfer_done;    // signaled on transfer completion
    SemaphoreHandle_t ctrl_mutex;        // serializes control transfer access
    _Atomic bool closing;                // set during device_close to reject new requests
    atomic_uint ctrl_xfer_gen;           // incremented on timeout to invalidate stale callbacks
    atomic_uint ctrl_xfer_submitted_gen; // gen at time of last submit (written under mutex, read by callback)

    // Event callback
    uac2_host_event_cb_t event_cb;
    void *event_cb_arg;

    // Disconnect tracking
    _Atomic bool disconnect_fired;       // ensures DISCONNECTED event fires exactly once

    // Streams
    uac2_stream_t *tx_stream;
    uac2_stream_t *rx_stream;
};

// ── Control transfer helpers ───────────────────────────────────────

static void ctrl_xfer_cb(usb_transfer_t *xfer)
{
    uac2_host_device_handle_t dev = (uac2_host_device_handle_t)xfer->context;
    // If a timeout advanced the generation counter, this callback is stale —
    // don't signal, so the next ctrl_request doesn't get a premature wakeup.
    if (atomic_load(&dev->ctrl_xfer_gen) != atomic_load(&dev->ctrl_xfer_submitted_gen)) {
        return;
    }
    xSemaphoreGive(dev->ctrl_xfer_done);
}

/**
 * Send a class-specific control request and wait for completion.
 * For SET: data_in should be NULL, data_out points to payload.
 * For GET: data_out should be NULL, response is in ctrl_xfer->data_buffer + 8.
 *
 * WARNING: On success, ctrl_mutex is intentionally left HELD so the caller
 * can safely read response data from the shared transfer buffer. The caller
 * MUST release ctrl_mutex (via xSemaphoreGive) after copying response data.
 * Use ctrl_request_no_data() for requests with no response to avoid this.
 */
static esp_err_t ctrl_request(uac2_host_device_handle_t dev,
                              uint8_t bm_request_type,
                              uint8_t b_request,
                              uint16_t w_value,
                              uint16_t w_index,
                              uint16_t w_length,
                              const uint8_t *data_out)
{
    if (!dev || !dev->ctrl_xfer || atomic_load(&dev->closing)) {
        return ESP_ERR_INVALID_ARG;
    }

    // Serialize access to the shared control transfer
    if (xSemaphoreTake(dev->ctrl_mutex, pdMS_TO_TICKS(UAC2_CTRL_XFER_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Control transfer mutex timeout");
        return ESP_ERR_TIMEOUT;
    }

    usb_transfer_t *xfer = dev->ctrl_xfer;

    // Fill setup packet (first 8 bytes of data buffer)
    usb_setup_packet_t *setup = (usb_setup_packet_t *)xfer->data_buffer;
    setup->bmRequestType = bm_request_type;
    setup->bRequest = b_request;
    setup->wValue = w_value;
    setup->wIndex = w_index;
    setup->wLength = w_length;

    // Bounds check: prevent buffer overflow on data copy
    if (w_length > UAC2_CTRL_XFER_MAX_SIZE) {
        ESP_LOGE(TAG, "Control data too large: %d > %d", w_length, UAC2_CTRL_XFER_MAX_SIZE);
        xSemaphoreGive(dev->ctrl_mutex);
        return ESP_ERR_INVALID_SIZE;
    }

    // Copy OUT data after setup packet
    if (data_out && w_length > 0) {
        memcpy(xfer->data_buffer + sizeof(usb_setup_packet_t), data_out, w_length);
    }

    xfer->num_bytes = sizeof(usb_setup_packet_t) + w_length;
    xfer->device_handle = dev->usb_dev;
    xfer->bEndpointAddress = 0;
    xfer->callback = ctrl_xfer_cb;
    xfer->context = dev;
    xfer->timeout_ms = UAC2_CTRL_XFER_TIMEOUT_MS;

    // Snapshot generation so the callback can detect staleness after timeout
    atomic_store(&dev->ctrl_xfer_submitted_gen, atomic_load(&dev->ctrl_xfer_gen));

    esp_err_t err = usb_host_transfer_submit_control(dev->client, xfer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Control submit failed: %s", esp_err_to_name(err));
        xSemaphoreGive(dev->ctrl_mutex);
        return err;
    }

    // Wait for completion
    if (xSemaphoreTake(dev->ctrl_xfer_done, pdMS_TO_TICKS(UAC2_CTRL_XFER_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Control transfer timeout");
        // Advance generation so late callback won't signal the semaphore
        atomic_fetch_add(&dev->ctrl_xfer_gen, 1);
        // Recover the control pipe (EP0) — without this, subsequent control transfers
        // may fail if the pipe is left in an error state after timeout.
        esp_err_t halt_err = usb_host_endpoint_halt(dev->usb_dev, 0);
        if (halt_err == ESP_OK) {
            usb_host_endpoint_flush(dev->usb_dev, 0);
            usb_host_endpoint_clear(dev->usb_dev, 0);
        }
        // Drain any semaphore signal that snuck in before gen increment
        xSemaphoreTake(dev->ctrl_xfer_done, 0);
        xSemaphoreGive(dev->ctrl_mutex);
        return ESP_ERR_TIMEOUT;
    }

    if (xfer->status != USB_TRANSFER_STATUS_COMPLETED) {
        ESP_LOGE(TAG, "Control transfer failed, status=%d", xfer->status);
        xSemaphoreGive(dev->ctrl_mutex);
        return ESP_FAIL;
    }

    // Debug hex dump of response data (enable with ESP_LOG_DEBUG level)
    int data_len = xfer->actual_num_bytes - (int)sizeof(usb_setup_packet_t);
    if (data_len > 0) {
        ESP_LOG_BUFFER_HEXDUMP(TAG, xfer->data_buffer + sizeof(usb_setup_packet_t),
                               data_len, ESP_LOG_DEBUG);
    }

    // Mutex stays held — caller must release after reading response data.
    // ctrl_set_cur releases immediately; ctrl_get_cur/range copy data then release.
    return ESP_OK;
}

/**
 * Send a control request that has no response data to read.
 * Auto-releases the mutex on success (unlike ctrl_request which holds it).
 */
static esp_err_t ctrl_request_no_data(uac2_host_device_handle_t dev,
                                       uint8_t bm_request_type,
                                       uint8_t b_request,
                                       uint16_t w_value,
                                       uint16_t w_index)
{
    esp_err_t err = ctrl_request(dev, bm_request_type, b_request,
                                 w_value, w_index, 0, NULL);
    if (err == ESP_OK) {
        xSemaphoreGive(dev->ctrl_mutex);
    }
    return err;
}

/**
 * Get the number of data bytes received in the last control GET response.
 * (Excludes the 8-byte setup packet.)
 */
static int ctrl_get_actual_len(uac2_host_device_handle_t dev)
{
    int total = dev->ctrl_xfer->actual_num_bytes;
    int data_len = total - (int)sizeof(usb_setup_packet_t);
    return data_len > 0 ? data_len : 0;
}

static esp_err_t ctrl_set_cur(uac2_host_device_handle_t dev,
                              uint8_t entity_id,
                              uint8_t control_selector,
                              uint8_t channel,
                              const uint8_t *data, uint16_t len)
{
    uint16_t w_value = (control_selector << 8) | channel;
    uint16_t w_index = (entity_id << 8) | dev->ac_iface_num;
    esp_err_t err = ctrl_request(dev, UAC2_CTRL_SET, UAC2_REQUEST_CUR,
                                  w_value, w_index, len, data);
    if (err == ESP_OK) {
        xSemaphoreGive(dev->ctrl_mutex);
    }
    return err;
}

static esp_err_t ctrl_get_cur(uac2_host_device_handle_t dev,
                              uint8_t entity_id,
                              uint8_t control_selector,
                              uint8_t channel,
                              uint8_t *data, uint16_t len)
{
    uint16_t w_value = (control_selector << 8) | channel;
    uint16_t w_index = (entity_id << 8) | dev->ac_iface_num;

    esp_err_t err = ctrl_request(dev, UAC2_CTRL_GET, UAC2_REQUEST_CUR,
                                 w_value, w_index, len, NULL);
    if (err == ESP_OK) {
        if (data) {
            int actual = ctrl_get_actual_len(dev);
            int copy_len = actual < len ? actual : len;
            if (copy_len > 0) {
                memcpy(data, dev->ctrl_xfer->data_buffer + sizeof(usb_setup_packet_t), copy_len);
            }
        }
        xSemaphoreGive(dev->ctrl_mutex);
    }
    return err;
}

static esp_err_t ctrl_get_range(uac2_host_device_handle_t dev,
                                uint8_t entity_id,
                                uint8_t control_selector,
                                uint8_t channel,
                                uint8_t *data, uint16_t len)
{
    uint16_t w_value = (control_selector << 8) | channel;
    uint16_t w_index = (entity_id << 8) | dev->ac_iface_num;

    esp_err_t err = ctrl_request(dev, UAC2_CTRL_GET, UAC2_REQUEST_RANGE,
                                 w_value, w_index, len, NULL);
    if (err == ESP_OK) {
        if (data) {
            int actual = ctrl_get_actual_len(dev);
            int copy_len = actual < len ? actual : len;
            if (copy_len > 0) {
                memcpy(data, dev->ctrl_xfer->data_buffer + sizeof(usb_setup_packet_t), copy_len);
            }
        }
        xSemaphoreGive(dev->ctrl_mutex);
    }
    return err;
}

// ── Isochronous transfer callbacks ─────────────────────────────────

static void stream_tx_xfer_submit(uac2_stream_t *stream,
                                  uac2_host_device_handle_t dev,
                                  usb_transfer_t *xfer);

static void stream_tx_xfer_done(usb_transfer_t *xfer)
{
    uac2_host_device_handle_t dev = (uac2_host_device_handle_t)xfer->context;
    uac2_stream_t *stream = dev->tx_stream;

    // Null check before critical section — stream_stop() may have NULLed the
    // pointer on the leak-timeout path while this callback was in-flight.
    if (!stream) return;

    portENTER_CRITICAL(&stream->state_lock);
    bool active = stream->state == UAC2_STREAM_STATE_ACTIVE;
    portEXIT_CRITICAL(&stream->state_lock);

    if (!active) {
        atomic_fetch_sub(&stream->urbs_in_flight, 1);
        return;
    }

    switch (xfer->status) {
    case USB_TRANSFER_STATUS_COMPLETED:
        atomic_store(&stream->consecutive_errors, 0);
        stream_tx_xfer_submit(stream, dev, xfer);
        break;

    case USB_TRANSFER_STATUS_NO_DEVICE:
    case USB_TRANSFER_STATUS_CANCELED:
        atomic_fetch_sub(&stream->urbs_in_flight, 1);
        if (xfer->status == USB_TRANSFER_STATUS_NO_DEVICE &&
            dev->event_cb && !atomic_exchange(&dev->disconnect_fired, true)) {
            dev->event_cb(dev, UAC2_HOST_EVENT_DISCONNECTED, dev->event_cb_arg);
        }
        return;

    default: {
        int errs = atomic_fetch_add(&stream->consecutive_errors, 1) + 1;
        ESP_LOGW(TAG, "TX transfer error, status=%d (consecutive: %d)", xfer->status, errs);
        if (dev->event_cb) {
            dev->event_cb(dev, UAC2_HOST_EVENT_TRANSFER_ERROR, dev->event_cb_arg);
        }
        if (errs >= UAC2_MAX_CONSECUTIVE_ERRORS) {
            ESP_LOGE(TAG, "TX: %d consecutive errors, stopping re-submission", errs);
            atomic_fetch_sub(&stream->urbs_in_flight, 1);
        } else {
            stream_tx_xfer_submit(stream, dev, xfer);
        }
        break;
    }
    }
}

static void stream_tx_xfer_submit(uac2_stream_t *stream,
                                  uac2_host_device_handle_t dev,
                                  usb_transfer_t *xfer)
{
    // State already verified under spinlock by the caller (stream_tx_xfer_done).
    // Re-check under spinlock to close the TOCTOU window with stream_stop.
    portENTER_CRITICAL(&stream->state_lock);
    bool active = stream->state == UAC2_STREAM_STATE_ACTIVE;
    portEXIT_CRITICAL(&stream->state_lock);
    if (!active) {
        atomic_fetch_sub(&stream->urbs_in_flight, 1);
        return;
    }

    size_t item_size = 0;

    // Feedback-based adaptive packet sizing: the device's feedback endpoint reports
    // its ideal samples-per-frame in 16.16 fixed-point. We accumulate the fractional
    // part and send one extra sample when it overflows. This prevents the device's
    // buffer from slowly under/overrunning during sustained playback.
    uint16_t pkt_size;
    uint32_t fb = atomic_load(&stream->fb_value);
    if (fb > 0) {
        uint16_t nominal_samples = (uint16_t)(fb >> 16);
        uint16_t fraction = (uint16_t)(fb & 0xFFFF);
        // Reject bogus feedback: zero samples or wildly out of range
        if (nominal_samples == 0 || nominal_samples > 1000) {
            pkt_size = stream->packet_size;
            goto send_packet;
        }

        stream->fb_accumulator += fraction;
        uint16_t extra = (uint16_t)(stream->fb_accumulator >> 16);
        stream->fb_accumulator &= 0xFFFF;

        uint16_t samples_this_frame = nominal_samples + extra;
        uint32_t raw_pkt_size = (uint32_t)samples_this_frame * stream->channels * stream->sub_slot_size;
        pkt_size = (raw_pkt_size > stream->ep_mps) ? stream->ep_mps : (uint16_t)raw_pkt_size;
    } else {
        pkt_size = stream->packet_size;
    }
send_packet:

    // Try to read one packet worth of data from ring buffer
    void *data = xRingbufferReceiveUpTo(stream->ringbuf, &item_size,
                                        0, pkt_size);

    if (data && item_size > 0) {
        memcpy(xfer->data_buffer, data, item_size);
        vRingbufferReturnItem(stream->ringbuf, data);

        // Pad remainder with silence if short
        if (item_size < pkt_size) {
            memset(xfer->data_buffer + item_size, 0, pkt_size - item_size);
        }

        // Notify when ringbuf drops below threshold (single-shot to avoid spam)
        size_t rb_used = stream->ringbuf_size - xRingbufferGetCurFreeSize(stream->ringbuf);
        if (rb_used < stream->ringbuf_threshold && dev->event_cb && !atomic_load(&stream->tx_done_pending)) {
            atomic_store(&stream->tx_done_pending, true);
            dev->event_cb(dev, UAC2_HOST_EVENT_TX_DONE, dev->event_cb_arg);
        }
    } else {
        // No data: send silence, notify once
        memset(xfer->data_buffer, 0, pkt_size);
        if (dev->event_cb && !atomic_load(&stream->tx_done_pending)) {
            atomic_store(&stream->tx_done_pending, true);
            dev->event_cb(dev, UAC2_HOST_EVENT_TX_DONE, dev->event_cb_arg);
        }
    }

    xfer->num_bytes = pkt_size;
    xfer->isoc_packet_desc[0].num_bytes = pkt_size;

    esp_err_t err = usb_host_transfer_submit(xfer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TX submit failed: %s", esp_err_to_name(err));
        atomic_fetch_sub(&stream->urbs_in_flight, 1);
    }
}

static void stream_rx_xfer_done(usb_transfer_t *xfer)
{
    uac2_host_device_handle_t dev = (uac2_host_device_handle_t)xfer->context;
    uac2_stream_t *stream = dev->rx_stream;

    if (!stream) return;

    portENTER_CRITICAL(&stream->state_lock);
    bool active = stream->state == UAC2_STREAM_STATE_ACTIVE;
    portEXIT_CRITICAL(&stream->state_lock);

    if (!active) {
        atomic_fetch_sub(&stream->urbs_in_flight, 1);
        return;
    }

    switch (xfer->status) {
    case USB_TRANSFER_STATUS_COMPLETED:
        atomic_store(&stream->consecutive_errors, 0);
        for (int i = 0; i < xfer->num_isoc_packets; i++) {
            usb_isoc_packet_desc_t *pkt = &xfer->isoc_packet_desc[i];
            if (pkt->status == USB_TRANSFER_STATUS_COMPLETED && pkt->actual_num_bytes > 0) {
                uint8_t *pkt_data = xfer->data_buffer + (i * stream->ep_mps);
                BaseType_t ok = xRingbufferSend(stream->ringbuf, pkt_data,
                                                pkt->actual_num_bytes, 0);
                if (ok != pdTRUE) {
                    ESP_LOGW(TAG, "RX ringbuf overflow, dropped %d bytes",
                             pkt->actual_num_bytes);
                }
            }
        }

        // Notify when ringbuf exceeds threshold
        {
            size_t rb_used = stream->ringbuf_size - xRingbufferGetCurFreeSize(stream->ringbuf);
            if (rb_used >= stream->ringbuf_threshold && dev->event_cb) {
                dev->event_cb(dev, UAC2_HOST_EVENT_RX_DONE, dev->event_cb_arg);
            }
        }

        // Resubmit
        xfer->num_bytes = stream->ep_mps * xfer->num_isoc_packets;
        for (int i = 0; i < xfer->num_isoc_packets; i++) {
            xfer->isoc_packet_desc[i].num_bytes = stream->ep_mps;
        }
        {
            esp_err_t sub_err = usb_host_transfer_submit(xfer);
            if (sub_err != ESP_OK) {
                ESP_LOGW(TAG, "RX resubmit failed: %s", esp_err_to_name(sub_err));
                atomic_fetch_sub(&stream->urbs_in_flight, 1);
            }
        }
        break;

    case USB_TRANSFER_STATUS_NO_DEVICE:
    case USB_TRANSFER_STATUS_CANCELED:
        atomic_fetch_sub(&stream->urbs_in_flight, 1);
        if (xfer->status == USB_TRANSFER_STATUS_NO_DEVICE &&
            dev->event_cb && !atomic_exchange(&dev->disconnect_fired, true)) {
            dev->event_cb(dev, UAC2_HOST_EVENT_DISCONNECTED, dev->event_cb_arg);
        }
        return;

    default: {
        int errs = atomic_fetch_add(&stream->consecutive_errors, 1) + 1;
        ESP_LOGW(TAG, "RX transfer error, status=%d (consecutive: %d)", xfer->status, errs);
        if (errs >= UAC2_MAX_CONSECUTIVE_ERRORS) {
            ESP_LOGE(TAG, "RX: %d consecutive errors, stopping re-submission", errs);
            atomic_fetch_sub(&stream->urbs_in_flight, 1);
            if (dev->event_cb) {
                dev->event_cb(dev, UAC2_HOST_EVENT_TRANSFER_ERROR, dev->event_cb_arg);
            }
        } else {
            esp_err_t sub_err = usb_host_transfer_submit(xfer);
            if (sub_err != ESP_OK) {
                ESP_LOGW(TAG, "RX resubmit failed: %s", esp_err_to_name(sub_err));
                atomic_fetch_sub(&stream->urbs_in_flight, 1);
            }
        }
        break;
    }
    }
}

static void feedback_xfer_done(usb_transfer_t *xfer)
{
    uac2_host_device_handle_t dev = (uac2_host_device_handle_t)xfer->context;
    uac2_stream_t *stream = dev->tx_stream;

    if (!stream) return;

    portENTER_CRITICAL(&stream->state_lock);
    bool active = stream->state == UAC2_STREAM_STATE_ACTIVE;
    portEXIT_CRITICAL(&stream->state_lock);

    if (!active) {
        atomic_fetch_sub(&stream->urbs_in_flight, 1);
        return;
    }

    // Real miniDSP 2x4 HD confirmed: 4-byte 16.16 feedback at Full Speed.
    // 48kHz value: 0x00300000 (48.0000). Both 3-byte (10.14) and 4-byte (16.16) handled.
    if (xfer->status != USB_TRANSFER_STATUS_COMPLETED) {
        if (xfer->status == USB_TRANSFER_STATUS_NO_DEVICE ||
            xfer->status == USB_TRANSFER_STATUS_CANCELED) {
            atomic_fetch_sub(&stream->urbs_in_flight, 1);
            if (xfer->status == USB_TRANSFER_STATUS_NO_DEVICE &&
                dev->event_cb && !atomic_exchange(&dev->disconnect_fired, true)) {
                dev->event_cb(dev, UAC2_HOST_EVENT_DISCONNECTED, dev->event_cb_arg);
            }
            return;
        }
        int errs = atomic_fetch_add(&stream->consecutive_errors, 1) + 1;
        ESP_LOGD(TAG, "Feedback transfer error, status=%d (consecutive: %d)", xfer->status, errs);
        if (errs >= UAC2_MAX_CONSECUTIVE_ERRORS) {
            ESP_LOGE(TAG, "Feedback: %d consecutive errors, stopping re-submission", errs);
            atomic_fetch_sub(&stream->urbs_in_flight, 1);
            return;
        }
    }

    if (xfer->status == USB_TRANSFER_STATUS_COMPLETED) {
        atomic_store(&stream->consecutive_errors, 0);
        int actual = xfer->isoc_packet_desc[0].actual_num_bytes;

        if (actual == 4) {
            // 16.16 format — deserialize little-endian explicitly
            uint32_t raw = xfer->data_buffer[0]
                         | ((uint32_t)xfer->data_buffer[1] << 8)
                         | ((uint32_t)xfer->data_buffer[2] << 16)
                         | ((uint32_t)xfer->data_buffer[3] << 24);
            atomic_store(&stream->fb_value, raw);
        } else if (actual == 3) {
            // 10.14 format (standard FS feedback), convert to 16.16
            uint32_t raw = (uint32_t)xfer->data_buffer[0]
                         | ((uint32_t)xfer->data_buffer[1] << 8)
                         | ((uint32_t)xfer->data_buffer[2] << 16);
            // 10.14 -> 16.16: shift left by 2
            atomic_store(&stream->fb_value, raw << 2);
        }

        {
            uint32_t fb_log = atomic_load(&stream->fb_value);
            ESP_LOGD(TAG, "Feedback: %" PRIu32 ".%04" PRIu32 " Hz",
                     (uint32_t)(fb_log >> 16),
                     (uint32_t)((fb_log & 0xFFFF) * 10000 / 65536));
        }
    }

    // Resubmit feedback URB — recheck state under spinlock to prevent race with stream_stop
    portENTER_CRITICAL(&stream->state_lock);
    bool still_active = stream->state == UAC2_STREAM_STATE_ACTIVE;
    portEXIT_CRITICAL(&stream->state_lock);
    if (still_active) {
        xfer->isoc_packet_desc[0].num_bytes = xfer->data_buffer_size;
        xfer->num_bytes = xfer->data_buffer_size;
        esp_err_t sub_err = usb_host_transfer_submit(xfer);
        if (sub_err != ESP_OK) {
            ESP_LOGW(TAG, "Feedback resubmit failed: %s", esp_err_to_name(sub_err));
            atomic_fetch_sub(&stream->urbs_in_flight, 1);
        }
    } else {
        atomic_fetch_sub(&stream->urbs_in_flight, 1);
    }
}

// ── Stream management ──────────────────────────────────────────────

/**
 * Find the best matching AS interface for the requested config.
 */
static const uac2_as_iface_t *find_matching_as_iface(
    const uac2_device_info_t *info,
    uac2_stream_dir_t dir,
    const uac2_stream_config_t *config)
{
    const uac2_as_iface_t *best = NULL;

    for (int i = 0; i < info->num_as_ifaces; i++) {
        const uac2_as_iface_t *as = &info->as_ifaces[i];

        // Match direction: OUT endpoint = playback (TX), IN = capture (RX)
        bool is_out = (as->ep_addr & 0x80) == 0;
        if (dir == UAC2_STREAM_TX && !is_out) continue;
        if (dir == UAC2_STREAM_RX && is_out) continue;

        // Match format
        if (config->bit_resolution && as->bit_resolution != config->bit_resolution) continue;
        if (config->channels && as->nr_channels != config->channels) continue;

        // Prefer higher bit depth
        if (!best || as->bit_resolution > best->bit_resolution) {
            best = as;
        }
    }

    return best;
}

static uint16_t calc_packet_size(uint32_t sample_rate, uint8_t channels,
                                 uint8_t sub_slot_size)
{
    // Bytes per 1ms frame at Full Speed
    // = (sample_rate / 1000) * channels * bytes_per_sample
    // Add 1 sample worth of space for rounding (async jitter)
    uint32_t samples_per_frame = (sample_rate + 999) / 1000;
    uint32_t result = samples_per_frame * channels * sub_slot_size;
    if (result > UINT16_MAX) {
        ESP_LOGE(TAG, "Packet size overflow: %" PRIu32 " (rate=%" PRIu32 " ch=%d ss=%d)",
                 result, sample_rate, channels, sub_slot_size);
        return 0;
    }
    return (uint16_t)result;
}

static esp_err_t stream_alloc(uac2_stream_t **out_stream,
                              uac2_stream_dir_t dir,
                              const uac2_as_iface_t *as,
                              const uac2_stream_config_t *config)
{
    uac2_stream_t *stream = heap_caps_calloc(1, sizeof(uac2_stream_t), MALLOC_CAP_DEFAULT);
    if (!stream) return ESP_ERR_NO_MEM;

    stream->dir = dir;
    stream->state = UAC2_STREAM_STATE_IDLE;
    portMUX_INITIALIZE(&stream->state_lock);
    stream->iface_num = as->interface_num;
    stream->alt_setting = as->alt_setting;
    stream->sample_rate = config->sample_rate;
    stream->channels = as->nr_channels;
    stream->bit_resolution = as->bit_resolution;
    stream->sub_slot_size = as->sub_slot_size;
    stream->ep_addr = as->ep_addr;
    stream->ep_mps = as->ep_max_packet_size;
    stream->fb_ep_addr = as->fb_ep_addr;
    atomic_init(&stream->fb_value, 0);
    stream->fb_accumulator = 0;
    atomic_init(&stream->urbs_in_flight, 0);
    atomic_init(&stream->consecutive_errors, 0);
    atomic_init(&stream->user_task_blocked, false);
    stream->first_frame_us = 0;
    stream->user_task_done = xSemaphoreCreateBinary();
    if (!stream->user_task_done) {
        heap_caps_free(stream);
        return ESP_ERR_NO_MEM;
    }

    stream->packet_size = calc_packet_size(config->sample_rate,
                                           as->nr_channels,
                                           as->sub_slot_size);

    // Reject if packet size is invalid (0 = overflow in calc) or exceeds endpoint MPS
    if (stream->packet_size == 0 || stream->packet_size > stream->ep_mps) {
        ESP_LOGE(TAG, "Packet size %d invalid (MPS %d) — config not supported",
                 stream->packet_size, stream->ep_mps);
        vSemaphoreDelete(stream->user_task_done);
        heap_caps_free(stream);
        return ESP_ERR_INVALID_SIZE;
    }

    // Ring buffer
    uint32_t rb_size = config->ringbuf_size;
    if (rb_size == 0) {
        // Default: ~100ms of audio
        rb_size = stream->packet_size * 100;
    }
    stream->ringbuf_size = rb_size;
    stream->ringbuf_threshold = config->ringbuf_threshold;
    if (stream->ringbuf_threshold == 0) {
        stream->ringbuf_threshold = rb_size / 2;
    }

    stream->ringbuf = xRingbufferCreate(rb_size, RINGBUF_TYPE_BYTEBUF);
    if (!stream->ringbuf) {
        vSemaphoreDelete(stream->user_task_done);
        heap_caps_free(stream);
        return ESP_ERR_NO_MEM;
    }

    // Allocate isochronous URBs
    int num_pkts = UAC2_NUM_PACKETS_PER_URB;
    size_t xfer_size = stream->ep_mps * num_pkts;

    for (int i = 0; i < UAC2_NUM_ISOC_URBS; i++) {
        esp_err_t err = usb_host_transfer_alloc(xfer_size, num_pkts, &stream->xfer[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to allocate URB %d: %s", i, esp_err_to_name(err));
            // Cleanup already-allocated
            for (int j = 0; j < i; j++) {
                usb_host_transfer_free(stream->xfer[j]);
            }
            vRingbufferDelete(stream->ringbuf);
            vSemaphoreDelete(stream->user_task_done);
            heap_caps_free(stream);
            return err;
        }
        stream->xfer_count = i + 1;
    }

    // Allocate feedback URB if needed
    if (stream->fb_ep_addr != 0 && dir == UAC2_STREAM_TX) {
        esp_err_t err = usb_host_transfer_alloc(4, 1, &stream->fb_xfer);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to allocate feedback URB: %s", esp_err_to_name(err));
            stream->fb_ep_addr = 0;  // Continue without feedback
        }
    }

    *out_stream = stream;
    return ESP_OK;
}

static void stream_free(uac2_stream_t *stream)
{
    if (!stream) return;

    for (int i = 0; i < stream->xfer_count; i++) {
        if (stream->xfer[i]) {
            usb_host_transfer_free(stream->xfer[i]);
        }
    }
    if (stream->fb_xfer) {
        usb_host_transfer_free(stream->fb_xfer);
    }
    if (stream->ringbuf) {
        // Unblock any task waiting on the ringbuffer before deleting it.
        if (stream->dir == UAC2_STREAM_TX) {
            // Unblock xRingbufferSend by draining buffer to free space
            size_t item_size;
            void *item;
            while ((item = xRingbufferReceiveUpTo(stream->ringbuf, &item_size, 0,
                                                   stream->ringbuf_size)) != NULL) {
                vRingbufferReturnItem(stream->ringbuf, item);
            }
        } else {
            // Unblock xRingbufferReceiveUpTo by sending a dummy byte
            uint8_t dummy = 0;
            xRingbufferSend(stream->ringbuf, &dummy, 1, 0);
        }
        // Wait for stream_write/stream_read to return if a task was blocked
        if (atomic_load(&stream->user_task_blocked)) {
            xSemaphoreTake(stream->user_task_done, pdMS_TO_TICKS(200));
        }
        vRingbufferDelete(stream->ringbuf);
    }
    if (stream->user_task_done) {
        vSemaphoreDelete(stream->user_task_done);
    }
    heap_caps_free(stream);
}

static esp_err_t stream_submit_urbs(uac2_stream_t *stream,
                                    uac2_host_device_handle_t dev)
{
    int num_pkts = UAC2_NUM_PACKETS_PER_URB;

    // Set state to ACTIVE before submitting any URBs — callbacks check this
    // and will drop URBs if they see READY instead of ACTIVE.
    portENTER_CRITICAL(&stream->state_lock);
    stream->state = UAC2_STREAM_STATE_ACTIVE;
    portEXIT_CRITICAL(&stream->state_lock);

    for (int i = 0; i < stream->xfer_count; i++) {
        usb_transfer_t *xfer = stream->xfer[i];
        xfer->device_handle = dev->usb_dev;
        xfer->bEndpointAddress = stream->ep_addr;
        xfer->context = dev;
        xfer->timeout_ms = 0;

        if (stream->dir == UAC2_STREAM_TX) {
            xfer->callback = stream_tx_xfer_done;
            // Start with silence
            memset(xfer->data_buffer, 0, stream->packet_size * num_pkts);
            xfer->num_bytes = stream->packet_size * num_pkts;
            for (int j = 0; j < num_pkts; j++) {
                xfer->isoc_packet_desc[j].num_bytes = stream->packet_size;
            }
        } else {
            xfer->callback = stream_rx_xfer_done;
            xfer->num_bytes = stream->ep_mps * num_pkts;
            for (int j = 0; j < num_pkts; j++) {
                xfer->isoc_packet_desc[j].num_bytes = stream->ep_mps;
            }
        }

        atomic_fetch_add(&stream->urbs_in_flight, 1);
        esp_err_t err = usb_host_transfer_submit(xfer);
        if (err != ESP_OK) {
            atomic_fetch_sub(&stream->urbs_in_flight, 1);
            ESP_LOGE(TAG, "Failed to submit URB %d: %s", i, esp_err_to_name(err));
            return err;
        }
        // Capture timestamp when first data URB submits
        if (i == 0 && stream->first_frame_us == 0) {
            stream->first_frame_us = esp_timer_get_time();
        }
    }

    // Submit feedback URB
    if (stream->fb_xfer && stream->fb_ep_addr != 0) {
        usb_transfer_t *fb = stream->fb_xfer;
        fb->device_handle = dev->usb_dev;
        fb->bEndpointAddress = stream->fb_ep_addr;
        fb->callback = feedback_xfer_done;
        fb->context = dev;
        fb->timeout_ms = 0;
        fb->num_bytes = 4;
        fb->isoc_packet_desc[0].num_bytes = 4;

        atomic_fetch_add(&stream->urbs_in_flight, 1);
        esp_err_t err = usb_host_transfer_submit(fb);
        if (err != ESP_OK) {
            atomic_fetch_sub(&stream->urbs_in_flight, 1);
            ESP_LOGW(TAG, "Feedback URB submit failed: %s", esp_err_to_name(err));
            // Non-fatal: continue without feedback
        }
    }

    return ESP_OK;
}

// ── Resolve clock source ID ────────────────────────────────────────

/**
 * Walk the clock topology to find the actual clock source entity.
 * If a clock selector is present, follow it to the first source.
 *
 * TODO(hardware): This takes the first clock source unconditionally. For devices
 * with multiple clock sources or selectors, we should trace the clock reference
 * from the terminal linked to the active AS interface back through selectors to
 * the actual source. Works for miniDSP (single source ID=41) but will hit the
 * wrong entity on multi-clock devices.
 */
static uint8_t resolve_clock_source(const uac2_device_info_t *info)
{
    // If we have clock sources, prefer the first one
    if (info->num_clock_sources > 0) {
        return info->clock_sources[0].clock_id;
    }

    // If we have clock selectors, use the first source in the first selector
    if (info->num_clock_selectors > 0 && info->clock_selectors[0].nr_pins > 0) {
        return info->clock_selectors[0].source_ids[0];
    }

    ESP_LOGW(TAG, "No clock source found in descriptors");
    return 0;
}

/**
 * Convert a USB string descriptor (UTF-16LE) to a C string (ASCII).
 * bLength is trusted from ESP-IDF's enumeration (it fetches exactly bLength bytes).
 * out_size clamp prevents overread even if bLength is larger than the backing buffer.
 */
static void usb_string_to_ascii(const usb_str_desc_t *str_desc, char *out, size_t out_size)
{
    if (!str_desc || !out || out_size == 0) return;
    out[0] = '\0';
    if (str_desc->bLength < 2) return;
    // bLength includes 2-byte header, each char is 2 bytes (UTF-16LE)
    int num_chars = (str_desc->bLength - 2) / 2;
    if (num_chars <= 0) return;
    if ((size_t)num_chars >= out_size) num_chars = out_size - 1;
    for (int i = 0; i < num_chars; i++) {
        uint16_t wchar = str_desc->wData[i];
        out[i] = (wchar < 128) ? (char)wchar : '?';
    }
    out[num_chars] = '\0';
}

// ── Public API: Device management ──────────────────────────────────

esp_err_t uac2_host_device_open(usb_host_client_handle_t client,
                                usb_device_handle_t usb_dev,
                                uac2_host_event_cb_t cb, void *cb_arg,
                                uac2_host_device_handle_t *out_dev)
{
    ESP_RETURN_ON_FALSE(client && usb_dev && out_dev, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    // Get config descriptor
    const usb_config_desc_t *config_desc;
    esp_err_t err = usb_host_get_active_config_descriptor(usb_dev, &config_desc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get config descriptor: %s", esp_err_to_name(err));
        return err;
    }

    // Get VID/PID and device info (speed, string descriptors) before allocating
    const usb_device_desc_t *dev_desc;
    err = usb_host_get_device_descriptor(usb_dev, &dev_desc);
    if (err != ESP_OK) {
        return err;
    }

    usb_device_info_t usb_info;
    err = usb_host_device_info(usb_dev, &usb_info);
    if (err != ESP_OK) {
        return err;
    }

    // Reject low-speed devices — isochronous transfers are not supported
    if (usb_info.speed == USB_SPEED_LOW) {
        ESP_LOGE(TAG, "Low-speed devices do not support isochronous transfers");
        return ESP_ERR_NOT_SUPPORTED;
    }

    // Allocate device early so we can parse directly into dev->desc_info
    // (avoids ~400-byte uac2_device_info_t on stack)
    uac2_host_device_handle_t dev = heap_caps_calloc(1, sizeof(struct uac2_host_device), MALLOC_CAP_DEFAULT);
    if (!dev) return ESP_ERR_NO_MEM;

    // Parse UAC2 descriptors directly into the device struct
    bool is_uac2 = uac2_parse_config_descriptor(
        (const uint8_t *)config_desc, config_desc->wTotalLength, &dev->desc_info);

    if (!is_uac2) {
        ESP_LOGW(TAG, "Not a UAC2 device");
        heap_caps_free(dev);
        return ESP_ERR_NOT_SUPPORTED;
    }

    // Populate device identification
    dev->desc_info.vid = dev_desc->idVendor;
    dev->desc_info.pid = dev_desc->idProduct;
    usb_string_to_ascii(usb_info.str_desc_manufacturer, dev->desc_info.manufacturer,
                         sizeof(dev->desc_info.manufacturer));
    usb_string_to_ascii(usb_info.str_desc_product, dev->desc_info.product,
                         sizeof(dev->desc_info.product));
    usb_string_to_ascii(usb_info.str_desc_serial_num, dev->desc_info.serial,
                         sizeof(dev->desc_info.serial));

    dev->client = client;
    dev->usb_dev = usb_dev;
    dev->event_cb = cb;
    dev->event_cb_arg = cb_arg;
    atomic_init(&dev->closing, false);
    atomic_init(&dev->disconnect_fired, false);
    atomic_init(&dev->ctrl_xfer_gen, 0);

    // AC interface number was extracted by the descriptor parser
    dev->ac_iface_num = dev->desc_info.ac_iface_num;

    // Resolve clock source
    dev->clock_source_id = resolve_clock_source(&dev->desc_info);

    // Resolve feature unit (use first one for volume/mute)
    if (dev->desc_info.num_feature_units > 0) {
        dev->has_feature_unit = true;
        dev->feature_unit_id = dev->desc_info.feature_units[0].unit_id;
    }

    // Allocate control transfer
    err = usb_host_transfer_alloc(sizeof(usb_setup_packet_t) + UAC2_CTRL_XFER_MAX_SIZE,
                                  0, &dev->ctrl_xfer);
    if (err != ESP_OK) {
        heap_caps_free(dev);
        return err;
    }

    dev->ctrl_xfer_done = xSemaphoreCreateBinary();
    if (!dev->ctrl_xfer_done) {
        usb_host_transfer_free(dev->ctrl_xfer);
        heap_caps_free(dev);
        return ESP_ERR_NO_MEM;
    }

    dev->ctrl_mutex = xSemaphoreCreateMutex();
    if (!dev->ctrl_mutex) {
        vSemaphoreDelete(dev->ctrl_xfer_done);
        usb_host_transfer_free(dev->ctrl_xfer);
        heap_caps_free(dev);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "UAC2 host driver v%d.%d.%d",
             UAC2_HOST_VER_MAJOR, UAC2_HOST_VER_MINOR, UAC2_HOST_VER_PATCH);
    ESP_LOGI(TAG, "UAC2 device opened: VID=0x%04X PID=0x%04X \"%s\"",
             dev->desc_info.vid, dev->desc_info.pid,
             dev->desc_info.product[0] ? dev->desc_info.product : "Unknown");
    ESP_LOGI(TAG, "  AC iface=%d, clock_source=%d, feature_unit=%d",
             dev->ac_iface_num, dev->clock_source_id,
             dev->has_feature_unit ? dev->feature_unit_id : 0);
    ESP_LOGI(TAG, "  %d AS interfaces, %d clock sources, %d terminals",
             dev->desc_info.num_as_ifaces, dev->desc_info.num_clock_sources,
             dev->desc_info.num_terminals);

    *out_dev = dev;
    return ESP_OK;
}

esp_err_t uac2_host_device_close(uac2_host_device_handle_t dev)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "Invalid device handle");

    // Stop any active streams first — stream_stop sends SET_INTERFACE(alt=0) via
    // ctrl_request, which checks the closing flag. Must happen before closing=true.
    if (dev->tx_stream) {
        uac2_host_stream_stop(dev, UAC2_STREAM_TX);
    }
    if (dev->rx_stream) {
        uac2_host_stream_stop(dev, UAC2_STREAM_RX);
    }

    // Now reject any new external control requests
    atomic_store(&dev->closing, true);

    // Wait for any in-progress control request to finish
    if (dev->ctrl_mutex) {
        xSemaphoreTake(dev->ctrl_mutex, pdMS_TO_TICKS(UAC2_CTRL_XFER_TIMEOUT_MS));
        xSemaphoreGive(dev->ctrl_mutex);
    }

    // Free control transfer resources
    if (dev->ctrl_xfer) {
        usb_host_transfer_free(dev->ctrl_xfer);
    }
    if (dev->ctrl_xfer_done) {
        vSemaphoreDelete(dev->ctrl_xfer_done);
    }
    if (dev->ctrl_mutex) {
        vSemaphoreDelete(dev->ctrl_mutex);
    }

    ESP_LOGI(TAG, "UAC2 device closed");
    heap_caps_free(dev);
    return ESP_OK;
}

esp_err_t uac2_host_device_get_info(uac2_host_device_handle_t dev,
                                    uac2_device_info_t *info)
{
    ESP_RETURN_ON_FALSE(dev && info, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    *info = dev->desc_info;
    return ESP_OK;
}

// ── Public API: Clock control ──────────────────────────────────────

esp_err_t uac2_host_get_sample_rate(uac2_host_device_handle_t dev,
                                    uint32_t *sample_rate)
{
    ESP_RETURN_ON_FALSE(dev && sample_rate, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    ESP_RETURN_ON_FALSE(dev->clock_source_id != 0, ESP_ERR_NOT_SUPPORTED, TAG, "No clock source");

    uint8_t data[4] = {0};
    esp_err_t err = ctrl_get_cur(dev, dev->clock_source_id,
                                 UAC2_CS_SAM_FREQ_CONTROL, 0, data, 4);
    if (err != ESP_OK) return err;

    *sample_rate = (uint32_t)data[0] | ((uint32_t)data[1] << 8)
                 | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
    ESP_LOGI(TAG, "Current sample rate: %" PRIu32 " Hz", *sample_rate);
    return ESP_OK;
}

esp_err_t uac2_host_set_sample_rate(uac2_host_device_handle_t dev,
                                    uint32_t sample_rate)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "Invalid device handle");
    ESP_RETURN_ON_FALSE(dev->clock_source_id != 0, ESP_ERR_NOT_SUPPORTED, TAG, "No clock source");

    uint8_t data[4] = {
        (uint8_t)(sample_rate & 0xFF),
        (uint8_t)((sample_rate >> 8) & 0xFF),
        (uint8_t)((sample_rate >> 16) & 0xFF),
        (uint8_t)((sample_rate >> 24) & 0xFF),
    };

    esp_err_t err = ctrl_set_cur(dev, dev->clock_source_id,
                                 UAC2_CS_SAM_FREQ_CONTROL, 0, data, 4);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Set sample rate: %" PRIu32 " Hz", sample_rate);
    }
    return err;
}

esp_err_t uac2_host_get_sample_rate_range(uac2_host_device_handle_t dev,
                                          uac2_sample_rate_range_t *ranges,
                                          uint8_t *num_ranges)
{
    ESP_RETURN_ON_FALSE(dev && ranges && num_ranges, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    ESP_RETURN_ON_FALSE(dev->clock_source_id != 0, ESP_ERR_NOT_SUPPORTED, TAG, "No clock source");

    // Zero-initialize to prevent reading uninitialized data if device returns
    // fewer bytes than claimed by the count field in the response header.
    uint8_t buf[2 + UAC2_MAX_SAMPLE_RATE_RANGES * 12];
    memset(buf, 0, sizeof(buf));
    esp_err_t err = ctrl_get_range(dev, dev->clock_source_id,
                                   UAC2_CS_SAM_FREQ_CONTROL, 0,
                                   buf, sizeof(buf));
    if (err != ESP_OK) return err;

    uint16_t count = buf[0] | (buf[1] << 8);
    if (count > UAC2_MAX_SAMPLE_RATE_RANGES) {
        count = UAC2_MAX_SAMPLE_RATE_RANGES;
    }

    for (int i = 0; i < count; i++) {
        const uint8_t *p = buf + 2 + (i * 12);
        ranges[i].min = (uint32_t)p[0] | ((uint32_t)p[1] << 8)
                      | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
        ranges[i].max = (uint32_t)p[4] | ((uint32_t)p[5] << 8)
                      | ((uint32_t)p[6] << 16) | ((uint32_t)p[7] << 24);
        ranges[i].res = (uint32_t)p[8] | ((uint32_t)p[9] << 8)
                      | ((uint32_t)p[10] << 16) | ((uint32_t)p[11] << 24);

        ESP_LOGI(TAG, "Sample rate range %d: %" PRIu32 " - %" PRIu32 " Hz (res %" PRIu32 ")",
                 i, ranges[i].min, ranges[i].max, ranges[i].res);
    }

    *num_ranges = (uint8_t)count;
    return ESP_OK;
}

esp_err_t uac2_host_get_clock_valid(uac2_host_device_handle_t dev,
                                    bool *valid)
{
    ESP_RETURN_ON_FALSE(dev && valid, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    ESP_RETURN_ON_FALSE(dev->clock_source_id != 0, ESP_ERR_NOT_SUPPORTED, TAG, "No clock source");

    uint8_t data = 0;
    esp_err_t err = ctrl_get_cur(dev, dev->clock_source_id,
                                 UAC2_CS_CLOCK_VALID_CONTROL, 0, &data, 1);
    if (err != ESP_OK) return err;

    *valid = (data & 0x01) != 0;
    ESP_LOGI(TAG, "Clock valid: %s", *valid ? "yes" : "no");
    return ESP_OK;
}

// ── Public API: Streaming ──────────────────────────────────────────

esp_err_t uac2_host_stream_start(uac2_host_device_handle_t dev,
                                 uac2_stream_dir_t dir,
                                 const uac2_stream_config_t *config)
{
    ESP_RETURN_ON_FALSE(dev && config, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    // Check not already streaming in this direction
    uac2_stream_t **stream_ptr = (dir == UAC2_STREAM_TX)
                                 ? &dev->tx_stream : &dev->rx_stream;
    if (*stream_ptr) {
        ESP_LOGE(TAG, "Stream already active for dir=%d", dir);
        return ESP_ERR_INVALID_STATE;
    }

    // ESP32-S3 FIFO limitation: with PERIODIC_OUT bias, RX FIFO is only 128 bytes.
    // Audio capture packets (~294 bytes) won't fit. Reject RX if TX is active.
    if (dir == UAC2_STREAM_RX && dev->tx_stream) {
        ESP_LOGE(TAG, "Cannot start RX while TX active — ESP32-S3 FIFO too small for duplex");
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (dir == UAC2_STREAM_TX && dev->rx_stream) {
        ESP_LOGE(TAG, "Cannot start TX while RX active — ESP32-S3 FIFO too small for duplex");
        return ESP_ERR_NOT_SUPPORTED;
    }

    // Find matching AS interface
    const uac2_as_iface_t *as = find_matching_as_iface(&dev->desc_info, dir, config);
    if (!as) {
        ESP_LOGE(TAG, "No matching AS interface for %s %dch/%dbit",
                 dir == UAC2_STREAM_TX ? "TX" : "RX",
                 config->channels, config->bit_resolution);
        return ESP_ERR_NOT_FOUND;
    }

    // Full Speed isochronous data endpoints must use bInterval=1 (every frame = 1ms).
    // Some devices report incorrect values. Log a warning; stream_alloc uses packet_size
    // which doesn't depend on bInterval, and EP configuration uses the value from the
    // claimed interface. The canonical desc_info is not mutated.
    if (as->ep_interval != 1) {
        ESP_LOGW(TAG, "Data EP 0x%02X bInterval=%d (expected 1 at FS), using 1ms framing",
                 as->ep_addr, as->ep_interval);
    }

    ESP_LOGI(TAG, "Starting %s stream: iface %d alt %d, %dch %d-bit, ep 0x%02X (MPS %d)",
             dir == UAC2_STREAM_TX ? "TX" : "RX",
             as->interface_num, as->alt_setting,
             as->nr_channels, as->bit_resolution,
             as->ep_addr, as->ep_max_packet_size);

    // Allocate stream resources
    uac2_stream_t *stream;
    esp_err_t err = stream_alloc(&stream, dir, as, config);
    if (err != ESP_OK) return err;

    // Claim interface with the appropriate alt setting (host-side endpoint setup)
    err = usb_host_interface_claim(dev->client, dev->usb_dev,
                                   stream->iface_num, stream->alt_setting);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to claim iface %d alt %d: %s",
                 stream->iface_num, stream->alt_setting, esp_err_to_name(err));
        stream_free(stream);
        return err;
    }

    // Send SET_INTERFACE to notify the device to activate isochronous endpoints.
    // usb_host_interface_claim() only sets up host-side pipes — the device needs
    // a standard SET_INTERFACE request to switch from alt 0 (zero-bandwidth) to
    // the active alt setting.
    err = ctrl_request_no_data(dev,
                       USB_BM_REQUEST_TYPE_DIR_OUT | USB_BM_REQUEST_TYPE_TYPE_STANDARD | USB_BM_REQUEST_TYPE_RECIP_INTERFACE,
                       USB_B_REQUEST_SET_INTERFACE,
                       stream->alt_setting,         // wValue = bAlternateSetting
                       stream->iface_num);          // wIndex = bInterfaceNumber
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SET_INTERFACE(%d, %d) failed: %s (continuing — device may auto-activate)",
                 stream->iface_num, stream->alt_setting, esp_err_to_name(err));
    }

    stream->state = UAC2_STREAM_STATE_READY;

    // Set sample rate if we have a clock source
    if (dev->clock_source_id != 0 && config->sample_rate > 0) {
        err = uac2_host_set_sample_rate(dev, config->sample_rate);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to set sample rate (continuing anyway)");
        }
    }

    // Submit URBs to start streaming
    err = stream_submit_urbs(stream, dev);
    if (err != ESP_OK) {
        // Revert state so callbacks stop resubmitting
        portENTER_CRITICAL(&stream->state_lock);
        stream->state = UAC2_STREAM_STATE_IDLE;
        portEXIT_CRITICAL(&stream->state_lock);
        // Revert SET_INTERFACE to alt 0 so device deactivates endpoints
        ctrl_request_no_data(dev,
            USB_BM_REQUEST_TYPE_DIR_OUT | USB_BM_REQUEST_TYPE_TYPE_STANDARD | USB_BM_REQUEST_TYPE_RECIP_INTERFACE,
            USB_B_REQUEST_SET_INTERFACE, 0, stream->iface_num);
        usb_host_interface_release(dev->client, dev->usb_dev, stream->iface_num);
        // Wait for already-submitted URBs to drain before freeing
        int wait_ms = 0;
        while (atomic_load(&stream->urbs_in_flight) > 0 && wait_ms < 500) {
            vTaskDelay(pdMS_TO_TICKS(5));
            wait_ms += 5;
        }
        stream_free(stream);
        return err;
    }

    *stream_ptr = stream;
    ESP_LOGI(TAG, "%s stream started (pkt_size=%d, ringbuf=%" PRIu32 ")",
             dir == UAC2_STREAM_TX ? "TX" : "RX",
             stream->packet_size, stream->ringbuf_size);
    return ESP_OK;
}

esp_err_t uac2_host_stream_stop(uac2_host_device_handle_t dev,
                                uac2_stream_dir_t dir)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "Invalid device handle");

    uac2_stream_t **stream_ptr = (dir == UAC2_STREAM_TX)
                                 ? &dev->tx_stream : &dev->rx_stream;
    uac2_stream_t *stream = *stream_ptr;

    if (!stream) return ESP_OK;

    // Set state to IDLE under spinlock — callbacks check this atomically
    portENTER_CRITICAL(&stream->state_lock);
    stream->state = UAC2_STREAM_STATE_IDLE;
    portEXIT_CRITICAL(&stream->state_lock);

    // Notify device to deactivate endpoints first (switch to alt 0 / zero-bandwidth).
    // Must happen before halt/flush/clear so the device stops sending feedback/data
    // before we tear down host-side pipes. Matches Espressif UAC1 reference driver order.
    esp_err_t si_err = ctrl_request_no_data(dev,
                       USB_BM_REQUEST_TYPE_DIR_OUT | USB_BM_REQUEST_TYPE_TYPE_STANDARD | USB_BM_REQUEST_TYPE_RECIP_INTERFACE,
                       USB_B_REQUEST_SET_INTERFACE,
                       0,                           // wValue = alt 0 (zero-bandwidth)
                       stream->iface_num);          // wIndex = bInterfaceNumber
    if (si_err != ESP_OK) {
        ESP_LOGW(TAG, "SET_INTERFACE(%d, 0) failed: %s", stream->iface_num, esp_err_to_name(si_err));
    }

    // Then halt and flush host-side pipes (skip if device already gone)
    esp_err_t halt_err = usb_host_endpoint_halt(dev->usb_dev, stream->ep_addr);
    if (halt_err == ESP_OK) {
        usb_host_endpoint_flush(dev->usb_dev, stream->ep_addr);
        usb_host_endpoint_clear(dev->usb_dev, stream->ep_addr);
    }
    if (stream->fb_ep_addr) {
        halt_err = usb_host_endpoint_halt(dev->usb_dev, stream->fb_ep_addr);
        if (halt_err == ESP_OK) {
            usb_host_endpoint_flush(dev->usb_dev, stream->fb_ep_addr);
            usb_host_endpoint_clear(dev->usb_dev, stream->fb_ep_addr);
        }
    }

    // Release interface (cancels pending transfers). Retry for ESP-IDF bug #17707.
    // Note: first attempt typically sees 1 URB in-flight because cancellation callbacks
    // are dispatched via usb_host_client_handle_events() on the class driver task.
    // The 20ms retry delay gives that task time to process the cancelled URB callbacks.
    for (int retry = 0; retry < 5; retry++) {
        esp_err_t rel_err = usb_host_interface_release(dev->client, dev->usb_dev, stream->iface_num);
        if (rel_err == ESP_OK) break;
        if (rel_err == ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "Interface release: URBs in-flight, retry %d", retry + 1);
            vTaskDelay(pdMS_TO_TICKS(20));
        } else {
            ESP_LOGE(TAG, "Interface release failed: %s", esp_err_to_name(rel_err));
            break;
        }
    }

    // Wait for all in-flight URBs to complete (callbacks decrement the counter).
    // After interface_release cancels pending transfers, callbacks should fire quickly.
    // Keep retrying — freeing while URBs are in-flight causes use-after-free.
    int wait_ms = 0;
    while (atomic_load(&stream->urbs_in_flight) > 0 && wait_ms < 2000) {
        vTaskDelay(pdMS_TO_TICKS(5));
        wait_ms += 5;
    }
    if (atomic_load(&stream->urbs_in_flight) > 0) {
        ESP_LOGE(TAG, "Stream stop: %d URBs still in-flight after 2s — leaking stream to avoid crash",
                 atomic_load(&stream->urbs_in_flight));
        // Don't free: URB callbacks still reference this memory.
        // Null the pointer so the driver doesn't use it further.
        *stream_ptr = NULL;
        return ESP_ERR_TIMEOUT;
    }

    stream_free(stream);
    *stream_ptr = NULL;

    ESP_LOGI(TAG, "%s stream stopped", dir == UAC2_STREAM_TX ? "TX" : "RX");
    return ESP_OK;
}

esp_err_t uac2_host_stream_write(uac2_host_device_handle_t dev,
                                 const uint8_t *data, uint32_t size,
                                 uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(dev && data && size > 0, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    uac2_stream_t *stream = dev->tx_stream;
    if (!stream || stream->state != UAC2_STREAM_STATE_ACTIVE) {
        return ESP_ERR_INVALID_STATE;
    }

    atomic_store(&stream->user_task_blocked, true);
    BaseType_t ok = xRingbufferSend(stream->ringbuf, data, size,
                                    pdMS_TO_TICKS(timeout_ms));

    // Capture result BEFORE signaling stream_free — after the signal,
    // stream_free may delete the stream and ringbuf immediately.
    bool still_active = (stream->state == UAC2_STREAM_STATE_ACTIVE);
    if (ok == pdTRUE && still_active) {
        atomic_store(&stream->tx_done_pending, false);  // re-arm TX_DONE notification
    }

    // Signal stream_free that we've returned from the blocking call.
    // Must not access stream after this point.
    atomic_store(&stream->user_task_blocked, false);
    xSemaphoreGive(stream->user_task_done);

    if (!still_active) return ESP_ERR_INVALID_STATE;
    return ok == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t uac2_host_stream_read(uac2_host_device_handle_t dev,
                                uint8_t *data, uint32_t size,
                                uint32_t *bytes_read,
                                uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(dev && data && bytes_read && size > 0, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    uac2_stream_t *stream = dev->rx_stream;
    if (!stream || stream->state != UAC2_STREAM_STATE_ACTIVE) {
        return ESP_ERR_INVALID_STATE;
    }

    atomic_store(&stream->user_task_blocked, true);
    size_t item_size = 0;
    void *item = xRingbufferReceiveUpTo(stream->ringbuf, &item_size,
                                        pdMS_TO_TICKS(timeout_ms), size);

    // Complete all stream/ringbuf access BEFORE signaling stream_free.
    // After the signal, stream_free may free the stream immediately.
    esp_err_t ret;
    bool still_active = (stream->state == UAC2_STREAM_STATE_ACTIVE);
    if (!still_active) {
        if (item) {
            vRingbufferReturnItem(stream->ringbuf, item);
        }
        *bytes_read = 0;
        ret = ESP_ERR_INVALID_STATE;
    } else if (!item) {
        *bytes_read = 0;
        ret = ESP_ERR_TIMEOUT;
    } else {
        memcpy(data, item, item_size);
        vRingbufferReturnItem(stream->ringbuf, item);
        *bytes_read = (uint32_t)item_size;
        ret = ESP_OK;
    }

    // Signal stream_free. Must not access stream after this point.
    atomic_store(&stream->user_task_blocked, false);
    xSemaphoreGive(stream->user_task_done);
    return ret;
}

int64_t uac2_host_stream_get_start_time(uac2_host_device_handle_t dev)
{
    if (!dev) return 0;
    uac2_stream_t *s = dev->tx_stream;
    if (!s) return 0;
    return atomic_load(&s->first_frame_us);
}

uint32_t uac2_host_stream_get_feedback(uac2_host_device_handle_t dev)
{
    if (!dev) return 0;
    uac2_stream_t *s = dev->tx_stream;
    if (!s) return 0;
    return atomic_load(&s->fb_value);
}

// ── Public API: Volume / Mute ──────────────────────────────────────

esp_err_t uac2_host_set_mute(uac2_host_device_handle_t dev,
                             uint8_t channel, bool mute)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "Invalid device handle");
    ESP_RETURN_ON_FALSE(dev->has_feature_unit, ESP_ERR_NOT_SUPPORTED, TAG, "No feature unit");

    uint8_t data = mute ? 1 : 0;
    return ctrl_set_cur(dev, dev->feature_unit_id,
                        UAC2_FU_MUTE_CONTROL, channel, &data, 1);
}

esp_err_t uac2_host_get_mute(uac2_host_device_handle_t dev,
                             uint8_t channel, bool *mute)
{
    ESP_RETURN_ON_FALSE(dev && mute, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    ESP_RETURN_ON_FALSE(dev->has_feature_unit, ESP_ERR_NOT_SUPPORTED, TAG, "No feature unit");

    uint8_t data = 0;
    esp_err_t err = ctrl_get_cur(dev, dev->feature_unit_id,
                                 UAC2_FU_MUTE_CONTROL, channel, &data, 1);
    if (err != ESP_OK) return err;

    *mute = (data != 0);
    return ESP_OK;
}

esp_err_t uac2_host_set_volume(uac2_host_device_handle_t dev,
                               uint8_t channel, int16_t volume_db256)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "Invalid device handle");
    ESP_RETURN_ON_FALSE(dev->has_feature_unit, ESP_ERR_NOT_SUPPORTED, TAG, "No feature unit");

    uint8_t data[2] = {
        (uint8_t)(volume_db256 & 0xFF),
        (uint8_t)((volume_db256 >> 8) & 0xFF),
    };
    return ctrl_set_cur(dev, dev->feature_unit_id,
                        UAC2_FU_VOLUME_CONTROL, channel, data, 2);
}

esp_err_t uac2_host_get_volume(uac2_host_device_handle_t dev,
                               uint8_t channel, int16_t *volume_db256)
{
    ESP_RETURN_ON_FALSE(dev && volume_db256, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    ESP_RETURN_ON_FALSE(dev->has_feature_unit, ESP_ERR_NOT_SUPPORTED, TAG, "No feature unit");

    uint8_t data[2] = {0};
    esp_err_t err = ctrl_get_cur(dev, dev->feature_unit_id,
                                 UAC2_FU_VOLUME_CONTROL, channel, data, 2);
    if (err != ESP_OK) return err;

    *volume_db256 = (int16_t)(data[0] | (data[1] << 8));
    return ESP_OK;
}

esp_err_t uac2_host_get_volume_range(uac2_host_device_handle_t dev,
                                     uint8_t channel,
                                     uac2_volume_range_t *ranges,
                                     uint8_t *num_ranges)
{
    ESP_RETURN_ON_FALSE(dev && ranges && num_ranges, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    ESP_RETURN_ON_FALSE(dev->has_feature_unit, ESP_ERR_NOT_SUPPORTED, TAG, "No feature unit");

    uint8_t buf[2 + UAC2_MAX_VOLUME_RANGES * 6];
    memset(buf, 0, sizeof(buf));
    esp_err_t err = ctrl_get_range(dev, dev->feature_unit_id,
                                   UAC2_FU_VOLUME_CONTROL, channel,
                                   buf, sizeof(buf));
    if (err != ESP_OK) return err;

    uint16_t count = buf[0] | (buf[1] << 8);
    if (count > UAC2_MAX_VOLUME_RANGES) {
        count = UAC2_MAX_VOLUME_RANGES;
    }

    for (int i = 0; i < count; i++) {
        const uint8_t *p = buf + 2 + (i * 6);
        ranges[i].min = (int16_t)(p[0] | (p[1] << 8));
        ranges[i].max = (int16_t)(p[2] | (p[3] << 8));
        ranges[i].res = (int16_t)(p[4] | (p[5] << 8));

        ESP_LOGI(TAG, "Volume range %d: min=%.2f dB max=%.2f dB res=%.4f dB",
                 i, ranges[i].min / 256.0, ranges[i].max / 256.0, ranges[i].res / 256.0);
    }

    *num_ranges = (uint8_t)count;
    return ESP_OK;
}
