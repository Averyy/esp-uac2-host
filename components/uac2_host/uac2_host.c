/**
 * @file uac2_host.c
 * @brief USB Audio Class 2.0 host driver implementation
 *
 * SPDX-License-Identifier: MIT
 */

#include <string.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/ringbuf.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "usb/usb_host.h"
#include "uac2_host.h"

static const char *TAG = "uac2-host-drv";

// Spinlock for stream state transitions (protects state checks in callbacks
// from racing with stream_stop)
static portMUX_TYPE uac2_stream_lock = portMUX_INITIALIZER_UNLOCKED;

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
    uint32_t fb_value;          // latest feedback in 16.16 format
    uint32_t fb_accumulator;    // fractional sample accumulator for adaptive sizing

    // Isochronous URBs
    usb_transfer_t *xfer[UAC2_NUM_ISOC_URBS];
    int xfer_count;
    atomic_int urbs_in_flight;      // decremented by callbacks, waited on by stream_stop

    // Timing
    int64_t first_frame_us;         // esp_timer_get_time() when first URB submitted (0 = not yet)

    // Ring buffer
    RingbufHandle_t ringbuf;
    uint32_t ringbuf_size;
    uint32_t ringbuf_threshold;
} uac2_stream_t;

struct uac2_host_device {
    // USB handles
    usb_host_client_handle_t client;
    usb_device_handle_t usb_dev;

    // Device info
    uint16_t vid;
    uint16_t pid;
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

    // Event callback
    uac2_host_event_cb_t event_cb;
    void *event_cb_arg;

    // Streams
    uac2_stream_t *tx_stream;
    uac2_stream_t *rx_stream;
};

// ── Control transfer helpers ───────────────────────────────────────

static void ctrl_xfer_cb(usb_transfer_t *xfer)
{
    uac2_host_device_handle_t dev = (uac2_host_device_handle_t)xfer->context;
    xSemaphoreGive(dev->ctrl_xfer_done);
}

/**
 * Send a class-specific control request and wait for completion.
 * For SET: data_in should be NULL, data_out points to payload.
 * For GET: data_out should be NULL, response is in ctrl_xfer->data_buffer + 8.
 */
static esp_err_t ctrl_request(uac2_host_device_handle_t dev,
                              uint8_t bm_request_type,
                              uint8_t b_request,
                              uint16_t w_value,
                              uint16_t w_index,
                              uint16_t w_length,
                              const uint8_t *data_out)
{
    if (!dev || !dev->ctrl_xfer) {
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

    esp_err_t err = usb_host_transfer_submit_control(dev->client, xfer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Control submit failed: %s", esp_err_to_name(err));
        xSemaphoreGive(dev->ctrl_mutex);
        return err;
    }

    // Wait for completion
    if (xSemaphoreTake(dev->ctrl_xfer_done, pdMS_TO_TICKS(UAC2_CTRL_XFER_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Control transfer timeout");
        // Drain any late callback signal to prevent next transfer from seeing stale data.
        // The USB transfer may still complete after our timeout — when its callback fires
        // and signals the semaphore, we don't want the NEXT ctrl_request to see it.
        xSemaphoreTake(dev->ctrl_xfer_done, pdMS_TO_TICKS(100));
        xSemaphoreGive(dev->ctrl_mutex);
        return ESP_ERR_TIMEOUT;
    }

    if (xfer->status != USB_TRANSFER_STATUS_COMPLETED) {
        ESP_LOGE(TAG, "Control transfer failed, status=%d", xfer->status);
        xSemaphoreGive(dev->ctrl_mutex);
        return ESP_FAIL;
    }

    // Mutex stays held — caller must release after reading response data.
    // ctrl_set_cur releases immediately; ctrl_get_cur/range copy data then release.
    return ESP_OK;
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

    portENTER_CRITICAL(&uac2_stream_lock);
    bool active = stream && stream->state == UAC2_STREAM_STATE_ACTIVE;
    portEXIT_CRITICAL(&uac2_stream_lock);

    if (!active) {
        if (stream) atomic_fetch_sub(&stream->urbs_in_flight, 1);
        return;
    }

    switch (xfer->status) {
    case USB_TRANSFER_STATUS_COMPLETED:
        stream_tx_xfer_submit(stream, dev, xfer);
        break;

    case USB_TRANSFER_STATUS_NO_DEVICE:
    case USB_TRANSFER_STATUS_CANCELED:
        atomic_fetch_sub(&stream->urbs_in_flight, 1);
        return;

    default:
        ESP_LOGW(TAG, "TX transfer error, status=%d", xfer->status);
        if (dev->event_cb) {
            dev->event_cb(dev, UAC2_HOST_EVENT_TRANSFER_ERROR, dev->event_cb_arg);
        }
        stream_tx_xfer_submit(stream, dev, xfer);
        break;
    }
}

static void stream_tx_xfer_submit(uac2_stream_t *stream,
                                  uac2_host_device_handle_t dev,
                                  usb_transfer_t *xfer)
{
    if (stream->state != UAC2_STREAM_STATE_ACTIVE) {
        return;
    }

    size_t item_size = 0;

    // Feedback-based adaptive packet sizing: the device's feedback endpoint reports
    // its ideal samples-per-frame in 16.16 fixed-point. We accumulate the fractional
    // part and send one extra sample when it overflows. This prevents the device's
    // buffer from slowly under/overrunning during sustained playback.
    uint16_t pkt_size;
    uint32_t fb = stream->fb_value;
    if (fb > 0) {
        uint16_t nominal_samples = (uint16_t)(fb >> 16);
        uint16_t fraction = (uint16_t)(fb & 0xFFFF);

        stream->fb_accumulator += fraction;
        uint16_t extra = (uint16_t)(stream->fb_accumulator >> 16);
        stream->fb_accumulator &= 0xFFFF;

        uint16_t samples_this_frame = nominal_samples + extra;
        uint32_t raw_pkt_size = (uint32_t)samples_this_frame * stream->channels * stream->sub_slot_size;
        pkt_size = (raw_pkt_size > stream->ep_mps) ? stream->ep_mps : (uint16_t)raw_pkt_size;
    } else {
        pkt_size = stream->packet_size;
    }

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

        // Notify when ringbuf drops below threshold
        UBaseType_t rb_free, rb_read, rb_write, rb_acq, rb_wait;
        vRingbufferGetInfo(stream->ringbuf, &rb_free, &rb_read, &rb_write, &rb_acq, &rb_wait);
        size_t rb_used = stream->ringbuf_size - rb_free;
        if (rb_used < stream->ringbuf_threshold && dev->event_cb) {
            dev->event_cb(dev, UAC2_HOST_EVENT_TX_DONE, dev->event_cb_arg);
        }
    } else {
        // No data: send silence, always notify
        memset(xfer->data_buffer, 0, pkt_size);
        if (dev->event_cb) {
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

    portENTER_CRITICAL(&uac2_stream_lock);
    bool active = stream && stream->state == UAC2_STREAM_STATE_ACTIVE;
    portEXIT_CRITICAL(&uac2_stream_lock);

    if (!active) {
        if (stream) atomic_fetch_sub(&stream->urbs_in_flight, 1);
        return;
    }

    switch (xfer->status) {
    case USB_TRANSFER_STATUS_COMPLETED:
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
            UBaseType_t rb_free, rb_read, rb_write, rb_acq, rb_wait;
            vRingbufferGetInfo(stream->ringbuf, &rb_free, &rb_read, &rb_write, &rb_acq, &rb_wait);
            size_t rb_used = stream->ringbuf_size - rb_free;
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
            }
        }
        break;

    case USB_TRANSFER_STATUS_NO_DEVICE:
    case USB_TRANSFER_STATUS_CANCELED:
        atomic_fetch_sub(&stream->urbs_in_flight, 1);
        return;

    default:
        ESP_LOGW(TAG, "RX transfer error, status=%d", xfer->status);
        {
            esp_err_t sub_err = usb_host_transfer_submit(xfer);
            if (sub_err != ESP_OK) {
                ESP_LOGW(TAG, "RX resubmit failed: %s", esp_err_to_name(sub_err));
                atomic_fetch_sub(&stream->urbs_in_flight, 1);
            }
        }
        break;
    }
}

static void feedback_xfer_done(usb_transfer_t *xfer)
{
    uac2_host_device_handle_t dev = (uac2_host_device_handle_t)xfer->context;
    uac2_stream_t *stream = dev->tx_stream;

    portENTER_CRITICAL(&uac2_stream_lock);
    bool active = stream && stream->state == UAC2_STREAM_STATE_ACTIVE;
    portEXIT_CRITICAL(&uac2_stream_lock);

    if (!active) {
        if (stream) atomic_fetch_sub(&stream->urbs_in_flight, 1);
        return;
    }

    // TODO(hardware): Verify feedback format with actual miniDSP. XMOS source code
    // confirms 3 bytes / 10.14 format at Full Speed (16.16 >> 2), despite MPS=4
    // (MPS=4 is for Windows UAC2 driver compatibility). Feedback arrives every
    // 2^(bInterval-1) = 8 frames = 8ms. For 48kHz, nominal = 0x0C0000 in 10.14.
    // Log actual_num_bytes on first callbacks to confirm 3 vs 4.
    if (xfer->status == USB_TRANSFER_STATUS_COMPLETED) {
        int actual = xfer->isoc_packet_desc[0].actual_num_bytes;

        if (actual == 4) {
            // 16.16 format — deserialize little-endian explicitly
            uint32_t raw = xfer->data_buffer[0]
                         | ((uint32_t)xfer->data_buffer[1] << 8)
                         | ((uint32_t)xfer->data_buffer[2] << 16)
                         | ((uint32_t)xfer->data_buffer[3] << 24);
            stream->fb_value = raw;
        } else if (actual == 3) {
            // 10.14 format (standard FS feedback), convert to 16.16
            uint32_t raw = xfer->data_buffer[0]
                         | (xfer->data_buffer[1] << 8)
                         | (xfer->data_buffer[2] << 16);
            // 10.14 -> 16.16: shift left by 2
            stream->fb_value = raw << 2;
        }

        ESP_LOGD(TAG, "Feedback: %lu.%04lu Hz",
                 (unsigned long)(stream->fb_value >> 16),
                 (unsigned long)((stream->fb_value & 0xFFFF) * 10000 / 65536));
    }

    // Resubmit feedback URB
    if (stream->state == UAC2_STREAM_STATE_ACTIVE) {
        xfer->isoc_packet_desc[0].num_bytes = xfer->data_buffer_size;
        xfer->num_bytes = xfer->data_buffer_size;
        esp_err_t sub_err = usb_host_transfer_submit(xfer);
        if (sub_err != ESP_OK) {
            ESP_LOGW(TAG, "Feedback resubmit failed: %s", esp_err_to_name(sub_err));
            atomic_fetch_sub(&stream->urbs_in_flight, 1);
        }
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
    return (uint16_t)(samples_per_frame * channels * sub_slot_size);
}

static esp_err_t stream_alloc(uac2_stream_t **out_stream,
                              uac2_stream_dir_t dir,
                              const uac2_as_iface_t *as,
                              const uac2_stream_config_t *config)
{
    uac2_stream_t *stream = calloc(1, sizeof(uac2_stream_t));
    if (!stream) return ESP_ERR_NO_MEM;

    stream->dir = dir;
    stream->state = UAC2_STREAM_STATE_IDLE;
    stream->iface_num = as->interface_num;
    stream->alt_setting = as->alt_setting;
    stream->sample_rate = config->sample_rate;
    stream->channels = as->nr_channels;
    stream->bit_resolution = as->bit_resolution;
    stream->sub_slot_size = as->sub_slot_size;
    stream->ep_addr = as->ep_addr;
    stream->ep_mps = as->ep_max_packet_size;
    stream->fb_ep_addr = as->fb_ep_addr;
    stream->fb_value = 0;
    stream->fb_accumulator = 0;
    atomic_init(&stream->urbs_in_flight, 0);
    stream->first_frame_us = 0;

    stream->packet_size = calc_packet_size(config->sample_rate,
                                           as->nr_channels,
                                           as->sub_slot_size);

    // Reject if packet size exceeds endpoint MPS
    if (stream->packet_size > stream->ep_mps) {
        ESP_LOGE(TAG, "Packet size %d exceeds endpoint MPS %d — config not supported",
                 stream->packet_size, stream->ep_mps);
        free(stream);
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
        free(stream);
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
            free(stream);
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
        vRingbufferDelete(stream->ringbuf);
    }
    free(stream);
}

static esp_err_t stream_submit_urbs(uac2_stream_t *stream,
                                    uac2_host_device_handle_t dev)
{
    int num_pkts = UAC2_NUM_PACKETS_PER_URB;

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

    stream->state = UAC2_STREAM_STATE_ACTIVE;
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

// ── Public API: Device management ──────────────────────────────────

esp_err_t uac2_host_device_open(usb_host_client_handle_t client,
                                usb_device_handle_t usb_dev,
                                uac2_host_event_cb_t cb, void *cb_arg,
                                uac2_host_device_handle_t *out_dev)
{
    if (!client || !usb_dev || !out_dev) {
        return ESP_ERR_INVALID_ARG;
    }

    // Get config descriptor
    const usb_config_desc_t *config_desc;
    esp_err_t err = usb_host_get_active_config_descriptor(usb_dev, &config_desc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get config descriptor: %s", esp_err_to_name(err));
        return err;
    }

    // Parse UAC2 descriptors
    uac2_device_info_t desc_info;
    bool is_uac2 = uac2_parse_config_descriptor(
        (const uint8_t *)config_desc, config_desc->wTotalLength, &desc_info);

    if (!is_uac2) {
        ESP_LOGW(TAG, "Not a UAC2 device");
        return ESP_ERR_NOT_SUPPORTED;
    }

    // Get VID/PID
    const usb_device_desc_t *dev_desc;
    err = usb_host_get_device_descriptor(usb_dev, &dev_desc);
    if (err != ESP_OK) {
        return err;
    }

    // Allocate device
    uac2_host_device_handle_t dev = calloc(1, sizeof(struct uac2_host_device));
    if (!dev) return ESP_ERR_NO_MEM;

    dev->client = client;
    dev->usb_dev = usb_dev;
    dev->vid = dev_desc->idVendor;
    dev->pid = dev_desc->idProduct;
    dev->desc_info = desc_info;
    dev->event_cb = cb;
    dev->event_cb_arg = cb_arg;

    // AC interface number was extracted by the descriptor parser
    dev->ac_iface_num = desc_info.ac_iface_num;

    // Resolve clock source
    dev->clock_source_id = resolve_clock_source(&desc_info);

    // Resolve feature unit (use first one for volume/mute)
    if (desc_info.num_feature_units > 0) {
        dev->has_feature_unit = true;
        dev->feature_unit_id = desc_info.feature_units[0].unit_id;
    }

    // Allocate control transfer
    err = usb_host_transfer_alloc(sizeof(usb_setup_packet_t) + UAC2_CTRL_XFER_MAX_SIZE,
                                  0, &dev->ctrl_xfer);
    if (err != ESP_OK) {
        free(dev);
        return err;
    }

    dev->ctrl_xfer_done = xSemaphoreCreateBinary();
    if (!dev->ctrl_xfer_done) {
        usb_host_transfer_free(dev->ctrl_xfer);
        free(dev);
        return ESP_ERR_NO_MEM;
    }

    dev->ctrl_mutex = xSemaphoreCreateMutex();
    if (!dev->ctrl_mutex) {
        vSemaphoreDelete(dev->ctrl_xfer_done);
        usb_host_transfer_free(dev->ctrl_xfer);
        free(dev);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "UAC2 device opened: VID=0x%04X PID=0x%04X", dev->vid, dev->pid);
    ESP_LOGI(TAG, "  AC iface=%d, clock_source=%d, feature_unit=%d",
             dev->ac_iface_num, dev->clock_source_id,
             dev->has_feature_unit ? dev->feature_unit_id : 0);
    ESP_LOGI(TAG, "  %d AS interfaces, %d clock sources, %d terminals",
             desc_info.num_as_ifaces, desc_info.num_clock_sources,
             desc_info.num_terminals);

    *out_dev = dev;
    return ESP_OK;
}

esp_err_t uac2_host_device_close(uac2_host_device_handle_t dev)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    // Stop any active streams
    if (dev->tx_stream) {
        uac2_host_stream_stop(dev, UAC2_STREAM_TX);
    }
    if (dev->rx_stream) {
        uac2_host_stream_stop(dev, UAC2_STREAM_RX);
    }

    // Free control transfer
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
    free(dev);
    return ESP_OK;
}

esp_err_t uac2_host_device_get_info(uac2_host_device_handle_t dev,
                                    uac2_device_info_t *info)
{
    if (!dev || !info) return ESP_ERR_INVALID_ARG;
    *info = dev->desc_info;
    return ESP_OK;
}

// ── Public API: Clock control ──────────────────────────────────────

esp_err_t uac2_host_get_sample_rate(uac2_host_device_handle_t dev,
                                    uint32_t *sample_rate)
{
    if (!dev || !sample_rate) return ESP_ERR_INVALID_ARG;
    if (dev->clock_source_id == 0) return ESP_ERR_NOT_SUPPORTED;

    uint8_t data[4] = {0};
    esp_err_t err = ctrl_get_cur(dev, dev->clock_source_id,
                                 UAC2_CS_SAM_FREQ_CONTROL, 0, data, 4);
    if (err != ESP_OK) return err;

    *sample_rate = (uint32_t)data[0] | ((uint32_t)data[1] << 8)
                 | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
    ESP_LOGI(TAG, "Current sample rate: %lu Hz", (unsigned long)*sample_rate);
    return ESP_OK;
}

esp_err_t uac2_host_set_sample_rate(uac2_host_device_handle_t dev,
                                    uint32_t sample_rate)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    if (dev->clock_source_id == 0) return ESP_ERR_NOT_SUPPORTED;

    uint8_t data[4] = {
        (uint8_t)(sample_rate & 0xFF),
        (uint8_t)((sample_rate >> 8) & 0xFF),
        (uint8_t)((sample_rate >> 16) & 0xFF),
        (uint8_t)((sample_rate >> 24) & 0xFF),
    };

    esp_err_t err = ctrl_set_cur(dev, dev->clock_source_id,
                                 UAC2_CS_SAM_FREQ_CONTROL, 0, data, 4);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Set sample rate: %lu Hz", (unsigned long)sample_rate);
    }
    return err;
}

esp_err_t uac2_host_get_sample_rate_range(uac2_host_device_handle_t dev,
                                          uac2_sample_rate_range_t *ranges,
                                          uint8_t *num_ranges)
{
    if (!dev || !ranges || !num_ranges) return ESP_ERR_INVALID_ARG;
    if (dev->clock_source_id == 0) return ESP_ERR_NOT_SUPPORTED;

    // First: get just the count (2 bytes)
    uint8_t buf[2 + UAC2_MAX_SAMPLE_RATE_RANGES * 12];
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

        ESP_LOGI(TAG, "Sample rate range %d: %lu - %lu Hz (res %lu)",
                 i, (unsigned long)ranges[i].min,
                 (unsigned long)ranges[i].max,
                 (unsigned long)ranges[i].res);
    }

    *num_ranges = (uint8_t)count;
    return ESP_OK;
}

esp_err_t uac2_host_get_clock_valid(uac2_host_device_handle_t dev,
                                    bool *valid)
{
    if (!dev || !valid) return ESP_ERR_INVALID_ARG;
    if (dev->clock_source_id == 0) return ESP_ERR_NOT_SUPPORTED;

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
    if (!dev || !config) return ESP_ERR_INVALID_ARG;

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

    ESP_LOGI(TAG, "Starting %s stream: iface %d alt %d, %dch %d-bit, ep 0x%02X (MPS %d)",
             dir == UAC2_STREAM_TX ? "TX" : "RX",
             as->interface_num, as->alt_setting,
             as->nr_channels, as->bit_resolution,
             as->ep_addr, as->ep_max_packet_size);

    // Allocate stream resources
    uac2_stream_t *stream;
    esp_err_t err = stream_alloc(&stream, dir, as, config);
    if (err != ESP_OK) return err;

    // Claim interface with the appropriate alt setting
    // TODO(hardware): Some XMOS devices need a delay after SET_INTERFACE before
    // submitting isochronous URBs. If we see transfer errors immediately after
    // stream start, add a small delay (5-10ms) here after the claim returns.
    err = usb_host_interface_claim(dev->client, dev->usb_dev,
                                   stream->iface_num, stream->alt_setting);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to claim iface %d alt %d: %s",
                 stream->iface_num, stream->alt_setting, esp_err_to_name(err));
        stream_free(stream);
        return err;
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
        usb_host_interface_release(dev->client, dev->usb_dev, stream->iface_num);
        stream_free(stream);
        return err;
    }

    *stream_ptr = stream;
    ESP_LOGI(TAG, "%s stream started (pkt_size=%d, ringbuf=%lu)",
             dir == UAC2_STREAM_TX ? "TX" : "RX",
             stream->packet_size, (unsigned long)stream->ringbuf_size);
    return ESP_OK;
}

esp_err_t uac2_host_stream_stop(uac2_host_device_handle_t dev,
                                uac2_stream_dir_t dir)
{
    if (!dev) return ESP_ERR_INVALID_ARG;

    uac2_stream_t **stream_ptr = (dir == UAC2_STREAM_TX)
                                 ? &dev->tx_stream : &dev->rx_stream;
    uac2_stream_t *stream = *stream_ptr;

    if (!stream) return ESP_OK;

    // Set state to IDLE under spinlock — callbacks check this atomically
    portENTER_CRITICAL(&uac2_stream_lock);
    stream->state = UAC2_STREAM_STATE_IDLE;
    portEXIT_CRITICAL(&uac2_stream_lock);

    // Release interface (cancels pending transfers). Retry for ESP-IDF bug #17707.
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

    // Wait for all in-flight URBs to complete (callbacks decrement the counter)
    int wait_ms = 0;
    while (atomic_load(&stream->urbs_in_flight) > 0 && wait_ms < 500) {
        vTaskDelay(pdMS_TO_TICKS(5));
        wait_ms += 5;
    }
    if (atomic_load(&stream->urbs_in_flight) > 0) {
        ESP_LOGW(TAG, "Stream stop: %d URBs still in-flight after 500ms",
                 atomic_load(&stream->urbs_in_flight));
    }

    // Flush ring buffer before freeing
    if (stream->ringbuf) {
        size_t item_size = 0;
        void *item;
        while ((item = xRingbufferReceiveUpTo(stream->ringbuf, &item_size, 0,
                                               stream->ringbuf_size)) != NULL) {
            vRingbufferReturnItem(stream->ringbuf, item);
        }
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
    if (!dev || !data || size == 0) return ESP_ERR_INVALID_ARG;

    uac2_stream_t *stream = dev->tx_stream;
    if (!stream || stream->state != UAC2_STREAM_STATE_ACTIVE) {
        return ESP_ERR_INVALID_STATE;
    }

    BaseType_t ok = xRingbufferSend(stream->ringbuf, data, size,
                                    pdMS_TO_TICKS(timeout_ms));
    return ok == pdTRUE ? ESP_OK : ESP_FAIL;
}

esp_err_t uac2_host_stream_read(uac2_host_device_handle_t dev,
                                uint8_t *data, uint32_t size,
                                uint32_t *bytes_read,
                                uint32_t timeout_ms)
{
    if (!dev || !data || !bytes_read || size == 0) return ESP_ERR_INVALID_ARG;

    uac2_stream_t *stream = dev->rx_stream;
    if (!stream || stream->state != UAC2_STREAM_STATE_ACTIVE) {
        return ESP_ERR_INVALID_STATE;
    }

    size_t item_size = 0;
    void *item = xRingbufferReceiveUpTo(stream->ringbuf, &item_size,
                                        pdMS_TO_TICKS(timeout_ms), size);
    if (!item) {
        *bytes_read = 0;
        return ESP_ERR_TIMEOUT;
    }

    memcpy(data, item, item_size);
    vRingbufferReturnItem(stream->ringbuf, item);
    *bytes_read = (uint32_t)item_size;
    return ESP_OK;
}

int64_t uac2_host_stream_get_start_time(uac2_host_device_handle_t dev)
{
    if (!dev || !dev->tx_stream) return 0;
    return dev->tx_stream->first_frame_us;
}

// ── Public API: Volume / Mute ──────────────────────────────────────

esp_err_t uac2_host_set_mute(uac2_host_device_handle_t dev,
                             uint8_t channel, bool mute)
{
    if (!dev) return ESP_ERR_INVALID_ARG;
    if (!dev->has_feature_unit) return ESP_ERR_NOT_SUPPORTED;

    uint8_t data = mute ? 1 : 0;
    return ctrl_set_cur(dev, dev->feature_unit_id,
                        UAC2_FU_MUTE_CONTROL, channel, &data, 1);
}

esp_err_t uac2_host_get_mute(uac2_host_device_handle_t dev,
                             uint8_t channel, bool *mute)
{
    if (!dev || !mute) return ESP_ERR_INVALID_ARG;
    if (!dev->has_feature_unit) return ESP_ERR_NOT_SUPPORTED;

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
    if (!dev) return ESP_ERR_INVALID_ARG;
    if (!dev->has_feature_unit) return ESP_ERR_NOT_SUPPORTED;

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
    if (!dev || !volume_db256) return ESP_ERR_INVALID_ARG;
    if (!dev->has_feature_unit) return ESP_ERR_NOT_SUPPORTED;

    uint8_t data[2] = {0};
    esp_err_t err = ctrl_get_cur(dev, dev->feature_unit_id,
                                 UAC2_FU_VOLUME_CONTROL, channel, data, 2);
    if (err != ESP_OK) return err;

    *volume_db256 = (int16_t)(data[0] | (data[1] << 8));
    return ESP_OK;
}
