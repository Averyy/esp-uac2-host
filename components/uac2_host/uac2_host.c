/*
 * SPDX-FileCopyrightText: 2026 Avery Levitt
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file uac2_host.c
 * @brief USB Audio Class 2.0 host driver implementation
 *
 * Espressif-pattern class driver with install/uninstall lifecycle,
 * internal device discovery, linked list management, and reference
 * counting on shared physical USB devices.
 */

#include <string.h>
#include <stdatomic.h>
#include <inttypes.h>
#include <sys/queue.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/ringbuf.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_check.h"
#include "usb/usb_host.h"
#include "usb/uac2_host.h"

static const char *TAG = "uac2-host";

// ── UAC2 control request constants ─────────────────────────────────

#define UAC2_FU_MUTE_CONTROL            0x01
#define UAC2_FU_VOLUME_CONTROL          0x02

#define UAC2_CTRL_SET   (USB_BM_REQUEST_TYPE_DIR_OUT | USB_BM_REQUEST_TYPE_TYPE_CLASS | USB_BM_REQUEST_TYPE_RECIP_INTERFACE)
#define UAC2_CTRL_GET   (USB_BM_REQUEST_TYPE_DIR_IN  | USB_BM_REQUEST_TYPE_TYPE_CLASS | USB_BM_REQUEST_TYPE_RECIP_INTERFACE)

// ── Critical section ───────────────────────────────────────────────

static portMUX_TYPE s_uac2_lock = portMUX_INITIALIZER_UNLOCKED;
#define UAC2_ENTER_CRITICAL()   portENTER_CRITICAL(&s_uac2_lock)
#define UAC2_EXIT_CRITICAL()    portEXIT_CRITICAL(&s_uac2_lock)

// ── Internal types ─────────────────────────────────────────────────

typedef enum {
    UAC2_IFACE_STATE_IDLE = 0,
    UAC2_IFACE_STATE_READY,
    UAC2_IFACE_STATE_ACTIVE,
    UAC2_IFACE_STATE_SUSPENDING,    // internal: transient during suspend, never observable externally
    UAC2_IFACE_STATE_ERROR,
} uac2_iface_state_t;

// (FLAG_IFACE_WAIT_USER_DELETE removed — not needed; user must call device_close after DISCONNECTED)

/**
 * Physical USB device — shared by multiple interfaces on the same device.
 * Reference counted via opened_cnt.
 */
typedef struct uac2_device {
    STAILQ_ENTRY(uac2_device) tailq_entry;
    usb_device_handle_t dev_hdl;
    uint8_t addr;
    uint8_t opened_cnt;             // number of open interfaces

    // Descriptor cache (parsed at first open)
    uac2_device_info_t desc_info;
    uint8_t ac_iface_num;
    uint8_t clock_source_id;
    uint8_t feature_unit_id;
    bool has_feature_unit;
    bool has_mute;          // cached from desc_info.feature_units[0].has_mute
    bool has_volume;        // cached from desc_info.feature_units[0].has_volume
    bool volume_range_valid;
    int16_t volume_min_db256;
    int16_t volume_max_db256;
    int16_t volume_res_db256;

    // Device gone flag — set on USB_HOST_CLIENT_EVENT_DEV_GONE, checked by
    // stream_stop_internal to skip SET_INTERFACE on a device that's already disconnected
    _Atomic bool gone;

    // Control transfer (shared across all interfaces on this device)
    usb_transfer_t *ctrl_xfer;
    SemaphoreHandle_t ctrl_xfer_done;
    SemaphoreHandle_t ctrl_mutex;
    // Generation counters for timeout/stale-callback detection. Both start at 0;
    // this is safe because a new device has no prior submitted transfers.
    atomic_uint ctrl_xfer_gen;
    atomic_uint ctrl_xfer_submitted_gen;
} uac2_device_t;

/**
 * Logical interface — one per opened UAC2 AS interface.
 * This is the user-facing handle (uac2_host_device_handle_t).
 */
typedef struct uac2_interface {
    STAILQ_ENTRY(uac2_interface) tailq_entry;
    uac2_device_t *parent;

    // Interface identity
    uac2_stream_dir_t dir;
    uint8_t iface_num;

    // API mutex (serializes public API calls except write/read)
    SemaphoreHandle_t api_mutex;

    // Event callback
    uac2_host_device_event_cb_t user_cb;
    void *user_cb_arg;

    // Buffer config (from device_open, used at device_start)
    uint32_t cfg_buffer_size;
    uint32_t cfg_buffer_threshold;

    // Disconnect tracking
    _Atomic bool disconnect_fired;

    // ── Stream state (folded from former uac2_stream_t) ──

    uac2_iface_state_t state;
    portMUX_TYPE state_lock;

    // Audio parameters (set during device_start)
    uint8_t  alt_setting;
    uint32_t sample_rate;
    uint8_t  channels;
    uint8_t  bit_resolution;
    uint8_t  sub_slot_size;
    uint16_t packet_size;

    // Data endpoint
    uint8_t  ep_addr;
    uint16_t ep_mps;

    // Feedback endpoint (async playback only)
    uint8_t  fb_ep_addr;
    usb_transfer_t *fb_xfer;
    atomic_uint fb_value;           // 16.16 feedback
    uint32_t fb_accumulator;        // fractional sample accumulator (callback context only)

    // Isochronous URBs
    usb_transfer_t *xfer[UAC2_NUM_ISOC_URBS];
    int xfer_count;
    atomic_int urbs_in_flight;
    atomic_int consecutive_errors;

    // Timing
    _Atomic int64_t first_frame_us;

    // Ring buffer
    RingbufHandle_t ringbuf;
    uint32_t ringbuf_size;
    uint32_t ringbuf_threshold;
    _Atomic bool tx_done_pending;

    // Sync for safe stream resource free
    SemaphoreHandle_t user_task_done;
    _Atomic bool user_task_blocked;
} uac2_iface_t;

/**
 * Singleton driver state — created by uac2_host_install().
 */
typedef struct {
    STAILQ_HEAD(devices, uac2_device) devices_tailq;
    STAILQ_HEAD(interfaces, uac2_interface) ifaces_tailq;
    volatile bool end_client_event_handling;
    bool event_handling_started;
    usb_host_client_handle_t client_handle;
    uac2_host_driver_event_cb_t user_cb;
    void *user_arg;
    SemaphoreHandle_t all_events_handled;
} uac2_driver_t;

static uac2_driver_t *s_uac2_driver;

// ── Forward declarations ──────────────────────────────────────────

static esp_err_t stream_stop_internal(uac2_iface_t *iface);
static void stream_tx_xfer_submit(uac2_iface_t *iface, usb_transfer_t *xfer);

// ── Lookup helpers ────────────────────────────────────────────────

static uac2_device_t *get_device_by_addr(uint8_t addr)
{
    uac2_device_t *dev = NULL;
    UAC2_ENTER_CRITICAL();
    STAILQ_FOREACH(dev, &s_uac2_driver->devices_tailq, tailq_entry) {
        if (dev->addr == addr) {
            UAC2_EXIT_CRITICAL();
            return dev;
        }
    }
    UAC2_EXIT_CRITICAL();
    return NULL;
}

static uac2_device_t *get_device_by_handle(usb_device_handle_t usb_handle)
{
    uac2_device_t *dev = NULL;
    UAC2_ENTER_CRITICAL();
    STAILQ_FOREACH(dev, &s_uac2_driver->devices_tailq, tailq_entry) {
        if (dev->dev_hdl == usb_handle) {
            UAC2_EXIT_CRITICAL();
            return dev;
        }
    }
    UAC2_EXIT_CRITICAL();
    return NULL;
}

static uac2_iface_t *get_iface_by_addr(uint8_t addr, uint8_t iface_num)
{
    uac2_iface_t *iface = NULL;
    UAC2_ENTER_CRITICAL();
    STAILQ_FOREACH(iface, &s_uac2_driver->ifaces_tailq, tailq_entry) {
        if (iface->parent && iface->parent->addr == addr && iface->iface_num == iface_num) {
            UAC2_EXIT_CRITICAL();
            return iface;
        }
    }
    UAC2_EXIT_CRITICAL();
    return NULL;
}

static inline bool is_interface_in_list(uac2_iface_t *target)
{
    uac2_iface_t *iface = NULL;
    UAC2_ENTER_CRITICAL();
    STAILQ_FOREACH(iface, &s_uac2_driver->ifaces_tailq, tailq_entry) {
        if (iface == target) {
            UAC2_EXIT_CRITICAL();
            return true;
        }
    }
    UAC2_EXIT_CRITICAL();
    return false;
}

static uac2_iface_t *get_iface_by_handle(uac2_host_device_handle_t handle)
{
    if (!handle || !s_uac2_driver) return NULL;
    uac2_iface_t *iface = (uac2_iface_t *)handle;
    if (!is_interface_in_list(iface)) return NULL;
    return iface;
}

// ── API mutex helpers ─────────────────────────────────────────────

#define UAC2_API_MUTEX_TIMEOUT_MS  5000

static inline esp_err_t api_lock(uac2_iface_t *iface) {
    if (xSemaphoreTake(iface->api_mutex, pdMS_TO_TICKS(UAC2_API_MUTEX_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "API mutex timeout");
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static inline void api_unlock(uac2_iface_t *iface) {
    xSemaphoreGive(iface->api_mutex);
}

// ── Control transfer helpers ──────────────────────────────────────

static void ctrl_xfer_cb(usb_transfer_t *xfer)
{
    uac2_device_t *dev = (uac2_device_t *)xfer->context;
    if (atomic_load(&dev->ctrl_xfer_gen) != atomic_load(&dev->ctrl_xfer_submitted_gen)) {
        return;  // stale callback after timeout
    }
    xSemaphoreGive(dev->ctrl_xfer_done);
}

/**
 * Send a class-specific control request and wait for completion.
 * On success, ctrl_mutex is intentionally left HELD so the caller
 * can read response data. Caller MUST release ctrl_mutex after.
 */
static esp_err_t ctrl_request(uac2_device_t *dev,
                              uint8_t bm_request_type,
                              uint8_t b_request,
                              uint16_t w_value,
                              uint16_t w_index,
                              uint16_t w_length,
                              const uint8_t *data_out)
{
    if (!dev || !dev->ctrl_xfer) return ESP_ERR_INVALID_ARG;

    if (xSemaphoreTake(dev->ctrl_mutex, pdMS_TO_TICKS(UAC2_CTRL_XFER_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Control transfer mutex timeout");
        return ESP_ERR_TIMEOUT;
    }

    usb_transfer_t *xfer = dev->ctrl_xfer;
    usb_setup_packet_t *setup = (usb_setup_packet_t *)xfer->data_buffer;
    setup->bmRequestType = bm_request_type;
    setup->bRequest = b_request;
    setup->wValue = w_value;
    setup->wIndex = w_index;
    setup->wLength = w_length;

    if (w_length > UAC2_CTRL_XFER_MAX_SIZE) {
        ESP_LOGE(TAG, "Control data too large: %d > %d", w_length, UAC2_CTRL_XFER_MAX_SIZE);
        xSemaphoreGive(dev->ctrl_mutex);
        return ESP_ERR_INVALID_SIZE;
    }

    if (data_out && w_length > 0) {
        memcpy(xfer->data_buffer + sizeof(usb_setup_packet_t), data_out, w_length);
    }

    xfer->num_bytes = sizeof(usb_setup_packet_t) + w_length;
    xfer->device_handle = dev->dev_hdl;
    xfer->bEndpointAddress = 0;
    xfer->callback = ctrl_xfer_cb;
    xfer->context = dev;
    xfer->timeout_ms = UAC2_CTRL_XFER_TIMEOUT_MS;

    atomic_store(&dev->ctrl_xfer_submitted_gen, atomic_load(&dev->ctrl_xfer_gen));

    esp_err_t err = usb_host_transfer_submit_control(s_uac2_driver->client_handle, xfer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Control submit failed: %s", esp_err_to_name(err));
        xSemaphoreGive(dev->ctrl_mutex);
        return err;
    }

    if (xSemaphoreTake(dev->ctrl_xfer_done, pdMS_TO_TICKS(UAC2_CTRL_XFER_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "Control transfer timeout");
        atomic_fetch_add(&dev->ctrl_xfer_gen, 1);
        esp_err_t halt_err = usb_host_endpoint_halt(dev->dev_hdl, 0);
        if (halt_err == ESP_OK) {
            usb_host_endpoint_flush(dev->dev_hdl, 0);
            usb_host_endpoint_clear(dev->dev_hdl, 0);
        }
        xSemaphoreTake(dev->ctrl_xfer_done, 0);
        xSemaphoreGive(dev->ctrl_mutex);
        return ESP_ERR_TIMEOUT;
    }

    if (xfer->status != USB_TRANSFER_STATUS_COMPLETED) {
        ESP_LOGE(TAG, "Control transfer failed, status=%d", xfer->status);
        xSemaphoreGive(dev->ctrl_mutex);
        return ESP_FAIL;
    }

    int data_len = xfer->actual_num_bytes - (int)sizeof(usb_setup_packet_t);
    if (data_len > 0) {
        ESP_LOG_BUFFER_HEXDUMP(TAG, xfer->data_buffer + sizeof(usb_setup_packet_t),
                               data_len, ESP_LOG_DEBUG);
    }

    return ESP_OK;  // mutex held — caller releases after reading data
}

static esp_err_t ctrl_request_no_data(uac2_device_t *dev,
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

static int ctrl_get_actual_len(uac2_device_t *dev)
{
    int total = dev->ctrl_xfer->actual_num_bytes;
    int data_len = total - (int)sizeof(usb_setup_packet_t);
    return data_len > 0 ? data_len : 0;
}

static esp_err_t ctrl_set_cur(uac2_device_t *dev,
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

static esp_err_t ctrl_get_cur(uac2_device_t *dev,
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

static esp_err_t ctrl_get_range(uac2_device_t *dev,
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

// ── Isochronous transfer callbacks ────────────────────────────────

static void stream_tx_xfer_done(usb_transfer_t *xfer)
{
    uac2_iface_t *iface = (uac2_iface_t *)xfer->context;
    if (!iface) return;

    portENTER_CRITICAL(&iface->state_lock);
    bool active = iface->state == UAC2_IFACE_STATE_ACTIVE;
    portEXIT_CRITICAL(&iface->state_lock);

    if (!active) {
        atomic_fetch_sub(&iface->urbs_in_flight, 1);
        return;
    }

    switch (xfer->status) {
    case USB_TRANSFER_STATUS_COMPLETED:
        atomic_store(&iface->consecutive_errors, 0);
        stream_tx_xfer_submit(iface, xfer);
        break;

    case USB_TRANSFER_STATUS_NO_DEVICE:
    case USB_TRANSFER_STATUS_CANCELED:
        atomic_fetch_sub(&iface->urbs_in_flight, 1);
        if (xfer->status == USB_TRANSFER_STATUS_NO_DEVICE &&
            iface->user_cb && !atomic_exchange(&iface->disconnect_fired, true)) {
            iface->user_cb(iface, UAC2_HOST_DEVICE_EVENT_DISCONNECTED, iface->user_cb_arg);
        }
        return;

    default: {
        int errs = atomic_fetch_add(&iface->consecutive_errors, 1) + 1;
        ESP_LOGW(TAG, "TX transfer error, status=%d (consecutive: %d)", xfer->status, errs);
        if (errs >= UAC2_MAX_CONSECUTIVE_ERRORS) {
            ESP_LOGE(TAG, "TX: %d consecutive errors, stream dead", errs);
            atomic_fetch_sub(&iface->urbs_in_flight, 1);
            portENTER_CRITICAL(&iface->state_lock);
            iface->state = UAC2_IFACE_STATE_ERROR;
            portEXIT_CRITICAL(&iface->state_lock);
            if (iface->user_cb) {
                iface->user_cb(iface, UAC2_HOST_DEVICE_EVENT_STREAM_ERROR, iface->user_cb_arg);
            }
        } else {
            if (iface->user_cb) {
                iface->user_cb(iface, UAC2_HOST_DEVICE_EVENT_TRANSFER_ERROR, iface->user_cb_arg);
            }
            stream_tx_xfer_submit(iface, xfer);
        }
        break;
    }
    }
}

static void stream_tx_xfer_submit(uac2_iface_t *iface, usb_transfer_t *xfer)
{
    portENTER_CRITICAL(&iface->state_lock);
    bool active = iface->state == UAC2_IFACE_STATE_ACTIVE;
    portEXIT_CRITICAL(&iface->state_lock);
    if (!active) {
        atomic_fetch_sub(&iface->urbs_in_flight, 1);
        return;
    }

    size_t item_size = 0;

    // Feedback-based adaptive packet sizing
    uint16_t pkt_size;
    uint32_t fb = atomic_load(&iface->fb_value);
    if (fb > 0) {
        uint16_t nominal_samples = (uint16_t)(fb >> 16);
        uint16_t fraction = (uint16_t)(fb & 0xFFFF);
        if (nominal_samples == 0 || nominal_samples > 1000) {
            pkt_size = iface->packet_size;
            goto send_packet;
        }
        iface->fb_accumulator += fraction;
        uint16_t extra = (uint16_t)(iface->fb_accumulator >> 16);
        iface->fb_accumulator &= 0xFFFF;
        uint16_t samples_this_frame = nominal_samples + extra;
        uint32_t raw_pkt_size = (uint32_t)samples_this_frame * iface->channels * iface->sub_slot_size;
        pkt_size = (raw_pkt_size > iface->ep_mps) ? iface->ep_mps : (uint16_t)raw_pkt_size;
    } else {
        pkt_size = iface->packet_size;
    }
send_packet:

    void *data = xRingbufferReceiveUpTo(iface->ringbuf, &item_size, 0, pkt_size);

    if (data && item_size > 0) {
        memcpy(xfer->data_buffer, data, item_size);
        vRingbufferReturnItem(iface->ringbuf, data);
        if (item_size < pkt_size) {
            memset(xfer->data_buffer + item_size, 0, pkt_size - item_size);
        }
        size_t rb_used = iface->ringbuf_size - xRingbufferGetCurFreeSize(iface->ringbuf);
        if (rb_used < iface->ringbuf_threshold && iface->user_cb && !atomic_load(&iface->tx_done_pending)) {
            atomic_store(&iface->tx_done_pending, true);
            iface->user_cb(iface, UAC2_HOST_DEVICE_EVENT_TX_DONE, iface->user_cb_arg);
        }
    } else {
        memset(xfer->data_buffer, 0, pkt_size);
        if (iface->user_cb && !atomic_load(&iface->tx_done_pending)) {
            atomic_store(&iface->tx_done_pending, true);
            iface->user_cb(iface, UAC2_HOST_DEVICE_EVENT_TX_DONE, iface->user_cb_arg);
        }
    }

    xfer->num_bytes = pkt_size;
    xfer->isoc_packet_desc[0].num_bytes = pkt_size;

    esp_err_t err = usb_host_transfer_submit(xfer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TX submit failed: %s", esp_err_to_name(err));
        atomic_fetch_sub(&iface->urbs_in_flight, 1);
    }
}

static void stream_rx_xfer_done(usb_transfer_t *xfer)
{
    uac2_iface_t *iface = (uac2_iface_t *)xfer->context;
    if (!iface) return;

    portENTER_CRITICAL(&iface->state_lock);
    bool active = iface->state == UAC2_IFACE_STATE_ACTIVE;
    portEXIT_CRITICAL(&iface->state_lock);

    if (!active) {
        atomic_fetch_sub(&iface->urbs_in_flight, 1);
        return;
    }

    switch (xfer->status) {
    case USB_TRANSFER_STATUS_COMPLETED:
        atomic_store(&iface->consecutive_errors, 0);
        for (int i = 0; i < xfer->num_isoc_packets; i++) {
            usb_isoc_packet_desc_t *pkt = &xfer->isoc_packet_desc[i];
            if (pkt->status == USB_TRANSFER_STATUS_COMPLETED && pkt->actual_num_bytes > 0) {
                uint8_t *pkt_data = xfer->data_buffer + (i * iface->ep_mps);
                BaseType_t ok = xRingbufferSend(iface->ringbuf, pkt_data,
                                                pkt->actual_num_bytes, 0);
                if (ok != pdTRUE) {
                    ESP_LOGW(TAG, "RX ringbuf overflow, dropped %d bytes",
                             pkt->actual_num_bytes);
                }
            }
        }
        {
            size_t rb_used = iface->ringbuf_size - xRingbufferGetCurFreeSize(iface->ringbuf);
            if (rb_used >= iface->ringbuf_threshold && iface->user_cb) {
                iface->user_cb(iface, UAC2_HOST_DEVICE_EVENT_RX_DONE, iface->user_cb_arg);
            }
        }
        xfer->num_bytes = iface->ep_mps * xfer->num_isoc_packets;
        for (int i = 0; i < xfer->num_isoc_packets; i++) {
            xfer->isoc_packet_desc[i].num_bytes = iface->ep_mps;
        }
        {
            esp_err_t sub_err = usb_host_transfer_submit(xfer);
            if (sub_err != ESP_OK) {
                ESP_LOGW(TAG, "RX resubmit failed: %s", esp_err_to_name(sub_err));
                atomic_fetch_sub(&iface->urbs_in_flight, 1);
            }
        }
        break;

    case USB_TRANSFER_STATUS_NO_DEVICE:
    case USB_TRANSFER_STATUS_CANCELED:
        atomic_fetch_sub(&iface->urbs_in_flight, 1);
        if (xfer->status == USB_TRANSFER_STATUS_NO_DEVICE &&
            iface->user_cb && !atomic_exchange(&iface->disconnect_fired, true)) {
            iface->user_cb(iface, UAC2_HOST_DEVICE_EVENT_DISCONNECTED, iface->user_cb_arg);
        }
        return;

    default: {
        int errs = atomic_fetch_add(&iface->consecutive_errors, 1) + 1;
        ESP_LOGW(TAG, "RX transfer error, status=%d (consecutive: %d)", xfer->status, errs);
        if (errs >= UAC2_MAX_CONSECUTIVE_ERRORS) {
            ESP_LOGE(TAG, "RX: %d consecutive errors, stream dead", errs);
            atomic_fetch_sub(&iface->urbs_in_flight, 1);
            portENTER_CRITICAL(&iface->state_lock);
            iface->state = UAC2_IFACE_STATE_ERROR;
            portEXIT_CRITICAL(&iface->state_lock);
            if (iface->user_cb) {
                iface->user_cb(iface, UAC2_HOST_DEVICE_EVENT_STREAM_ERROR, iface->user_cb_arg);
            }
        } else {
            esp_err_t sub_err = usb_host_transfer_submit(xfer);
            if (sub_err != ESP_OK) {
                ESP_LOGW(TAG, "RX resubmit failed: %s", esp_err_to_name(sub_err));
                atomic_fetch_sub(&iface->urbs_in_flight, 1);
            }
        }
        break;
    }
    }
}

static void feedback_xfer_done(usb_transfer_t *xfer)
{
    uac2_iface_t *iface = (uac2_iface_t *)xfer->context;
    if (!iface) return;

    portENTER_CRITICAL(&iface->state_lock);
    bool active = iface->state == UAC2_IFACE_STATE_ACTIVE;
    portEXIT_CRITICAL(&iface->state_lock);

    if (!active) {
        atomic_fetch_sub(&iface->urbs_in_flight, 1);
        return;
    }

    if (xfer->status != USB_TRANSFER_STATUS_COMPLETED) {
        if (xfer->status == USB_TRANSFER_STATUS_NO_DEVICE ||
            xfer->status == USB_TRANSFER_STATUS_CANCELED) {
            atomic_fetch_sub(&iface->urbs_in_flight, 1);
            if (xfer->status == USB_TRANSFER_STATUS_NO_DEVICE &&
                iface->user_cb && !atomic_exchange(&iface->disconnect_fired, true)) {
                iface->user_cb(iface, UAC2_HOST_DEVICE_EVENT_DISCONNECTED, iface->user_cb_arg);
            }
            return;
        }
        int errs = atomic_fetch_add(&iface->consecutive_errors, 1) + 1;
        ESP_LOGD(TAG, "Feedback transfer error, status=%d (consecutive: %d)", xfer->status, errs);
        if (errs >= UAC2_MAX_CONSECUTIVE_ERRORS) {
            ESP_LOGE(TAG, "Feedback: %d consecutive errors, stream dead", errs);
            atomic_fetch_sub(&iface->urbs_in_flight, 1);
            portENTER_CRITICAL(&iface->state_lock);
            iface->state = UAC2_IFACE_STATE_ERROR;
            portEXIT_CRITICAL(&iface->state_lock);
            if (iface->user_cb) {
                iface->user_cb(iface, UAC2_HOST_DEVICE_EVENT_STREAM_ERROR, iface->user_cb_arg);
            }
            return;
        }
    }

    if (xfer->status == USB_TRANSFER_STATUS_COMPLETED) {
        atomic_store(&iface->consecutive_errors, 0);
        int actual = xfer->isoc_packet_desc[0].actual_num_bytes;
        if (actual > 4) actual = 4;  // clamp to buffer size

        if (actual == 4) {
            uint32_t raw = xfer->data_buffer[0]
                         | ((uint32_t)xfer->data_buffer[1] << 8)
                         | ((uint32_t)xfer->data_buffer[2] << 16)
                         | ((uint32_t)xfer->data_buffer[3] << 24);
            atomic_store(&iface->fb_value, raw);
        } else if (actual == 3) {
            uint32_t raw = (uint32_t)xfer->data_buffer[0]
                         | ((uint32_t)xfer->data_buffer[1] << 8)
                         | ((uint32_t)xfer->data_buffer[2] << 16);
            atomic_store(&iface->fb_value, raw << 2);
        }

        {
            uint32_t fb_log = atomic_load(&iface->fb_value);
            ESP_LOGD(TAG, "Feedback: %" PRIu32 ".%04" PRIu32 " Hz",
                     (uint32_t)(fb_log >> 16),
                     (uint32_t)((fb_log & 0xFFFF) * 10000 / 65536));
        }
    }

    // Resubmit — recheck state under spinlock
    portENTER_CRITICAL(&iface->state_lock);
    bool still_active = iface->state == UAC2_IFACE_STATE_ACTIVE;
    portEXIT_CRITICAL(&iface->state_lock);
    if (still_active) {
        xfer->isoc_packet_desc[0].num_bytes = xfer->data_buffer_size;
        xfer->num_bytes = xfer->data_buffer_size;
        esp_err_t sub_err = usb_host_transfer_submit(xfer);
        if (sub_err != ESP_OK) {
            ESP_LOGW(TAG, "Feedback resubmit failed: %s", esp_err_to_name(sub_err));
            atomic_fetch_sub(&iface->urbs_in_flight, 1);
        }
    } else {
        atomic_fetch_sub(&iface->urbs_in_flight, 1);
    }
}

// ── Stream resource management ────────────────────────────────────

static const uac2_as_iface_t *find_matching_as_iface(
    const uac2_device_info_t *info,
    uac2_stream_dir_t dir,
    const uac2_host_stream_config_t *config,
    uint8_t iface_num)
{
    const uac2_as_iface_t *best = NULL;

    for (int i = 0; i < info->num_as_ifaces; i++) {
        const uac2_as_iface_t *as = &info->as_ifaces[i];

        // Must match this interface number
        if (as->interface_num != iface_num) continue;

        // Match direction
        bool is_out = (as->ep_addr & 0x80) == 0;
        if (dir == UAC2_STREAM_TX && !is_out) continue;
        if (dir == UAC2_STREAM_RX && is_out) continue;

        // Match format
        if (config->bit_resolution && as->bit_resolution != config->bit_resolution) continue;
        if (config->channels && as->nr_channels != config->channels) continue;

        if (!best || as->bit_resolution > best->bit_resolution) {
            best = as;
        }
    }

    return best;
}

static uint16_t calc_packet_size(uint32_t sample_rate, uint8_t channels,
                                 uint8_t sub_slot_size)
{
    uint32_t samples_per_frame = (sample_rate + 999) / 1000;
    uint32_t result = samples_per_frame * channels * sub_slot_size;
    if (result > UINT16_MAX) {
        ESP_LOGE(TAG, "Packet size overflow: %" PRIu32, result);
        return 0;
    }
    return (uint16_t)result;
}

static esp_err_t stream_resources_alloc(uac2_iface_t *iface,
                                        const uac2_as_iface_t *as,
                                        const uac2_host_stream_config_t *config)
{
    iface->alt_setting = as->alt_setting;
    iface->sample_rate = config->sample_freq;
    iface->channels = as->nr_channels;
    iface->bit_resolution = as->bit_resolution;
    iface->sub_slot_size = as->sub_slot_size;
    iface->ep_addr = as->ep_addr;
    iface->ep_mps = as->ep_max_packet_size;
    iface->fb_ep_addr = as->fb_ep_addr;
    // Use atomic_store (not atomic_init) because the iface struct persists
    // across start/stop cycles. atomic_init on an already-initialized atomic is UB.
    atomic_store(&iface->fb_value, 0);
    iface->fb_accumulator = 0;
    atomic_store(&iface->urbs_in_flight, 0);
    atomic_store(&iface->consecutive_errors, 0);
    atomic_store(&iface->user_task_blocked, false);
    atomic_store(&iface->first_frame_us, 0);
    atomic_store(&iface->tx_done_pending, false);

    iface->user_task_done = xSemaphoreCreateBinary();
    if (!iface->user_task_done) return ESP_ERR_NO_MEM;

    iface->packet_size = calc_packet_size(config->sample_freq,
                                          as->nr_channels, as->sub_slot_size);
    if (iface->packet_size == 0 || iface->packet_size > iface->ep_mps) {
        ESP_LOGE(TAG, "Packet size %d invalid (MPS %d)", iface->packet_size, iface->ep_mps);
        vSemaphoreDelete(iface->user_task_done);
        iface->user_task_done = NULL;
        return ESP_ERR_INVALID_SIZE;
    }

    // Ring buffer
    uint32_t rb_size = iface->cfg_buffer_size;
    if (rb_size == 0) {
        rb_size = iface->packet_size * 100;  // ~100ms
    }
    iface->ringbuf_size = rb_size;
    iface->ringbuf_threshold = iface->cfg_buffer_threshold;
    if (iface->ringbuf_threshold == 0) {
        iface->ringbuf_threshold = rb_size / 2;
    }

    iface->ringbuf = xRingbufferCreate(rb_size, RINGBUF_TYPE_BYTEBUF);
    if (!iface->ringbuf) {
        vSemaphoreDelete(iface->user_task_done);
        iface->user_task_done = NULL;
        return ESP_ERR_NO_MEM;
    }

    // Allocate isochronous URBs
    int num_pkts = UAC2_NUM_PACKETS_PER_URB;
    size_t xfer_size = iface->ep_mps * num_pkts;

    for (int i = 0; i < UAC2_NUM_ISOC_URBS; i++) {
        esp_err_t err = usb_host_transfer_alloc(xfer_size, num_pkts, &iface->xfer[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to allocate URB %d: %s", i, esp_err_to_name(err));
            for (int j = 0; j < i; j++) {
                usb_host_transfer_free(iface->xfer[j]);
                iface->xfer[j] = NULL;
            }
            vRingbufferDelete(iface->ringbuf);
            iface->ringbuf = NULL;
            vSemaphoreDelete(iface->user_task_done);
            iface->user_task_done = NULL;
            return err;
        }
        iface->xfer_count = i + 1;
    }

    // Feedback URB
    if (iface->fb_ep_addr != 0 && iface->dir == UAC2_STREAM_TX) {
        esp_err_t err = usb_host_transfer_alloc(4, 1, &iface->fb_xfer);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to allocate feedback URB: %s", esp_err_to_name(err));
            iface->fb_ep_addr = 0;
        }
    }

    return ESP_OK;
}

static void stream_resources_free(uac2_iface_t *iface)
{
    for (int i = 0; i < iface->xfer_count; i++) {
        if (iface->xfer[i]) {
            usb_host_transfer_free(iface->xfer[i]);
            iface->xfer[i] = NULL;
        }
    }
    iface->xfer_count = 0;

    if (iface->fb_xfer) {
        usb_host_transfer_free(iface->fb_xfer);
        iface->fb_xfer = NULL;
    }

    if (iface->ringbuf) {
        // Unblock any task waiting on the ringbuffer
        if (iface->dir == UAC2_STREAM_TX) {
            size_t item_size;
            void *item;
            while ((item = xRingbufferReceiveUpTo(iface->ringbuf, &item_size, 0,
                                                   iface->ringbuf_size)) != NULL) {
                vRingbufferReturnItem(iface->ringbuf, item);
            }
        } else {
            uint8_t dummy = 0;
            xRingbufferSend(iface->ringbuf, &dummy, 1, 0);
        }
        if (atomic_load(&iface->user_task_blocked)) {
            xSemaphoreTake(iface->user_task_done, pdMS_TO_TICKS(200));
        }
        vRingbufferDelete(iface->ringbuf);
        iface->ringbuf = NULL;
    }

    if (iface->user_task_done) {
        vSemaphoreDelete(iface->user_task_done);
        iface->user_task_done = NULL;
    }
}

static esp_err_t stream_submit_urbs(uac2_iface_t *iface)
{
    uac2_device_t *dev = iface->parent;
    int num_pkts = UAC2_NUM_PACKETS_PER_URB;

    portENTER_CRITICAL(&iface->state_lock);
    iface->state = UAC2_IFACE_STATE_ACTIVE;
    portEXIT_CRITICAL(&iface->state_lock);

    for (int i = 0; i < iface->xfer_count; i++) {
        usb_transfer_t *xfer = iface->xfer[i];
        xfer->device_handle = dev->dev_hdl;
        xfer->bEndpointAddress = iface->ep_addr;
        xfer->context = iface;
        xfer->timeout_ms = 0;

        if (iface->dir == UAC2_STREAM_TX) {
            xfer->callback = stream_tx_xfer_done;
            memset(xfer->data_buffer, 0, iface->packet_size * num_pkts);
            xfer->num_bytes = iface->packet_size * num_pkts;
            for (int j = 0; j < num_pkts; j++) {
                xfer->isoc_packet_desc[j].num_bytes = iface->packet_size;
            }
        } else {
            xfer->callback = stream_rx_xfer_done;
            xfer->num_bytes = iface->ep_mps * num_pkts;
            for (int j = 0; j < num_pkts; j++) {
                xfer->isoc_packet_desc[j].num_bytes = iface->ep_mps;
            }
        }

        atomic_fetch_add(&iface->urbs_in_flight, 1);
        esp_err_t err = usb_host_transfer_submit(xfer);
        if (err != ESP_OK) {
            atomic_fetch_sub(&iface->urbs_in_flight, 1);
            ESP_LOGE(TAG, "Failed to submit URB %d: %s", i, esp_err_to_name(err));
            return err;
        }
        if (i == 0 && atomic_load(&iface->first_frame_us) == 0) {
            atomic_store(&iface->first_frame_us, esp_timer_get_time());
        }
    }

    // Submit feedback URB
    if (iface->fb_xfer && iface->fb_ep_addr != 0) {
        usb_transfer_t *fb = iface->fb_xfer;
        fb->device_handle = dev->dev_hdl;
        fb->bEndpointAddress = iface->fb_ep_addr;
        fb->callback = feedback_xfer_done;
        fb->context = iface;
        fb->timeout_ms = 0;
        fb->num_bytes = 4;
        fb->isoc_packet_desc[0].num_bytes = 4;

        atomic_fetch_add(&iface->urbs_in_flight, 1);
        esp_err_t err = usb_host_transfer_submit(fb);
        if (err != ESP_OK) {
            atomic_fetch_sub(&iface->urbs_in_flight, 1);
            ESP_LOGW(TAG, "Feedback URB submit failed: %s", esp_err_to_name(err));
        }
    }

    return ESP_OK;
}

// ── Clock helpers ─────────────────────────────────────────────────

static uint8_t resolve_clock_source(const uac2_device_info_t *info)
{
    if (info->num_clock_sources > 0) {
        return info->clock_sources[0].clock_id;
    }
    if (info->num_clock_selectors > 0 && info->clock_selectors[0].nr_pins > 0) {
        return info->clock_selectors[0].source_ids[0];
    }
    ESP_LOGW(TAG, "No clock source found in descriptors");
    return 0;
}

static esp_err_t set_sample_rate_internal(uac2_device_t *dev, uint32_t sample_rate)
{
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

static void validate_sample_rate(uac2_device_t *dev, uint32_t sample_rate)
{
    if (dev->clock_source_id == 0) return;

    uint8_t buf[2 + UAC2_MAX_SAMPLE_RATE_RANGES * 12];
    memset(buf, 0, sizeof(buf));
    esp_err_t err = ctrl_get_range(dev, dev->clock_source_id,
                                   UAC2_CS_SAM_FREQ_CONTROL, 0, buf, sizeof(buf));
    if (err != ESP_OK) return;

    uint16_t count = buf[0] | (buf[1] << 8);
    if (count > UAC2_MAX_SAMPLE_RATE_RANGES) count = UAC2_MAX_SAMPLE_RATE_RANGES;
    if (count == 0) return;

    for (int i = 0; i < count; i++) {
        const uint8_t *p = buf + 2 + (i * 12);
        uint32_t min = (uint32_t)p[0] | ((uint32_t)p[1] << 8)
                     | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
        uint32_t max = (uint32_t)p[4] | ((uint32_t)p[5] << 8)
                     | ((uint32_t)p[6] << 16) | ((uint32_t)p[7] << 24);
        if (min == max) {
            if (sample_rate == min) return;
        } else {
            if (sample_rate >= min && sample_rate <= max) return;
        }
    }
    ESP_LOGW(TAG, "Sample rate %" PRIu32 " Hz not in device's advertised ranges "
             "(continuing — some devices accept non-advertised rates)", sample_rate);
}

// ── String conversion ─────────────────────────────────────────────

static void usb_string_to_ascii(const usb_str_desc_t *str_desc, char *out, size_t out_size)
{
    if (!str_desc || !out || out_size == 0) return;
    out[0] = '\0';
    if (str_desc->bLength < 2) return;
    int num_chars = (str_desc->bLength - 2) / 2;
    if (num_chars <= 0) return;
    if ((size_t)num_chars >= out_size) num_chars = out_size - 1;
    for (int i = 0; i < num_chars; i++) {
        uint16_t wchar = str_desc->wData[i];
        out[i] = (wchar < 128) ? (char)wchar : '?';
    }
    out[num_chars] = '\0';
}

// ── Device discovery (internal) ───────────────────────────────────

static void driver_user_callback(uint8_t addr, uint8_t iface_num,
                                 uac2_host_driver_event_t event)
{
    if (s_uac2_driver && s_uac2_driver->user_cb) {
        s_uac2_driver->user_cb(addr, iface_num, event, s_uac2_driver->user_arg);
    }
}

static esp_err_t uac2_host_device_connected(uint8_t addr)
{
    bool is_uac2 = false;
    usb_device_handle_t dev_hdl;
    const usb_config_desc_t *config_desc = NULL;

    // Temporarily open device to inspect descriptors
    if (usb_host_device_open(s_uac2_driver->client_handle, addr, &dev_hdl) != ESP_OK) {
        return ESP_FAIL;
    }

    if (usb_host_get_active_config_descriptor(dev_hdl, &config_desc) == ESP_OK) {
        uac2_device_info_t info;
        is_uac2 = uac2_parse_config_descriptor(
            (const uint8_t *)config_desc, config_desc->wTotalLength, &info);

        if (is_uac2) {
            // Collect interfaces to notify BEFORE closing the device.
            // Must close the inspection handle first so device_create can re-open.
            uint8_t notify_ifaces[UAC2_MAX_AS_INTERFACES];
            bool notify_is_out[UAC2_MAX_AS_INTERFACES];
            int notify_count = 0;

            for (int i = 0; i < info.num_as_ifaces; i++) {
                const uac2_as_iface_t *as = &info.as_ifaces[i];

                bool already = false;
                for (int j = 0; j < notify_count; j++) {
                    if (notify_ifaces[j] == as->interface_num) { already = true; break; }
                }
                if (already) continue;
                if (notify_count < UAC2_MAX_AS_INTERFACES) {
                    notify_ifaces[notify_count] = as->interface_num;
                    notify_is_out[notify_count] = (as->ep_addr & 0x80) == 0;
                    notify_count++;
                }
            }

            // Close device BEFORE firing callbacks — the callback's device_open
            // needs this client's handle free to re-open the device.
            usb_host_device_close(s_uac2_driver->client_handle, dev_hdl);
            dev_hdl = NULL;

            // Now fire callbacks
            for (int i = 0; i < notify_count; i++) {
                uac2_host_driver_event_t event = notify_is_out[i]
                    ? UAC2_HOST_DRIVER_EVENT_TX_CONNECTED
                    : UAC2_HOST_DRIVER_EVENT_RX_CONNECTED;
                ESP_LOGI(TAG, "UAC2 %s interface found: addr=%d iface=%d",
                         notify_is_out[i] ? "TX" : "RX", addr, notify_ifaces[i]);
                driver_user_callback(addr, notify_ifaces[i], event);
            }
        } else {
            ESP_LOGD(TAG, "USB device addr %d is not UAC2", addr);
        }
    }

    if (dev_hdl) {
        usb_host_device_close(s_uac2_driver->client_handle, dev_hdl);
    }
    return is_uac2 ? ESP_OK : ESP_ERR_NOT_FOUND;
}

static esp_err_t uac2_host_device_disconnected(usb_device_handle_t dev_hdl)
{
    uac2_device_t *dev = get_device_by_handle(dev_hdl);
    if (!dev) return ESP_OK;  // Not a UAC2 device we track

    // Mark device as gone — stream_stop_internal will skip SET_INTERFACE and endpoint ops
    atomic_store(&dev->gone, true);

    // Collect interfaces to notify (don't modify the list during iteration)
    uac2_iface_t *to_notify[UAC2_MAX_AS_INTERFACES];
    int notify_count = 0;

    UAC2_ENTER_CRITICAL();
    uac2_iface_t *iface;
    STAILQ_FOREACH(iface, &s_uac2_driver->ifaces_tailq, tailq_entry) {
        if (iface->parent && iface->parent->addr == dev->addr) {
            if (notify_count < UAC2_MAX_AS_INTERFACES) {
                to_notify[notify_count++] = iface;
            }
        }
    }
    UAC2_EXIT_CRITICAL();

    // Process each interface outside the critical section
    for (int i = 0; i < notify_count; i++) {
        iface = to_notify[i];

        // Stop active stream — callbacks see IDLE and stop resubmitting
        portENTER_CRITICAL(&iface->state_lock);
        iface->state = UAC2_IFACE_STATE_IDLE;
        portEXIT_CRITICAL(&iface->state_lock);

        // Fire disconnect event exactly once (URB callbacks also race to fire this).
        // User MUST call uac2_host_device_close() from a separate task after receiving this.
        if (!atomic_exchange(&iface->disconnect_fired, true)) {
            if (iface->user_cb) {
                iface->user_cb(iface, UAC2_HOST_DEVICE_EVENT_DISCONNECTED, iface->user_cb_arg);
            } else {
                // No callback registered — log warning. User must still call device_close
                // to free resources. We cannot call it here because we're on the USB event task
                // and device_close blocks waiting for URB drain callbacks from this same task.
                ESP_LOGW(TAG, "Interface addr=%d iface=%d disconnected with no callback — "
                         "call uac2_host_device_close() to free resources", dev->addr, iface->iface_num);
            }
        }
    }

    return ESP_OK;
}

static void client_event_cb(const usb_host_client_event_msg_t *event, void *arg)
{
    if (event->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
        uac2_host_device_connected(event->new_dev.address);
    } else if (event->event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
        uac2_host_device_disconnected(event->dev_gone.dev_hdl);
    }
}

// ── Event handler task ────────────────────────────────────────────

static void event_handler_task(void *arg)
{
    ESP_LOGD(TAG, "UAC2 event handling start");
    while (uac2_host_handle_events(portMAX_DELAY) == ESP_OK) {
    }
    ESP_LOGD(TAG, "UAC2 event handling stop");
    vTaskDelete(NULL);
}

// ── Physical device management (internal) ─────────────────────────

static esp_err_t device_create(uint8_t addr, uac2_device_t **out_dev)
{
    usb_device_handle_t dev_hdl;
    esp_err_t err = usb_host_device_open(s_uac2_driver->client_handle, addr, &dev_hdl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open USB device addr %d: %s", addr, esp_err_to_name(err));
        return err;
    }

    const usb_config_desc_t *config_desc;
    err = usb_host_get_active_config_descriptor(dev_hdl, &config_desc);
    if (err != ESP_OK) {
        usb_host_device_close(s_uac2_driver->client_handle, dev_hdl);
        return err;
    }

    // Reject low-speed
    usb_device_info_t usb_info;
    err = usb_host_device_info(dev_hdl, &usb_info);
    if (err != ESP_OK) {
        usb_host_device_close(s_uac2_driver->client_handle, dev_hdl);
        return err;
    }
    if (usb_info.speed == USB_SPEED_LOW) {
        ESP_LOGE(TAG, "Low-speed devices do not support isochronous transfers");
        usb_host_device_close(s_uac2_driver->client_handle, dev_hdl);
        return ESP_ERR_NOT_SUPPORTED;
    }

    uac2_device_t *dev = heap_caps_calloc(1, sizeof(uac2_device_t), MALLOC_CAP_DEFAULT);
    if (!dev) {
        usb_host_device_close(s_uac2_driver->client_handle, dev_hdl);
        return ESP_ERR_NO_MEM;
    }

    // Parse descriptors
    bool is_uac2 = uac2_parse_config_descriptor(
        (const uint8_t *)config_desc, config_desc->wTotalLength, &dev->desc_info);
    if (!is_uac2) {
        heap_caps_free(dev);
        usb_host_device_close(s_uac2_driver->client_handle, dev_hdl);
        return ESP_ERR_NOT_SUPPORTED;
    }

    // Populate identification
    const usb_device_desc_t *dev_desc;
    err = usb_host_get_device_descriptor(dev_hdl, &dev_desc);
    if (err != ESP_OK) { goto fail; }
    dev->desc_info.vid = dev_desc->idVendor;
    dev->desc_info.pid = dev_desc->idProduct;
    usb_string_to_ascii(usb_info.str_desc_manufacturer, dev->desc_info.manufacturer,
                         sizeof(dev->desc_info.manufacturer));
    usb_string_to_ascii(usb_info.str_desc_product, dev->desc_info.product,
                         sizeof(dev->desc_info.product));
    usb_string_to_ascii(usb_info.str_desc_serial_num, dev->desc_info.serial,
                         sizeof(dev->desc_info.serial));

    dev->dev_hdl = dev_hdl;
    dev->addr = addr;
    dev->opened_cnt = 0;
    atomic_init(&dev->gone, false);
    dev->ac_iface_num = dev->desc_info.ac_iface_num;
    dev->clock_source_id = resolve_clock_source(&dev->desc_info);

    if (dev->desc_info.num_feature_units > 0) {
        const uac2_feature_unit_t *fu = &dev->desc_info.feature_units[0];
        dev->has_feature_unit = true;
        dev->feature_unit_id = fu->unit_id;
        dev->has_mute = fu->has_mute;
        dev->has_volume = fu->has_volume;
    }

    // Allocate control transfer
    err = usb_host_transfer_alloc(sizeof(usb_setup_packet_t) + UAC2_CTRL_XFER_MAX_SIZE,
                                  0, &dev->ctrl_xfer);
    if (err != ESP_OK) goto fail;

    dev->ctrl_xfer_done = xSemaphoreCreateBinary();
    if (!dev->ctrl_xfer_done) { err = ESP_ERR_NO_MEM; goto fail; }

    dev->ctrl_mutex = xSemaphoreCreateMutex();
    if (!dev->ctrl_mutex) { err = ESP_ERR_NO_MEM; goto fail; }

    atomic_init(&dev->ctrl_xfer_gen, 0);
    atomic_init(&dev->ctrl_xfer_submitted_gen, 0);

    // Cache volume range
    if (dev->has_feature_unit && dev->has_volume) {
        uint8_t buf[2 + 6];  // sized for exactly 1 range triplet (min+max+res)
        memset(buf, 0, sizeof(buf));
        esp_err_t vr_err = ctrl_get_range(dev, dev->feature_unit_id,
                                           UAC2_FU_VOLUME_CONTROL, 0, buf, sizeof(buf));
        if (vr_err == ESP_OK) {
            uint16_t count = buf[0] | (buf[1] << 8);
            if (count > 1) count = 1;  // buffer holds only 1 triplet
            if (count > 0) {
                dev->volume_min_db256 = (int16_t)(buf[2] | (buf[3] << 8));
                dev->volume_max_db256 = (int16_t)(buf[4] | (buf[5] << 8));
                dev->volume_res_db256 = (int16_t)(buf[6] | (buf[7] << 8));
                dev->volume_range_valid = true;
                ESP_LOGI(TAG, "Volume range: %.2f to %.2f dB (res %.4f dB)",
                         dev->volume_min_db256 / 256.0, dev->volume_max_db256 / 256.0,
                         dev->volume_res_db256 / 256.0);
            }
        }
    }

    // Add to driver list
    UAC2_ENTER_CRITICAL();
    STAILQ_INSERT_TAIL(&s_uac2_driver->devices_tailq, dev, tailq_entry);
    UAC2_EXIT_CRITICAL();

    ESP_LOGI(TAG, "UAC2 device opened: addr=%d VID=0x%04X PID=0x%04X \"%s\"",
             addr, dev->desc_info.vid, dev->desc_info.pid,
             dev->desc_info.product[0] ? dev->desc_info.product : "Unknown");

    *out_dev = dev;
    return ESP_OK;

fail:
    if (dev->ctrl_mutex) vSemaphoreDelete(dev->ctrl_mutex);
    if (dev->ctrl_xfer_done) vSemaphoreDelete(dev->ctrl_xfer_done);
    if (dev->ctrl_xfer) usb_host_transfer_free(dev->ctrl_xfer);
    heap_caps_free(dev);
    usb_host_device_close(s_uac2_driver->client_handle, dev_hdl);
    return err;
}

static void device_destroy(uac2_device_t *dev)
{
    UAC2_ENTER_CRITICAL();
    STAILQ_REMOVE(&s_uac2_driver->devices_tailq, dev, uac2_device, tailq_entry);
    UAC2_EXIT_CRITICAL();

    // Wait for any in-progress control request to complete.
    // After this, no new ctrl_request can arrive because all interfaces are closed
    // (callers hold api_mutex on an interface, and all interfaces are removed from the list).
    if (dev->ctrl_mutex) {
        xSemaphoreTake(dev->ctrl_mutex, pdMS_TO_TICKS(UAC2_CTRL_XFER_TIMEOUT_MS));
        xSemaphoreGive(dev->ctrl_mutex);
    }

    if (dev->ctrl_xfer) usb_host_transfer_free(dev->ctrl_xfer);
    if (dev->ctrl_xfer_done) vSemaphoreDelete(dev->ctrl_xfer_done);
    if (dev->ctrl_mutex) vSemaphoreDelete(dev->ctrl_mutex);

    usb_host_device_close(s_uac2_driver->client_handle, dev->dev_hdl);
    ESP_LOGI(TAG, "Physical device addr %d closed", dev->addr);
    heap_caps_free(dev);
}

// ── Public API: Driver lifecycle ──────────────────────────────────

esp_err_t uac2_host_install(const uac2_host_driver_config_t *config)
{
    ESP_RETURN_ON_FALSE(!s_uac2_driver, ESP_ERR_INVALID_STATE, TAG, "UAC2 driver already installed");
    ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, TAG, "Config is NULL");
    ESP_RETURN_ON_FALSE(config->callback, ESP_ERR_INVALID_ARG, TAG, "Callback is NULL");

    if (config->create_background_task) {
        ESP_RETURN_ON_FALSE(config->stack_size != 0, ESP_ERR_INVALID_ARG, TAG, "Wrong stack size");
        ESP_RETURN_ON_FALSE(config->task_priority != 0, ESP_ERR_INVALID_ARG, TAG, "Wrong task priority");
    }

    uac2_driver_t *driver = heap_caps_calloc(1, sizeof(uac2_driver_t), MALLOC_CAP_DEFAULT);
    ESP_RETURN_ON_FALSE(driver, ESP_ERR_NO_MEM, TAG, "Unable to allocate driver");

    driver->user_cb = config->callback;
    driver->user_arg = config->callback_arg;
    driver->end_client_event_handling = false;

    driver->all_events_handled = xSemaphoreCreateBinary();
    if (!driver->all_events_handled) {
        heap_caps_free(driver);
        return ESP_ERR_NO_MEM;
    }

    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .async.client_event_callback = client_event_cb,
        .async.callback_arg = NULL,
        .max_num_event_msg = 16,
    };
    esp_err_t err = usb_host_client_register(&client_config, &driver->client_handle);
    if (err != ESP_OK) {
        vSemaphoreDelete(driver->all_events_handled);
        heap_caps_free(driver);
        return err;
    }

    UAC2_ENTER_CRITICAL();
    s_uac2_driver = driver;
    STAILQ_INIT(&s_uac2_driver->devices_tailq);
    STAILQ_INIT(&s_uac2_driver->ifaces_tailq);
    UAC2_EXIT_CRITICAL();

    if (config->create_background_task) {
        BaseType_t task_created = xTaskCreatePinnedToCore(
            event_handler_task, "uac2_host", config->stack_size,
            NULL, config->task_priority, NULL, config->core_id);
        if (!task_created) {
            UAC2_ENTER_CRITICAL();
            s_uac2_driver = NULL;
            UAC2_EXIT_CRITICAL();
            usb_host_client_deregister(driver->client_handle);
            vSemaphoreDelete(driver->all_events_handled);
            heap_caps_free(driver);
            return ESP_ERR_NO_MEM;
        }
    }

    ESP_LOGI(TAG, "UAC2 Host driver installed, v%d.%d.%d",
             UAC2_HOST_VER_MAJOR, UAC2_HOST_VER_MINOR, UAC2_HOST_VER_PATCH);
    return ESP_OK;
}

esp_err_t uac2_host_uninstall(void)
{
    ESP_RETURN_ON_FALSE(s_uac2_driver, ESP_OK, TAG, "UAC2 driver not installed");

    UAC2_ENTER_CRITICAL();
    if (s_uac2_driver->end_client_event_handling) {
        UAC2_EXIT_CRITICAL();
        return ESP_ERR_INVALID_STATE;
    }
    if (!STAILQ_EMPTY(&s_uac2_driver->devices_tailq) ||
        !STAILQ_EMPTY(&s_uac2_driver->ifaces_tailq)) {
        UAC2_EXIT_CRITICAL();
        ESP_LOGE(TAG, "Cannot uninstall: devices/interfaces still open");
        return ESP_ERR_INVALID_STATE;
    }
    s_uac2_driver->end_client_event_handling = true;
    UAC2_EXIT_CRITICAL();

    if (s_uac2_driver->event_handling_started) {
        esp_err_t unblock_err = usb_host_client_unblock(s_uac2_driver->client_handle);
        if (unblock_err != ESP_OK) {
            ESP_LOGE(TAG, "client_unblock failed: %s", esp_err_to_name(unblock_err));
        }
        xSemaphoreTake(s_uac2_driver->all_events_handled, portMAX_DELAY);
    }
    vSemaphoreDelete(s_uac2_driver->all_events_handled);
    esp_err_t dereg_err = usb_host_client_deregister(s_uac2_driver->client_handle);
    if (dereg_err != ESP_OK) {
        ESP_LOGE(TAG, "client_deregister failed: %s", esp_err_to_name(dereg_err));
    }
    heap_caps_free(s_uac2_driver);
    s_uac2_driver = NULL;
    ESP_LOGI(TAG, "UAC2 Host driver uninstalled");
    return ESP_OK;
}

esp_err_t uac2_host_handle_events(TickType_t timeout)
{
    ESP_RETURN_ON_FALSE(s_uac2_driver, ESP_ERR_INVALID_STATE, TAG, "UAC2 driver not installed");
    s_uac2_driver->event_handling_started = true;
    esp_err_t ret = usb_host_client_handle_events(s_uac2_driver->client_handle, timeout);
    UAC2_ENTER_CRITICAL();
    if (s_uac2_driver->end_client_event_handling) {
        UAC2_EXIT_CRITICAL();
        xSemaphoreGive(s_uac2_driver->all_events_handled);
        return ESP_FAIL;
    }
    UAC2_EXIT_CRITICAL();
    return ret;
}

// ── Public API: Device management ─────────────────────────────────

esp_err_t uac2_host_device_open(const uac2_host_device_config_t *config,
                                uac2_host_device_handle_t *out_handle)
{
    ESP_RETURN_ON_FALSE(config && out_handle, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    ESP_RETURN_ON_FALSE(s_uac2_driver, ESP_ERR_INVALID_STATE, TAG, "UAC2 driver not installed");
    *out_handle = NULL;

    // Reject if already open — caller must close first
    uac2_iface_t *existing = get_iface_by_addr(config->addr, config->iface_num);
    if (existing) {
        ESP_LOGE(TAG, "Interface addr=%d iface=%d already open", config->addr, config->iface_num);
        return ESP_ERR_INVALID_STATE;
    }

    // Get or create physical device
    esp_err_t err;
    uac2_device_t *dev = get_device_by_addr(config->addr);
    if (!dev) {
        err = device_create(config->addr, &dev);
        if (err != ESP_OK) return err;
    }

    // Determine direction from descriptor info
    uac2_stream_dir_t dir = UAC2_STREAM_TX;
    bool found = false;
    for (int i = 0; i < dev->desc_info.num_as_ifaces; i++) {
        if (dev->desc_info.as_ifaces[i].interface_num == config->iface_num) {
            dir = (dev->desc_info.as_ifaces[i].ep_addr & 0x80) ? UAC2_STREAM_RX : UAC2_STREAM_TX;
            found = true;
            break;
        }
    }
    if (!found) {
        ESP_LOGE(TAG, "Interface %d not found in device descriptors", config->iface_num);
        UAC2_ENTER_CRITICAL();
        bool orphan = (dev->opened_cnt == 0);
        UAC2_EXIT_CRITICAL();
        if (orphan) device_destroy(dev);
        return ESP_ERR_NOT_FOUND;
    }

    // Create interface
    uac2_iface_t *iface = heap_caps_calloc(1, sizeof(uac2_iface_t), MALLOC_CAP_DEFAULT);
    if (!iface) {
        UAC2_ENTER_CRITICAL();
        bool orphan = (dev->opened_cnt == 0);
        UAC2_EXIT_CRITICAL();
        if (orphan) device_destroy(dev);
        return ESP_ERR_NO_MEM;
    }

    iface->parent = dev;
    iface->dir = dir;
    iface->iface_num = config->iface_num;
    iface->state = UAC2_IFACE_STATE_IDLE;
    portMUX_INITIALIZE(&iface->state_lock);
    iface->user_cb = config->callback;
    iface->user_cb_arg = config->callback_arg;
    iface->cfg_buffer_size = config->buffer_size;
    iface->cfg_buffer_threshold = config->buffer_threshold;
    atomic_init(&iface->disconnect_fired, false);

    iface->api_mutex = xSemaphoreCreateMutex();
    if (!iface->api_mutex) {
        heap_caps_free(iface);
        UAC2_ENTER_CRITICAL();
        bool orphan = (dev->opened_cnt == 0);
        UAC2_EXIT_CRITICAL();
        if (orphan) device_destroy(dev);
        return ESP_ERR_NO_MEM;
    }

    // Add to list and increment refcount
    UAC2_ENTER_CRITICAL();
    STAILQ_INSERT_TAIL(&s_uac2_driver->ifaces_tailq, iface, tailq_entry);
    dev->opened_cnt++;
    UAC2_EXIT_CRITICAL();

    ESP_LOGI(TAG, "Opened %s interface: addr=%d iface=%d (device refs=%d)",
             dir == UAC2_STREAM_TX ? "TX" : "RX",
             config->addr, config->iface_num, dev->opened_cnt);

    *out_handle = iface;
    return ESP_OK;
}

esp_err_t uac2_host_device_close(uac2_host_device_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uac2_iface_t *iface = get_iface_by_handle(handle);
    ESP_RETURN_ON_FALSE(iface, ESP_ERR_INVALID_ARG, TAG, "Handle not in list (already closed?)");

    // Take api_mutex to serialize with any in-progress API calls
    esp_err_t lock_err = api_lock(iface);
    if (lock_err != ESP_OK) {
        ESP_LOGW(TAG, "device_close: api_mutex timeout, proceeding anyway");
    }

    // Re-validate after acquiring lock — a concurrent close may have freed this
    if (!is_interface_in_list(iface)) {
        if (lock_err == ESP_OK) api_unlock(iface);
        return ESP_ERR_INVALID_ARG;
    }

    // Stop active stream
    if (iface->state != UAC2_IFACE_STATE_IDLE) {
        stream_stop_internal(iface);
    }

    if (lock_err == ESP_OK) {
        api_unlock(iface);
    }

    uac2_device_t *dev = iface->parent;

    // Remove interface from list
    UAC2_ENTER_CRITICAL();
    STAILQ_REMOVE(&s_uac2_driver->ifaces_tailq, iface, uac2_interface, tailq_entry);
    dev->opened_cnt--;
    uint8_t remaining = dev->opened_cnt;
    UAC2_EXIT_CRITICAL();

    if (iface->api_mutex) vSemaphoreDelete(iface->api_mutex);
    ESP_LOGI(TAG, "Closed interface addr=%d iface=%d (device refs=%d)",
             dev->addr, iface->iface_num, remaining);
    heap_caps_free(iface);

    // If last interface, destroy the physical device
    if (remaining == 0) {
        device_destroy(dev);
    }

    return ESP_OK;
}

esp_err_t uac2_host_device_get_info(uac2_host_device_handle_t handle,
                                    uac2_device_info_t *info)
{
    ESP_RETURN_ON_FALSE(handle && info, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    uac2_iface_t *iface = get_iface_by_handle(handle);
    ESP_RETURN_ON_FALSE(iface, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    *info = iface->parent->desc_info;
    return ESP_OK;
}

esp_err_t uac2_host_get_device_alt_param(uac2_host_device_handle_t handle,
                                         uint8_t alt,
                                         uac2_host_dev_alt_param_t *param)
{
    ESP_RETURN_ON_FALSE(handle && param, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    uac2_iface_t *iface = get_iface_by_handle(handle);
    ESP_RETURN_ON_FALSE(iface, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(alt > 0, ESP_ERR_INVALID_ARG, TAG, "Alt setting must be >= 1");

    uac2_device_t *dev = iface->parent;
    for (int i = 0; i < dev->desc_info.num_as_ifaces; i++) {
        const uac2_as_iface_t *as = &dev->desc_info.as_ifaces[i];
        if (as->interface_num == iface->iface_num && as->alt_setting == alt) {
            param->alt_setting = as->alt_setting;
            param->channels = as->nr_channels;
            param->bit_resolution = as->bit_resolution;
            param->sub_slot_size = as->sub_slot_size;
            param->ep_max_packet_size = as->ep_max_packet_size;
            param->ep_addr = as->ep_addr;
            param->fb_ep_addr = as->fb_ep_addr;
            return ESP_OK;
        }
    }
    return ESP_ERR_NOT_FOUND;
}

// ── Public API: Clock control ─────────────────────────────────────

esp_err_t uac2_host_device_get_sample_rate(uac2_host_device_handle_t handle,
                                           uint32_t *sample_rate)
{
    ESP_RETURN_ON_FALSE(handle && sample_rate, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    uac2_iface_t *iface = get_iface_by_handle(handle);
    ESP_RETURN_ON_FALSE(iface, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uac2_device_t *dev = iface->parent;
    ESP_RETURN_ON_FALSE(dev->clock_source_id != 0, ESP_ERR_NOT_SUPPORTED, TAG, "No clock source");
    esp_err_t ret = api_lock(iface);
    if (ret != ESP_OK) return ret;

    uint8_t data[4] = {0};
    ret = ctrl_get_cur(dev, dev->clock_source_id, UAC2_CS_SAM_FREQ_CONTROL, 0, data, 4);
    if (ret == ESP_OK) {
        *sample_rate = (uint32_t)data[0] | ((uint32_t)data[1] << 8)
                     | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
        ESP_LOGI(TAG, "Current sample rate: %" PRIu32 " Hz", *sample_rate);
    }
    api_unlock(iface);
    return ret;
}

esp_err_t uac2_host_device_set_sample_rate(uac2_host_device_handle_t handle,
                                           uint32_t sample_rate)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uac2_iface_t *iface = get_iface_by_handle(handle);
    ESP_RETURN_ON_FALSE(iface, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uac2_device_t *dev = iface->parent;
    ESP_RETURN_ON_FALSE(dev->clock_source_id != 0, ESP_ERR_NOT_SUPPORTED, TAG, "No clock source");
    esp_err_t ret = api_lock(iface);
    if (ret != ESP_OK) return ret;
    ret = set_sample_rate_internal(dev, sample_rate);
    api_unlock(iface);
    return ret;
}

esp_err_t uac2_host_device_get_sample_rate_range(uac2_host_device_handle_t handle,
                                                 uac2_sample_rate_range_t *ranges,
                                                 uint8_t *num_ranges)
{
    ESP_RETURN_ON_FALSE(handle && ranges && num_ranges, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    uac2_iface_t *iface = get_iface_by_handle(handle);
    ESP_RETURN_ON_FALSE(iface, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uac2_device_t *dev = iface->parent;
    ESP_RETURN_ON_FALSE(dev->clock_source_id != 0, ESP_ERR_NOT_SUPPORTED, TAG, "No clock source");
    esp_err_t ret = api_lock(iface);
    if (ret != ESP_OK) return ret;

    uint8_t buf[2 + UAC2_MAX_SAMPLE_RATE_RANGES * 12];
    memset(buf, 0, sizeof(buf));
    ret = ctrl_get_range(dev, dev->clock_source_id, UAC2_CS_SAM_FREQ_CONTROL, 0, buf, sizeof(buf));
    if (ret != ESP_OK) { api_unlock(iface); return ret; }

    uint16_t count = buf[0] | (buf[1] << 8);
    if (count > UAC2_MAX_SAMPLE_RATE_RANGES) count = UAC2_MAX_SAMPLE_RATE_RANGES;

    for (int i = 0; i < count; i++) {
        const uint8_t *p = buf + 2 + (i * 12);
        ranges[i].min = (uint32_t)p[0] | ((uint32_t)p[1] << 8)
                      | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
        ranges[i].max = (uint32_t)p[4] | ((uint32_t)p[5] << 8)
                      | ((uint32_t)p[6] << 16) | ((uint32_t)p[7] << 24);
        ranges[i].res = (uint32_t)p[8] | ((uint32_t)p[9] << 8)
                      | ((uint32_t)p[10] << 16) | ((uint32_t)p[11] << 24);
    }
    *num_ranges = (uint8_t)count;
    api_unlock(iface);
    return ESP_OK;
}

esp_err_t uac2_host_device_get_clock_valid(uac2_host_device_handle_t handle, bool *valid)
{
    ESP_RETURN_ON_FALSE(handle && valid, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    uac2_iface_t *iface = get_iface_by_handle(handle);
    ESP_RETURN_ON_FALSE(iface, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uac2_device_t *dev = iface->parent;
    ESP_RETURN_ON_FALSE(dev->clock_source_id != 0, ESP_ERR_NOT_SUPPORTED, TAG, "No clock source");
    esp_err_t ret = api_lock(iface);
    if (ret != ESP_OK) return ret;

    uint8_t data = 0;
    ret = ctrl_get_cur(dev, dev->clock_source_id, UAC2_CS_CLOCK_VALID_CONTROL, 0, &data, 1);
    if (ret == ESP_OK) {
        *valid = (data & 0x01) != 0;
    }
    api_unlock(iface);
    return ret;
}

// ── Public API: Streaming ─────────────────────────────────────────

esp_err_t uac2_host_device_start(uac2_host_device_handle_t handle,
                                 const uac2_host_stream_config_t *config)
{
    ESP_RETURN_ON_FALSE(handle && config, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    uac2_iface_t *iface = get_iface_by_handle(handle);
    ESP_RETURN_ON_FALSE(iface, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    esp_err_t ret = api_lock(iface);
    if (ret != ESP_OK) return ret;

    if (iface->state != UAC2_IFACE_STATE_IDLE) {
        ESP_LOGE(TAG, "Stream already active");
        api_unlock(iface);
        return ESP_ERR_INVALID_STATE;
    }

    uac2_device_t *dev = iface->parent;

    // Find matching AS interface for this interface number + requested format
    const uac2_as_iface_t *as = find_matching_as_iface(&dev->desc_info, iface->dir,
                                                        config, iface->iface_num);
    if (!as) {
        ESP_LOGE(TAG, "No matching AS interface for %s %dch/%dbit",
                 iface->dir == UAC2_STREAM_TX ? "TX" : "RX",
                 config->channels, config->bit_resolution);
        api_unlock(iface);
        return ESP_ERR_NOT_FOUND;
    }

    if (as->ep_interval != 1) {
        ESP_LOGW(TAG, "Data EP 0x%02X bInterval=%d (expected 1 at FS), using 1ms framing",
                 as->ep_addr, as->ep_interval);
    }

    ESP_LOGI(TAG, "Starting %s stream: iface %d alt %d, %dch %d-bit, ep 0x%02X (MPS %d)",
             iface->dir == UAC2_STREAM_TX ? "TX" : "RX",
             as->interface_num, as->alt_setting,
             as->nr_channels, as->bit_resolution,
             as->ep_addr, as->ep_max_packet_size);

    // Allocate stream resources
    esp_err_t err = stream_resources_alloc(iface, as, config);
    if (err != ESP_OK) { api_unlock(iface); return err; }

    // Claim interface
    err = usb_host_interface_claim(s_uac2_driver->client_handle, dev->dev_hdl,
                                   iface->iface_num, iface->alt_setting);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to claim iface %d alt %d: %s",
                 iface->iface_num, iface->alt_setting, esp_err_to_name(err));
        stream_resources_free(iface);
        api_unlock(iface);
        return err;
    }

    // SET_INTERFACE to activate endpoints on device
    err = ctrl_request_no_data(dev,
        USB_BM_REQUEST_TYPE_DIR_OUT | USB_BM_REQUEST_TYPE_TYPE_STANDARD | USB_BM_REQUEST_TYPE_RECIP_INTERFACE,
        USB_B_REQUEST_SET_INTERFACE, iface->alt_setting, iface->iface_num);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SET_INTERFACE(%d, %d) failed: %s (continuing)",
                 iface->iface_num, iface->alt_setting, esp_err_to_name(err));
    }

    iface->state = UAC2_IFACE_STATE_READY;

    // Validate and set sample rate
    if (dev->clock_source_id != 0 && config->sample_freq > 0) {
        validate_sample_rate(dev, config->sample_freq);
        err = set_sample_rate_internal(dev, config->sample_freq);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to set sample rate (continuing anyway)");
        }
    }

    // Check for suspend-after-start flag
    if (config->flags & UAC2_FLAG_STREAM_SUSPEND_AFTER_START) {
        ESP_LOGI(TAG, "%s stream started (suspended)", iface->dir == UAC2_STREAM_TX ? "TX" : "RX");
        api_unlock(iface);
        return ESP_OK;
    }

    // Submit URBs
    err = stream_submit_urbs(iface);
    if (err != ESP_OK) {
        portENTER_CRITICAL(&iface->state_lock);
        iface->state = UAC2_IFACE_STATE_IDLE;
        portEXIT_CRITICAL(&iface->state_lock);
        ctrl_request_no_data(dev,
            USB_BM_REQUEST_TYPE_DIR_OUT | USB_BM_REQUEST_TYPE_TYPE_STANDARD | USB_BM_REQUEST_TYPE_RECIP_INTERFACE,
            USB_B_REQUEST_SET_INTERFACE, 0, iface->iface_num);
        usb_host_interface_release(s_uac2_driver->client_handle, dev->dev_hdl, iface->iface_num);
        int wait_ms = 0;
        while (atomic_load(&iface->urbs_in_flight) > 0 && wait_ms < 500) {
            vTaskDelay(pdMS_TO_TICKS(5));
            wait_ms += 5;
        }
        stream_resources_free(iface);
        api_unlock(iface);
        return err;
    }

    ESP_LOGI(TAG, "%s stream started (pkt_size=%d, ringbuf=%" PRIu32 ")",
             iface->dir == UAC2_STREAM_TX ? "TX" : "RX",
             iface->packet_size, iface->ringbuf_size);
    api_unlock(iface);
    return ESP_OK;
}

// Internal stream_stop — caller must hold api_mutex (or be in close path)
static esp_err_t stream_stop_internal(uac2_iface_t *iface)
{
    // If state is IDLE and no resources allocated, nothing to do.
    // But if state is IDLE with resources still present (disconnect set state
    // to IDLE without freeing), we must still clean up.
    if (iface->state == UAC2_IFACE_STATE_IDLE && iface->xfer_count == 0) return ESP_OK;

    uac2_device_t *dev = iface->parent;

    portENTER_CRITICAL(&iface->state_lock);
    iface->state = UAC2_IFACE_STATE_IDLE;
    portEXIT_CRITICAL(&iface->state_lock);

    // SET_INTERFACE(alt=0) — skip if device already gone (avoids 5s timeout)
    if (!atomic_load(&dev->gone)) {
        esp_err_t si_err = ctrl_request_no_data(dev,
            USB_BM_REQUEST_TYPE_DIR_OUT | USB_BM_REQUEST_TYPE_TYPE_STANDARD | USB_BM_REQUEST_TYPE_RECIP_INTERFACE,
            USB_B_REQUEST_SET_INTERFACE, 0, iface->iface_num);
        if (si_err != ESP_OK) {
            ESP_LOGW(TAG, "SET_INTERFACE(%d, 0) failed: %s", iface->iface_num, esp_err_to_name(si_err));
        }
    }

    // Halt and flush host-side pipes (skip if device gone — endpoints already invalid)
    esp_err_t halt_err = atomic_load(&dev->gone) ? ESP_FAIL : usb_host_endpoint_halt(dev->dev_hdl, iface->ep_addr);
    if (halt_err == ESP_OK) {
        usb_host_endpoint_flush(dev->dev_hdl, iface->ep_addr);
        usb_host_endpoint_clear(dev->dev_hdl, iface->ep_addr);
    }
    if (iface->fb_ep_addr && !atomic_load(&dev->gone)) {
        halt_err = usb_host_endpoint_halt(dev->dev_hdl, iface->fb_ep_addr);
        if (halt_err == ESP_OK) {
            usb_host_endpoint_flush(dev->dev_hdl, iface->fb_ep_addr);
            usb_host_endpoint_clear(dev->dev_hdl, iface->fb_ep_addr);
        }
    }

    // Release interface — retry for ESP-IDF bug #17707 (skip if device gone)
    for (int retry = 0; retry < 5 && !atomic_load(&dev->gone); retry++) {
        esp_err_t rel_err = usb_host_interface_release(s_uac2_driver->client_handle,
                                                        dev->dev_hdl, iface->iface_num);
        if (rel_err == ESP_OK) break;
        if (rel_err == ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "Interface release: URBs in-flight, retry %d", retry + 1);
            vTaskDelay(pdMS_TO_TICKS(20));
        } else {
            ESP_LOGE(TAG, "Interface release failed: %s", esp_err_to_name(rel_err));
            break;
        }
    }

    // Wait for in-flight URBs
    int wait_ms = 0;
    while (atomic_load(&iface->urbs_in_flight) > 0 && wait_ms < 2000) {
        vTaskDelay(pdMS_TO_TICKS(5));
        wait_ms += 5;
    }
    if (atomic_load(&iface->urbs_in_flight) > 0) {
        ESP_LOGE(TAG, "Stream stop: %d URBs still in-flight after 2s — leaking to avoid crash",
                 atomic_load(&iface->urbs_in_flight));
        // Leak stream resources — URB callbacks still reference them.
        // Zero the pointers so we don't try to free them again.
        memset(iface->xfer, 0, sizeof(iface->xfer));
        iface->xfer_count = 0;
        iface->fb_xfer = NULL;
        iface->ringbuf = NULL;
        iface->user_task_done = NULL;
        // Set ERROR state to prevent device_start from re-using this interface
        portENTER_CRITICAL(&iface->state_lock);
        iface->state = UAC2_IFACE_STATE_ERROR;
        portEXIT_CRITICAL(&iface->state_lock);
        return ESP_ERR_TIMEOUT;
    }

    stream_resources_free(iface);
    ESP_LOGI(TAG, "%s stream stopped", iface->dir == UAC2_STREAM_TX ? "TX" : "RX");
    return ESP_OK;
}

esp_err_t uac2_host_device_stop(uac2_host_device_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uac2_iface_t *iface = get_iface_by_handle(handle);
    ESP_RETURN_ON_FALSE(iface, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    esp_err_t ret = api_lock(iface);
    if (ret != ESP_OK) return ret;
    ret = stream_stop_internal(iface);
    api_unlock(iface);
    return ret;
}

esp_err_t uac2_host_device_write(uac2_host_device_handle_t handle,
                                 const uint8_t *data, uint32_t size,
                                 uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(handle && data && size > 0, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    uac2_iface_t *iface = (uac2_iface_t *)handle;

    if (!iface->ringbuf || iface->state != UAC2_IFACE_STATE_ACTIVE) {
        return ESP_ERR_INVALID_STATE;
    }

    atomic_store(&iface->user_task_blocked, true);
    BaseType_t ok = xRingbufferSend(iface->ringbuf, data, size,
                                    pdMS_TO_TICKS(timeout_ms));

    bool still_active = (iface->state == UAC2_IFACE_STATE_ACTIVE);
    if (ok == pdTRUE && still_active) {
        atomic_store(&iface->tx_done_pending, false);
    }

    // Signal stream_resources_free only on actual blocked→unblocked transition
    if (atomic_exchange(&iface->user_task_blocked, false) && iface->user_task_done) {
        xSemaphoreGive(iface->user_task_done);
    }

    if (!still_active) return ESP_ERR_INVALID_STATE;
    return ok == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t uac2_host_device_read(uac2_host_device_handle_t handle,
                                uint8_t *data, uint32_t size,
                                uint32_t *bytes_read,
                                uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(handle && data && bytes_read && size > 0, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    uac2_iface_t *iface = (uac2_iface_t *)handle;

    if (!iface->ringbuf || iface->state != UAC2_IFACE_STATE_ACTIVE) {
        return ESP_ERR_INVALID_STATE;
    }

    atomic_store(&iface->user_task_blocked, true);
    size_t item_size = 0;
    void *item = xRingbufferReceiveUpTo(iface->ringbuf, &item_size,
                                        pdMS_TO_TICKS(timeout_ms), size);

    esp_err_t ret;
    bool still_active = (iface->state == UAC2_IFACE_STATE_ACTIVE);
    if (!still_active) {
        if (item) vRingbufferReturnItem(iface->ringbuf, item);
        *bytes_read = 0;
        ret = ESP_ERR_INVALID_STATE;
    } else if (!item) {
        *bytes_read = 0;
        ret = ESP_ERR_TIMEOUT;
    } else {
        memcpy(data, item, item_size);
        vRingbufferReturnItem(iface->ringbuf, item);
        *bytes_read = (uint32_t)item_size;
        ret = ESP_OK;
    }

    if (atomic_exchange(&iface->user_task_blocked, false) && iface->user_task_done) {
        xSemaphoreGive(iface->user_task_done);
    }
    return ret;
}

int64_t uac2_host_device_get_start_time(uac2_host_device_handle_t handle)
{
    if (!handle) return 0;
    uac2_iface_t *iface = (uac2_iface_t *)handle;
    return atomic_load(&iface->first_frame_us);
}

uint32_t uac2_host_device_get_feedback(uac2_host_device_handle_t handle)
{
    if (!handle) return 0;
    uac2_iface_t *iface = (uac2_iface_t *)handle;
    return atomic_load(&iface->fb_value);
}

// ── Public API: Volume / Mute ─────────────────────────────────────

esp_err_t uac2_host_device_set_mute(uac2_host_device_handle_t handle,
                                    uint8_t channel, bool mute)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uac2_iface_t *iface = get_iface_by_handle(handle);
    ESP_RETURN_ON_FALSE(iface, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uac2_device_t *dev = iface->parent;
    ESP_RETURN_ON_FALSE(dev->has_feature_unit, ESP_ERR_NOT_SUPPORTED, TAG, "No feature unit");
    ESP_RETURN_ON_FALSE(dev->has_mute, ESP_ERR_NOT_SUPPORTED, TAG, "No mute control");
    esp_err_t ret = api_lock(iface);
    if (ret != ESP_OK) return ret;
    uint8_t data = mute ? 1 : 0;
    ret = ctrl_set_cur(dev, dev->feature_unit_id, UAC2_FU_MUTE_CONTROL, channel, &data, 1);
    api_unlock(iface);
    return ret;
}

esp_err_t uac2_host_device_get_mute(uac2_host_device_handle_t handle,
                                    uint8_t channel, bool *mute)
{
    ESP_RETURN_ON_FALSE(handle && mute, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    uac2_iface_t *iface = get_iface_by_handle(handle);
    ESP_RETURN_ON_FALSE(iface, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uac2_device_t *dev = iface->parent;
    ESP_RETURN_ON_FALSE(dev->has_feature_unit, ESP_ERR_NOT_SUPPORTED, TAG, "No feature unit");
    ESP_RETURN_ON_FALSE(dev->has_mute, ESP_ERR_NOT_SUPPORTED, TAG, "No mute control");
    esp_err_t ret = api_lock(iface);
    if (ret != ESP_OK) return ret;
    uint8_t data = 0;
    ret = ctrl_get_cur(dev, dev->feature_unit_id, UAC2_FU_MUTE_CONTROL, channel, &data, 1);
    if (ret == ESP_OK) *mute = (data != 0);
    api_unlock(iface);
    return ret;
}

esp_err_t uac2_host_device_set_volume(uac2_host_device_handle_t handle,
                                      uint8_t channel, int16_t volume_db256)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uac2_iface_t *iface = get_iface_by_handle(handle);
    ESP_RETURN_ON_FALSE(iface, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uac2_device_t *dev = iface->parent;
    ESP_RETURN_ON_FALSE(dev->has_feature_unit, ESP_ERR_NOT_SUPPORTED, TAG, "No feature unit");
    ESP_RETURN_ON_FALSE(dev->has_volume, ESP_ERR_NOT_SUPPORTED, TAG, "No volume control");
    esp_err_t ret = api_lock(iface);
    if (ret != ESP_OK) return ret;
    // Range check under lock to avoid TOCTOU with reconnect
    if (dev->volume_range_valid) {
        if (volume_db256 < dev->volume_min_db256 || volume_db256 > dev->volume_max_db256) {
            ESP_LOGE(TAG, "Volume %.2f dB out of range [%.2f, %.2f]",
                     volume_db256 / 256.0, dev->volume_min_db256 / 256.0,
                     dev->volume_max_db256 / 256.0);
            api_unlock(iface);
            return ESP_ERR_INVALID_ARG;
        }
    }
    uint8_t data[2] = { (uint8_t)(volume_db256 & 0xFF), (uint8_t)((volume_db256 >> 8) & 0xFF) };
    ret = ctrl_set_cur(dev, dev->feature_unit_id, UAC2_FU_VOLUME_CONTROL, channel, data, 2);
    api_unlock(iface);
    return ret;
}

esp_err_t uac2_host_device_get_volume(uac2_host_device_handle_t handle,
                                      uint8_t channel, int16_t *volume_db256)
{
    ESP_RETURN_ON_FALSE(handle && volume_db256, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    uac2_iface_t *iface = get_iface_by_handle(handle);
    ESP_RETURN_ON_FALSE(iface, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uac2_device_t *dev = iface->parent;
    ESP_RETURN_ON_FALSE(dev->has_feature_unit, ESP_ERR_NOT_SUPPORTED, TAG, "No feature unit");
    ESP_RETURN_ON_FALSE(dev->has_volume, ESP_ERR_NOT_SUPPORTED, TAG, "No volume control");
    esp_err_t ret = api_lock(iface);
    if (ret != ESP_OK) return ret;
    uint8_t data[2] = {0};
    ret = ctrl_get_cur(dev, dev->feature_unit_id, UAC2_FU_VOLUME_CONTROL, channel, data, 2);
    if (ret == ESP_OK) *volume_db256 = (int16_t)(data[0] | (data[1] << 8));
    api_unlock(iface);
    return ret;
}

esp_err_t uac2_host_device_get_volume_range(uac2_host_device_handle_t handle,
                                            uint8_t channel,
                                            uac2_volume_range_t *ranges,
                                            uint8_t *num_ranges)
{
    ESP_RETURN_ON_FALSE(handle && ranges && num_ranges, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    uac2_iface_t *iface = get_iface_by_handle(handle);
    ESP_RETURN_ON_FALSE(iface, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uac2_device_t *dev = iface->parent;
    ESP_RETURN_ON_FALSE(dev->has_feature_unit, ESP_ERR_NOT_SUPPORTED, TAG, "No feature unit");
    esp_err_t ret = api_lock(iface);
    if (ret != ESP_OK) return ret;

    uint8_t buf[2 + UAC2_MAX_VOLUME_RANGES * 6];
    memset(buf, 0, sizeof(buf));
    ret = ctrl_get_range(dev, dev->feature_unit_id, UAC2_FU_VOLUME_CONTROL, channel, buf, sizeof(buf));
    if (ret != ESP_OK) { api_unlock(iface); return ret; }

    uint16_t count = buf[0] | (buf[1] << 8);
    if (count > UAC2_MAX_VOLUME_RANGES) count = UAC2_MAX_VOLUME_RANGES;
    for (int i = 0; i < count; i++) {
        const uint8_t *p = buf + 2 + (i * 6);
        ranges[i].min = (int16_t)(p[0] | (p[1] << 8));
        ranges[i].max = (int16_t)(p[2] | (p[3] << 8));
        ranges[i].res = (int16_t)(p[4] | (p[5] << 8));
    }
    *num_ranges = (uint8_t)count;
    api_unlock(iface);
    return ESP_OK;
}

esp_err_t uac2_host_device_set_volume_percent(uac2_host_device_handle_t handle,
                                              uint8_t channel, uint8_t percent)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(percent <= 100, ESP_ERR_INVALID_ARG, TAG, "Percent must be 0-100");
    uac2_iface_t *iface = get_iface_by_handle(handle);
    ESP_RETURN_ON_FALSE(iface, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uac2_device_t *dev = iface->parent;
    ESP_RETURN_ON_FALSE(dev->volume_range_valid, ESP_ERR_NOT_SUPPORTED, TAG, "Volume range not cached");
    int16_t db256 = dev->volume_min_db256 +
        (int16_t)(((int32_t)(dev->volume_max_db256 - dev->volume_min_db256) * percent) / 100);
    return uac2_host_device_set_volume(handle, channel, db256);
}

esp_err_t uac2_host_device_get_volume_percent(uac2_host_device_handle_t handle,
                                              uint8_t channel, uint8_t *percent)
{
    ESP_RETURN_ON_FALSE(handle && percent, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    uac2_iface_t *iface = get_iface_by_handle(handle);
    ESP_RETURN_ON_FALSE(iface, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uac2_device_t *dev = iface->parent;
    ESP_RETURN_ON_FALSE(dev->volume_range_valid, ESP_ERR_NOT_SUPPORTED, TAG, "Volume range not cached");
    int16_t db256;
    esp_err_t err = uac2_host_device_get_volume(handle, channel, &db256);
    if (err != ESP_OK) return err;
    int32_t range = dev->volume_max_db256 - dev->volume_min_db256;
    if (range <= 0) { *percent = 0; }
    else {
        int32_t pct = ((int32_t)(db256 - dev->volume_min_db256) * 100) / range;
        if (pct < 0) pct = 0;
        if (pct > 100) pct = 100;
        *percent = (uint8_t)pct;
    }
    return ESP_OK;
}

esp_err_t uac2_host_device_set_volume_all_channels(uac2_host_device_handle_t handle,
                                                   int16_t volume_db256)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uac2_iface_t *iface = get_iface_by_handle(handle);
    ESP_RETURN_ON_FALSE(iface, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uac2_device_t *dev = iface->parent;
    ESP_RETURN_ON_FALSE(dev->has_feature_unit, ESP_ERR_NOT_SUPPORTED, TAG, "No feature unit");
    ESP_RETURN_ON_FALSE(dev->has_volume, ESP_ERR_NOT_SUPPORTED, TAG, "No volume control");

    const uac2_feature_unit_t *fu = &dev->desc_info.feature_units[0];
    uint32_t ch_map = fu->volume_ch_map;

    for (uint8_t ch = 0; ch <= fu->nr_channels && ch < 32; ch++) {
        if (ch_map & (1u << ch)) {
            esp_err_t err = uac2_host_device_set_volume(handle, ch, volume_db256);
            if (err != ESP_OK) return err;
        }
    }
    return ESP_OK;
}

// ── Public API: Debug Print ───────────────────────────────────────

static const char *iface_state_str(uac2_iface_state_t state)
{
    switch (state) {
    case UAC2_IFACE_STATE_IDLE:       return "IDLE";
    case UAC2_IFACE_STATE_READY:      return "READY";
    case UAC2_IFACE_STATE_ACTIVE:     return "ACTIVE";
    case UAC2_IFACE_STATE_SUSPENDING: return "SUSPENDING";
    case UAC2_IFACE_STATE_ERROR:      return "ERROR";
    default: return "UNKNOWN";
    }
}

void uac2_host_device_print_info(uac2_host_device_handle_t handle)
{
    if (!handle) { ESP_LOGE(TAG, "print_info: NULL handle"); return; }
    uac2_iface_t *iface = get_iface_by_handle(handle);
    if (!iface) { ESP_LOGE(TAG, "print_info: invalid handle"); return; }
    uac2_device_t *dev = iface->parent;

    bool locked = (xSemaphoreTake(iface->api_mutex, pdMS_TO_TICKS(500)) == pdTRUE);

    uac2_log_device_info(&dev->desc_info);

    ESP_LOGI(TAG, "--- Runtime State ---");
    ESP_LOGI(TAG, "  Clock source ID: %d", dev->clock_source_id);
    ESP_LOGI(TAG, "  Feature unit: %s (ID=%d, mute=%s, volume=%s)",
             dev->has_feature_unit ? "yes" : "no", dev->feature_unit_id,
             dev->has_mute ? "yes" : "no", dev->has_volume ? "yes" : "no");
    if (dev->volume_range_valid) {
        ESP_LOGI(TAG, "  Volume range: %.2f to %.2f dB (res %.4f dB)",
                 dev->volume_min_db256 / 256.0, dev->volume_max_db256 / 256.0,
                 dev->volume_res_db256 / 256.0);
    }
    ESP_LOGI(TAG, "  Interface %d (%s): %s, ep=0x%02X, pkt=%d, ringbuf=%" PRIu32 ", urbs=%d",
             iface->iface_num,
             iface->dir == UAC2_STREAM_TX ? "TX" : "RX",
             iface_state_str(iface->state),
             iface->ep_addr, iface->packet_size,
             iface->ringbuf_size, atomic_load(&iface->urbs_in_flight));
    if (iface->dir == UAC2_STREAM_TX) {
        ESP_LOGI(TAG, "  Feedback: 0x%08" PRIX32, (uint32_t)atomic_load(&iface->fb_value));
    }

    if (locked) xSemaphoreGive(iface->api_mutex);
}

// ── Public API: Suspend / Resume ──────────────────────────────────

static esp_err_t stream_deactivate(uac2_iface_t *iface)
{
    uac2_device_t *dev = iface->parent;

    esp_err_t si_err = ctrl_request_no_data(dev,
        USB_BM_REQUEST_TYPE_DIR_OUT | USB_BM_REQUEST_TYPE_TYPE_STANDARD | USB_BM_REQUEST_TYPE_RECIP_INTERFACE,
        USB_B_REQUEST_SET_INTERFACE, 0, iface->iface_num);
    if (si_err != ESP_OK) {
        ESP_LOGW(TAG, "SET_INTERFACE(%d, 0) failed: %s", iface->iface_num, esp_err_to_name(si_err));
    }

    esp_err_t halt_err = usb_host_endpoint_halt(dev->dev_hdl, iface->ep_addr);
    if (halt_err == ESP_OK) {
        usb_host_endpoint_flush(dev->dev_hdl, iface->ep_addr);
        usb_host_endpoint_clear(dev->dev_hdl, iface->ep_addr);
    }
    if (iface->fb_ep_addr) {
        halt_err = usb_host_endpoint_halt(dev->dev_hdl, iface->fb_ep_addr);
        if (halt_err == ESP_OK) {
            usb_host_endpoint_flush(dev->dev_hdl, iface->fb_ep_addr);
            usb_host_endpoint_clear(dev->dev_hdl, iface->fb_ep_addr);
        }
    }

    int wait_ms = 0;
    while (atomic_load(&iface->urbs_in_flight) > 0 && wait_ms < 2000) {
        vTaskDelay(pdMS_TO_TICKS(5));
        wait_ms += 5;
    }
    if (atomic_load(&iface->urbs_in_flight) > 0) {
        ESP_LOGE(TAG, "Deactivate: %d URBs still in-flight after 2s — leaking to avoid crash",
                 atomic_load(&iface->urbs_in_flight));
        // Zero pointers so subsequent stream_stop_internal won't double-free
        memset(iface->xfer, 0, sizeof(iface->xfer));
        iface->xfer_count = 0;
        iface->fb_xfer = NULL;
        iface->ringbuf = NULL;
        iface->user_task_done = NULL;
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t uac2_host_device_suspend(uac2_host_device_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uac2_iface_t *iface = get_iface_by_handle(handle);
    ESP_RETURN_ON_FALSE(iface, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    esp_err_t ret = api_lock(iface);
    if (ret != ESP_OK) return ret;

    portENTER_CRITICAL(&iface->state_lock);
    uac2_iface_state_t state = iface->state;
    if (state == UAC2_IFACE_STATE_ACTIVE) {
        iface->state = UAC2_IFACE_STATE_SUSPENDING;
    }
    portEXIT_CRITICAL(&iface->state_lock);

    if (state == UAC2_IFACE_STATE_READY) {
        api_unlock(iface);
        return ESP_OK;
    }
    if (state != UAC2_IFACE_STATE_ACTIVE) {
        api_unlock(iface);
        return ESP_ERR_INVALID_STATE;
    }

    ret = stream_deactivate(iface);
    if (ret != ESP_OK) {
        portENTER_CRITICAL(&iface->state_lock);
        iface->state = UAC2_IFACE_STATE_ERROR;
        portEXIT_CRITICAL(&iface->state_lock);
        api_unlock(iface);
        return ret;
    }

    // Flush ringbuffer
    if (iface->ringbuf) {
        size_t item_size;
        void *item;
        while ((item = xRingbufferReceiveUpTo(iface->ringbuf, &item_size, 0,
                                               iface->ringbuf_size)) != NULL) {
            vRingbufferReturnItem(iface->ringbuf, item);
        }
    }

    atomic_store(&iface->consecutive_errors, 0);
    atomic_store(&iface->fb_value, 0);
    iface->fb_accumulator = 0;
    atomic_store(&iface->first_frame_us, 0);
    atomic_store(&iface->tx_done_pending, false);

    portENTER_CRITICAL(&iface->state_lock);
    iface->state = UAC2_IFACE_STATE_READY;
    portEXIT_CRITICAL(&iface->state_lock);

    ESP_LOGI(TAG, "%s stream suspended", iface->dir == UAC2_STREAM_TX ? "TX" : "RX");
    api_unlock(iface);
    return ESP_OK;
}

esp_err_t uac2_host_device_resume(uac2_host_device_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uac2_iface_t *iface = get_iface_by_handle(handle);
    ESP_RETURN_ON_FALSE(iface, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    esp_err_t ret = api_lock(iface);
    if (ret != ESP_OK) return ret;

    portENTER_CRITICAL(&iface->state_lock);
    uac2_iface_state_t state = iface->state;
    portEXIT_CRITICAL(&iface->state_lock);

    if (state == UAC2_IFACE_STATE_ACTIVE) {
        api_unlock(iface);
        return ESP_OK;
    }
    if (state != UAC2_IFACE_STATE_READY) {
        api_unlock(iface);
        return ESP_ERR_INVALID_STATE;
    }

    uac2_device_t *dev = iface->parent;

    ret = ctrl_request_no_data(dev,
        USB_BM_REQUEST_TYPE_DIR_OUT | USB_BM_REQUEST_TYPE_TYPE_STANDARD | USB_BM_REQUEST_TYPE_RECIP_INTERFACE,
        USB_B_REQUEST_SET_INTERFACE, iface->alt_setting, iface->iface_num);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "SET_INTERFACE(%d, %d) failed on resume: %s",
                 iface->iface_num, iface->alt_setting, esp_err_to_name(ret));
    }

    if (dev->clock_source_id != 0 && iface->sample_rate > 0) {
        set_sample_rate_internal(dev, iface->sample_rate);
    }

    ret = stream_submit_urbs(iface);
    if (ret != ESP_OK) {
        portENTER_CRITICAL(&iface->state_lock);
        iface->state = UAC2_IFACE_STATE_READY;
        portEXIT_CRITICAL(&iface->state_lock);
        ESP_LOGE(TAG, "Resume failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "%s stream resumed", iface->dir == UAC2_STREAM_TX ? "TX" : "RX");
    }

    api_unlock(iface);
    return ret;
}
