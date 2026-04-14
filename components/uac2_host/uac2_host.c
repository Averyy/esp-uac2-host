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
    uint8_t feature_unit_id;
    bool has_feature_unit;
    bool has_mute;
    bool has_volume;
    bool volume_range_valid;
    int16_t volume_min_db256;
    int16_t volume_max_db256;
    int16_t volume_res_db256;

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
    _Atomic bool closing;
    atomic_uint io_users;

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
    bool interface_claimed;

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
    SemaphoreHandle_t lifecycle_mutex;
} uac2_driver_t;

static uac2_driver_t *s_uac2_driver;

// ── Forward declarations ──────────────────────────────────────────

static esp_err_t stream_stop_internal(uac2_iface_t *iface);
static void stream_tx_xfer_submit(uac2_iface_t *iface, usb_transfer_t *xfer);
static esp_err_t release_interface_claim(uac2_device_t *dev, uac2_iface_t *iface, TickType_t timeout_ticks);
static esp_err_t stream_deactivate(uac2_iface_t *iface);
static esp_err_t read_current_sample_rate(uac2_device_t *dev, uint32_t *sample_rate);
static void validate_sample_rate(uac2_device_t *dev, uint32_t sample_rate);
static void device_destroy(uac2_device_t *dev);
static esp_err_t wait_for_urbs_quiesced(uac2_iface_t *iface, uint32_t timeout_ms,
                                        const char *stage, bool set_error_state);

static inline bool stream_has_resources(const uac2_iface_t *iface)
{
    return iface->xfer_count > 0 ||
           iface->fb_xfer != NULL ||
           iface->ringbuf != NULL ||
           iface->user_task_done != NULL;
}

static esp_err_t driver_lifecycle_lock(void)
{
    if (!s_uac2_driver || !s_uac2_driver->lifecycle_mutex) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_uac2_driver->lifecycle_mutex, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Lifecycle mutex timeout");
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static void driver_lifecycle_unlock(void)
{
    if (s_uac2_driver && s_uac2_driver->lifecycle_mutex) {
        xSemaphoreGive(s_uac2_driver->lifecycle_mutex);
    }
}

static esp_err_t acquire_iface_runtime_ref(uac2_host_device_handle_t handle, uac2_iface_t **out_iface)
{
    if (!out_iface) return ESP_ERR_INVALID_ARG;
    *out_iface = NULL;
    if (!handle) return ESP_ERR_INVALID_ARG;
    if (!s_uac2_driver) return ESP_ERR_INVALID_STATE;

    uac2_iface_t *target = (uac2_iface_t *)handle;
    bool found = false;

    UAC2_ENTER_CRITICAL();
    uac2_iface_t *iface = NULL;
    STAILQ_FOREACH(iface, &s_uac2_driver->ifaces_tailq, tailq_entry) {
        if (iface == target) {
            found = true;
            if (!atomic_load(&iface->closing)) {
                atomic_fetch_add(&iface->io_users, 1);
                *out_iface = iface;
                UAC2_EXIT_CRITICAL();
                return ESP_OK;
            }
            break;
        }
    }
    UAC2_EXIT_CRITICAL();
    return found ? ESP_ERR_INVALID_STATE : ESP_ERR_INVALID_ARG;
}

static uac2_iface_t *acquire_iface_io_ref(uac2_host_device_handle_t handle)
{
    uac2_iface_t *iface = NULL;
    if (acquire_iface_runtime_ref(handle, &iface) != ESP_OK) {
        return NULL;
    }
    return iface;
}

static void release_iface_io_ref(uac2_iface_t *iface)
{
    if (iface) {
        atomic_fetch_sub(&iface->io_users, 1);
    }
}

static esp_err_t wait_for_iface_io_quiesced(uac2_iface_t *iface, TickType_t timeout_ticks)
{
    TickType_t start = xTaskGetTickCount();
    while (atomic_load(&iface->io_users) > 0) {
        if ((xTaskGetTickCount() - start) >= timeout_ticks) {
            ESP_LOGE(TAG, "I/O users still active during close: %u",
                     (unsigned)atomic_load(&iface->io_users));
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return ESP_OK;
}

static esp_err_t release_interface_claim(uac2_device_t *dev, uac2_iface_t *iface, TickType_t timeout_ticks)
{
    TickType_t start = xTaskGetTickCount();
    uint32_t retries = 0;
    while ((xTaskGetTickCount() - start) < timeout_ticks) {
        esp_err_t rel_err = usb_host_interface_release(s_uac2_driver->client_handle,
                                                       dev->dev_hdl, iface->iface_num);
        if (rel_err == ESP_OK) {
            if (retries > 0) {
                ESP_LOGD(TAG, "Interface release succeeded after %" PRIu32 " retries", retries);
            }
            return ESP_OK;
        }
        if (rel_err == ESP_ERR_INVALID_STATE) {
            retries++;
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (atomic_load(&dev->gone) &&
            (rel_err == ESP_ERR_INVALID_ARG || rel_err == ESP_ERR_NOT_FOUND)) {
            ESP_LOGW(TAG, "Interface release after disconnect returned %s",
                     esp_err_to_name(rel_err));
            return ESP_OK;
        }
        ESP_LOGE(TAG, "Interface release failed: %s", esp_err_to_name(rel_err));
        return rel_err;
    }

    ESP_LOGE(TAG, "Interface release timed out after %" PRIu32 " retries", retries);
    return ESP_ERR_TIMEOUT;
}

static esp_err_t wait_for_urbs_quiesced(uac2_iface_t *iface, uint32_t timeout_ms,
                                        const char *stage, bool set_error_state)
{
    uint32_t wait_ms = 0;
    while (atomic_load(&iface->urbs_in_flight) > 0 && wait_ms < timeout_ms) {
        vTaskDelay(pdMS_TO_TICKS(5));
        wait_ms += 5;
    }
    if (atomic_load(&iface->urbs_in_flight) > 0) {
        ESP_LOGE(TAG, "%s: %d URBs still in-flight after %" PRIu32 "ms — retaining resources",
                 stage, atomic_load(&iface->urbs_in_flight), timeout_ms);
        if (set_error_state) {
            portENTER_CRITICAL(&iface->state_lock);
            iface->state = UAC2_IFACE_STATE_ERROR;
            portEXIT_CRITICAL(&iface->state_lock);
        }
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

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

static inline esp_err_t api_lock_internal(uac2_iface_t *iface, bool allow_closing)
{
    if (xSemaphoreTake(iface->api_mutex, pdMS_TO_TICKS(UAC2_API_MUTEX_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "API mutex timeout");
        return ESP_ERR_TIMEOUT;
    }
    if (!allow_closing && atomic_load(&iface->closing)) {
        xSemaphoreGive(iface->api_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

static inline esp_err_t api_lock(uac2_iface_t *iface)
{
    return api_lock_internal(iface, false);
}

static inline esp_err_t api_lock_allow_closing(uac2_iface_t *iface)
{
    return api_lock_internal(iface, true);
}

static inline void api_unlock(uac2_iface_t *iface) {
    xSemaphoreGive(iface->api_mutex);
}

static esp_err_t acquire_locked_iface(uac2_host_device_handle_t handle, uac2_iface_t **out_iface)
{
    esp_err_t err = acquire_iface_runtime_ref(handle, out_iface);
    if (err != ESP_OK) return err;

    err = api_lock(*out_iface);
    if (err != ESP_OK) {
        release_iface_io_ref(*out_iface);
        *out_iface = NULL;
    }
    return err;
}

static void release_locked_iface(uac2_iface_t *iface)
{
    if (!iface) return;
    api_unlock(iface);
    release_iface_io_ref(iface);
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
        int actual = ctrl_get_actual_len(dev);
        if (actual != len) {
            ESP_LOGE(TAG, "GET_CUR short response: entity=%u ctrl=%u ch=%u expected=%u actual=%d",
                     entity_id, control_selector, channel, len, actual);
            xSemaphoreGive(dev->ctrl_mutex);
            return ESP_ERR_INVALID_RESPONSE;
        }
        if (data) {
            if (len > 0) {
                memcpy(data, dev->ctrl_xfer->data_buffer + sizeof(usb_setup_packet_t), len);
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
                                uint8_t *data, uint16_t len,
                                uint16_t *actual_len_out)
{
    uint16_t w_value = (control_selector << 8) | channel;
    uint16_t w_index = (entity_id << 8) | dev->ac_iface_num;

    esp_err_t err = ctrl_request(dev, UAC2_CTRL_GET, UAC2_REQUEST_RANGE,
                                 w_value, w_index, len, NULL);
    if (err == ESP_OK) {
        int actual = ctrl_get_actual_len(dev);
        if (actual < 2) {
            ESP_LOGE(TAG, "GET_RANGE short response: entity=%u ctrl=%u ch=%u actual=%d",
                     entity_id, control_selector, channel, actual);
            xSemaphoreGive(dev->ctrl_mutex);
            return ESP_ERR_INVALID_RESPONSE;
        }
        if (actual_len_out) {
            *actual_len_out = (uint16_t)actual;
        }
        if (data) {
            int copy_len = actual < len ? actual : len;
            if (copy_len > 0) {
                memcpy(data, dev->ctrl_xfer->data_buffer + sizeof(usb_setup_packet_t), copy_len);
            }
        }
        xSemaphoreGive(dev->ctrl_mutex);
    }
    return err;
}

static esp_err_t ensure_iface_device_available(uac2_iface_t *iface,
                                               uac2_iface_state_t expected_state,
                                               const char *stage)
{
    if (!iface) {
        return ESP_ERR_INVALID_ARG;
    }
    if (atomic_load(&iface->parent->gone)) {
        ESP_LOGW(TAG, "%s aborted: device disconnected", stage);
        return ESP_ERR_INVALID_STATE;
    }

    portENTER_CRITICAL(&iface->state_lock);
    uac2_iface_state_t state = iface->state;
    portEXIT_CRITICAL(&iface->state_lock);
    if (state != expected_state) {
        ESP_LOGW(TAG, "%s aborted: iface state=%d expected=%d",
                 stage, state, expected_state);
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
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
    case USB_TRANSFER_STATUS_COMPLETED: {
        atomic_store(&iface->consecutive_errors, 0);
        // Check per-packet status — the HCD sets overall status to COMPLETED
        // even when individual packets are SKIPPED (frame slot missed).
        int skipped = 0;
        int errors = 0;
        for (int i = 0; i < xfer->num_isoc_packets; i++) {
            usb_transfer_status_t pkt_st = xfer->isoc_packet_desc[i].status;
            if (pkt_st == USB_TRANSFER_STATUS_SKIPPED) {
                skipped++;
            } else if (pkt_st != USB_TRANSFER_STATUS_COMPLETED) {
                errors++;
            }
        }
        if (skipped > 0 || errors > 0) {
            ESP_LOGW(TAG, "TX URB: %d/%d packets skipped, %d errors",
                     skipped, xfer->num_isoc_packets, errors);
        }
        stream_tx_xfer_submit(iface, xfer);
        break;
    }

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

    int num_pkts = xfer->num_isoc_packets;
    uint32_t fb = atomic_load(&iface->fb_value);
    bool use_feedback = iface->fb_ep_addr != 0 &&
                        iface->sample_rate > 0 &&
                        (iface->sample_rate % 1000) != 0;
    uint32_t bytes_filled = 0;

    for (int i = 0; i < num_pkts; i++) {
        // Feedback-based adaptive packet sizing (per packet)
        uint16_t pkt_size;
        if (use_feedback && fb > 0) {
            uint16_t nominal_samples = (uint16_t)(fb >> 16);
            uint16_t fraction = (uint16_t)(fb & 0xFFFF);
            if (nominal_samples == 0 || nominal_samples > 1000) {
                pkt_size = iface->packet_size;
            } else {
                iface->fb_accumulator += fraction;
                uint16_t extra = (uint16_t)(iface->fb_accumulator >> 16);
                iface->fb_accumulator &= 0xFFFF;
                uint16_t samples_this_frame = nominal_samples + extra;
                uint32_t raw_pkt_size = (uint32_t)samples_this_frame * iface->channels * iface->sub_slot_size;
                pkt_size = (raw_pkt_size > iface->ep_mps) ? iface->ep_mps : (uint16_t)raw_pkt_size;
            }
        } else {
            pkt_size = iface->packet_size;
        }

        // HCD reads isochronous OUT data contiguously (hcd_dwc.c _buffer_fill_isoc)
        uint8_t *pkt_buf = xfer->data_buffer + bytes_filled;
        size_t item_size = 0;
        void *data = xRingbufferReceiveUpTo(iface->ringbuf, &item_size, 0, pkt_size);

        if (data && item_size > 0) {
            memcpy(pkt_buf, data, item_size);
            vRingbufferReturnItem(iface->ringbuf, data);
            // BYTEBUF ring buffers only return contiguous data — at the
            // internal wrap point we get a short read.  Pull the remaining
            // bytes (now at the head of the buffer) before zero-padding.
            if (item_size < pkt_size) {
                size_t need = pkt_size - item_size;
                size_t extra_size = 0;
                void *extra = xRingbufferReceiveUpTo(iface->ringbuf, &extra_size, 0, need);
                if (extra && extra_size > 0) {
                    memcpy(pkt_buf + item_size, extra, extra_size);
                    vRingbufferReturnItem(iface->ringbuf, extra);
                    item_size += extra_size;
                }
                if (item_size < pkt_size) {
                    memset(pkt_buf + item_size, 0, pkt_size - item_size);
                }
            }
        } else {
            memset(pkt_buf, 0, pkt_size);
        }

        xfer->isoc_packet_desc[i].num_bytes = pkt_size;
        bytes_filled += pkt_size;

        // Per-packet low-watermark check to keep refill signaling responsive
        if (iface->user_cb && !atomic_load(&iface->tx_done_pending)) {
            size_t rb_used = iface->ringbuf_size - xRingbufferGetCurFreeSize(iface->ringbuf);
            if (rb_used < iface->ringbuf_threshold) {
                atomic_store(&iface->tx_done_pending, true);
                iface->user_cb(iface, UAC2_HOST_DEVICE_EVENT_TX_DONE, iface->user_cb_arg);
            }
        }
    }

    xfer->num_bytes = bytes_filled;

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

static bool device_has_opposite_direction_stream(const uac2_iface_t *iface)
{
#if CONFIG_IDF_TARGET_ESP32S3
    bool blocked = false;

    UAC2_ENTER_CRITICAL();
    uac2_iface_t *other = NULL;
    STAILQ_FOREACH(other, &s_uac2_driver->ifaces_tailq, tailq_entry) {
        if (other == iface || other->parent != iface->parent || other->dir == iface->dir) {
            continue;
        }
        if (other->state != UAC2_IFACE_STATE_IDLE || stream_has_resources(other)) {
            blocked = true;
            break;
        }
    }
    UAC2_EXIT_CRITICAL();

    return blocked;
#else
    (void)iface;
    return false;
#endif
}

static bool fractional_playback_requires_feedback(uac2_stream_dir_t dir,
                                                  uint8_t fb_ep_addr,
                                                  uint32_t sample_rate)
{
    return dir == UAC2_STREAM_TX &&
           fb_ep_addr == 0 &&
           sample_rate > 0 &&
           (sample_rate % 1000) != 0;
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

static esp_err_t stream_abort_startup(uac2_iface_t *iface)
{
    uac2_device_t *dev = iface->parent;

    portENTER_CRITICAL(&iface->state_lock);
    iface->state = UAC2_IFACE_STATE_IDLE;
    portEXIT_CRITICAL(&iface->state_lock);

    if (!atomic_load(&dev->gone)) {
        esp_err_t si_err = ctrl_request_no_data(dev,
            USB_BM_REQUEST_TYPE_DIR_OUT | USB_BM_REQUEST_TYPE_TYPE_STANDARD | USB_BM_REQUEST_TYPE_RECIP_INTERFACE,
            USB_B_REQUEST_SET_INTERFACE, 0, iface->iface_num);
        if (si_err != ESP_OK) {
            ESP_LOGW(TAG, "SET_INTERFACE(%d, 0) failed during startup abort: %s",
                     iface->iface_num, esp_err_to_name(si_err));
        }
    }

    if (iface->interface_claimed) {
        esp_err_t rel_err = release_interface_claim(dev, iface, pdMS_TO_TICKS(2000));
        if (rel_err != ESP_OK) {
            portENTER_CRITICAL(&iface->state_lock);
            iface->state = UAC2_IFACE_STATE_ERROR;
            portEXIT_CRITICAL(&iface->state_lock);
            return rel_err;
        }
        iface->interface_claimed = false;
    }

    esp_err_t wait_err = wait_for_urbs_quiesced(iface, 2000, "Startup abort", true);
    if (wait_err != ESP_OK) {
        return wait_err;
    }

    stream_resources_free(iface);
    portENTER_CRITICAL(&iface->state_lock);
    iface->state = UAC2_IFACE_STATE_IDLE;
    portEXIT_CRITICAL(&iface->state_lock);
    return ESP_OK;
}

static void stream_flush_ringbuf(uac2_iface_t *iface)
{
    if (!iface->ringbuf) {
        return;
    }

    if (iface->dir == UAC2_STREAM_TX) {
        size_t item_size;
        void *item;
        while ((item = xRingbufferReceiveUpTo(iface->ringbuf, &item_size, 0,
                                               iface->ringbuf_size)) != NULL) {
            vRingbufferReturnItem(iface->ringbuf, item);
        }
    } else {
        size_t item_size;
        void *item;
        while ((item = xRingbufferReceiveUpTo(iface->ringbuf, &item_size, 0,
                                               iface->ringbuf_size)) != NULL) {
            vRingbufferReturnItem(iface->ringbuf, item);
        }
    }
}

static void stream_reset_runtime_state(uac2_iface_t *iface)
{
    atomic_store(&iface->consecutive_errors, 0);
    atomic_store(&iface->fb_value, 0);
    iface->fb_accumulator = 0;
    atomic_store(&iface->first_frame_us, 0);
    atomic_store(&iface->tx_done_pending, false);
}

static esp_err_t resume_rollback_to_ready(uac2_iface_t *iface,
                                          esp_err_t cause_err,
                                          const char *stage)
{
    esp_err_t cleanup_err = stream_deactivate(iface);
    if (cleanup_err == ESP_OK) {
        stream_flush_ringbuf(iface);
        stream_reset_runtime_state(iface);
        bool gone = atomic_load(&iface->parent->gone);
        portENTER_CRITICAL(&iface->state_lock);
        uac2_iface_state_t state = iface->state;
        portEXIT_CRITICAL(&iface->state_lock);
        portENTER_CRITICAL(&iface->state_lock);
        iface->state = (gone || state == UAC2_IFACE_STATE_IDLE)
            ? UAC2_IFACE_STATE_IDLE
            : UAC2_IFACE_STATE_READY;
        portEXIT_CRITICAL(&iface->state_lock);
        ESP_LOGE(TAG, "Resume failed during %s: %s", stage, esp_err_to_name(cause_err));
        return (gone || state == UAC2_IFACE_STATE_IDLE) ? ESP_ERR_INVALID_STATE : cause_err;
    }

    portENTER_CRITICAL(&iface->state_lock);
    iface->state = UAC2_IFACE_STATE_ERROR;
    portEXIT_CRITICAL(&iface->state_lock);
    ESP_LOGE(TAG, "Resume cleanup failed after %s error %s: %s",
             stage, esp_err_to_name(cause_err), esp_err_to_name(cleanup_err));
    return cleanup_err;
}

// ── Clock helpers ─────────────────────────────────────────────────

static uint8_t resolve_clock_source(const uac2_device_info_t *info)
{
    // Walk UAC2 clock topology: terminal → selector/multiplier → clock source.
    // Start from the first output terminal (playback sink) since that's the
    // primary use case. Fall back to any terminal with a clock reference.
    uint8_t entity_id = 0;
    for (int i = 0; i < info->num_terminals; i++) {
        if (!info->terminals[i].is_input && info->terminals[i].clock_source_id != 0) {
            entity_id = info->terminals[i].clock_source_id;
            break;
        }
    }
    if (entity_id == 0) {
        for (int i = 0; i < info->num_terminals; i++) {
            if (info->terminals[i].clock_source_id != 0) {
                entity_id = info->terminals[i].clock_source_id;
                break;
            }
        }
    }
    if (entity_id == 0) {
        if (info->num_clock_sources > 0) return info->clock_sources[0].clock_id;
        ESP_LOGW(TAG, "No clock source found in descriptors");
        return 0;
    }

    // Follow the chain through selectors/multipliers to the clock source.
    // Max 8 hops prevents infinite loops from malformed descriptors.
    for (int depth = 0; depth < 8; depth++) {
        // Reached a clock source — done
        for (int i = 0; i < info->num_clock_sources; i++) {
            if (info->clock_sources[i].clock_id == entity_id) {
                return entity_id;
            }
        }
        // Clock selector — follow first input pin
        // (can't query GET_CUR for active input at discovery time, no ctrl xfer yet)
        bool followed = false;
        for (int i = 0; i < info->num_clock_selectors; i++) {
            if (info->clock_selectors[i].clock_id == entity_id && info->clock_selectors[i].nr_pins > 0) {
                ESP_LOGD(TAG, "Clock walk: selector %d → source %d (pin 0)",
                         entity_id, info->clock_selectors[i].source_ids[0]);
                entity_id = info->clock_selectors[i].source_ids[0];
                followed = true;
                break;
            }
        }
        if (followed) continue;
        // Clock multiplier — follow upstream source
        for (int i = 0; i < info->num_clock_multipliers; i++) {
            if (info->clock_multipliers[i].clock_id == entity_id) {
                ESP_LOGD(TAG, "Clock walk: multiplier %d → source %d",
                         entity_id, info->clock_multipliers[i].source_id);
                entity_id = info->clock_multipliers[i].source_id;
                followed = true;
                break;
            }
        }
        if (followed) continue;
        // Entity not found in any category — broken topology
        ESP_LOGW(TAG, "Clock entity %d not found in topology", entity_id);
        break;
    }

    // Fallback if walk didn't resolve
    if (info->num_clock_sources > 0) {
        ESP_LOGW(TAG, "Clock topology walk incomplete, using first clock source");
        return info->clock_sources[0].clock_id;
    }
    ESP_LOGW(TAG, "No clock source found in descriptors");
    return 0;
}

static const uac2_terminal_t *find_terminal_by_id(const uac2_device_info_t *info, uint8_t terminal_id)
{
    for (int i = 0; i < info->num_terminals; i++) {
        if (info->terminals[i].terminal_id == terminal_id) {
            return &info->terminals[i];
        }
    }
    return NULL;
}

static const uac2_feature_unit_t *find_feature_unit_by_id(const uac2_device_info_t *info, uint8_t unit_id)
{
    for (int i = 0; i < info->num_feature_units; i++) {
        if (info->feature_units[i].unit_id == unit_id) {
            return &info->feature_units[i];
        }
    }
    return NULL;
}

static const uac2_feature_unit_t *resolve_feature_unit_for_iface(const uac2_device_info_t *info,
                                                                 uint8_t iface_num)
{
    uint8_t terminal_link = 0;
    for (int i = 0; i < info->num_as_ifaces; i++) {
        const uac2_as_iface_t *as = &info->as_ifaces[i];
        if (as->interface_num == iface_num && as->terminal_link != 0) {
            terminal_link = as->terminal_link;
            break;
        }
    }
    if (terminal_link == 0) {
        return NULL;
    }

    const uac2_terminal_t *terminal = find_terminal_by_id(info, terminal_link);
    if (!terminal) {
        return NULL;
    }

    if (terminal->is_input) {
        for (int i = 0; i < info->num_feature_units; i++) {
            if (info->feature_units[i].source_id == terminal->terminal_id) {
                return &info->feature_units[i];
            }
        }
        return NULL;
    }

    if (terminal->source_id != 0) {
        return find_feature_unit_by_id(info, terminal->source_id);
    }

    return NULL;
}

static const uac2_feature_unit_t *get_iface_feature_unit(const uac2_iface_t *iface)
{
    if (!iface || !iface->has_feature_unit) {
        return NULL;
    }

    return find_feature_unit_by_id(&iface->parent->desc_info, iface->feature_unit_id);
}

static esp_err_t validate_feature_channel(const uac2_feature_unit_t *fu, uint8_t channel,
                                          uint32_t channel_map, const char *control_name)
{
    ESP_RETURN_ON_FALSE(fu, ESP_ERR_NOT_FOUND, TAG, "Feature unit topology missing");
    ESP_RETURN_ON_FALSE(channel <= fu->nr_channels, ESP_ERR_INVALID_ARG, TAG, "Channel out of range");
    ESP_RETURN_ON_FALSE(channel_map & (1u << channel), ESP_ERR_NOT_SUPPORTED, TAG,
                        "Control unsupported on channel");
    (void)control_name;
    return ESP_OK;
}

static esp_err_t validate_range_payload_len(const char *what, uint16_t actual_len,
                                            uint16_t count, uint16_t max_count,
                                            uint8_t stride)
{
    uint16_t parsed_count = count > max_count ? max_count : count;
    uint32_t required = 2u + ((uint32_t)parsed_count * stride);
    if (actual_len < required) {
        ESP_LOGE(TAG, "%s short response: actual=%u required=%" PRIu32 " count=%u parsed=%u",
                 what, actual_len, required, count, parsed_count);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

static esp_err_t fetch_volume_range_triplet(uac2_iface_t *iface, uint8_t channel,
                                            int16_t *min_db256, int16_t *max_db256,
                                            int16_t *res_db256)
{
    uint8_t buf[2 + UAC2_MAX_VOLUME_RANGES * 6];
    uint16_t actual_len = 0;
    memset(buf, 0, sizeof(buf));

    esp_err_t err = ctrl_get_range(iface->parent, iface->feature_unit_id,
                                   UAC2_FU_VOLUME_CONTROL, channel, buf, sizeof(buf),
                                   &actual_len);
    if (err != ESP_OK) {
        return err;
    }

    uint16_t count = buf[0] | (buf[1] << 8);
    if (count == 0) {
        return ESP_ERR_NOT_FOUND;
    }

    err = validate_range_payload_len("Volume RANGE", actual_len, count,
                                     UAC2_MAX_VOLUME_RANGES, 6);
    if (err != ESP_OK) {
        return err;
    }

    int16_t parsed_min = (int16_t)(buf[2] | (buf[3] << 8));
    int16_t parsed_max = (int16_t)(buf[4] | (buf[5] << 8));
    int16_t parsed_res = (int16_t)(buf[6] | (buf[7] << 8));

    if (min_db256) *min_db256 = parsed_min;
    if (max_db256) *max_db256 = parsed_max;
    if (res_db256) *res_db256 = parsed_res;

    if (channel == 0) {
        iface->volume_min_db256 = parsed_min;
        iface->volume_max_db256 = parsed_max;
        iface->volume_res_db256 = parsed_res;
        iface->volume_range_valid = true;
    }

    return ESP_OK;
}

static esp_err_t get_volume_range_triplet(uac2_iface_t *iface, uint8_t channel,
                                          int16_t *min_db256, int16_t *max_db256,
                                          int16_t *res_db256)
{
    if (channel == 0 && iface->volume_range_valid) {
        if (min_db256) *min_db256 = iface->volume_min_db256;
        if (max_db256) *max_db256 = iface->volume_max_db256;
        if (res_db256) *res_db256 = iface->volume_res_db256;
        return ESP_OK;
    }
    return fetch_volume_range_triplet(iface, channel, min_db256, max_db256, res_db256);
}

static void iface_cache_feature_controls(uac2_iface_t *iface)
{
    const uac2_feature_unit_t *fu = resolve_feature_unit_for_iface(&iface->parent->desc_info,
                                                                   iface->iface_num);
    if (!fu) {
        return;
    }

    iface->feature_unit_id = fu->unit_id;
    iface->has_feature_unit = true;
    iface->has_mute = fu->mute_ch_map != 0;
    iface->has_volume = fu->volume_ch_map != 0;

    if (!iface->has_volume || (fu->volume_ch_map & 0x1u) == 0) {
        return;
    }

    int16_t min_db256 = 0;
    int16_t max_db256 = 0;
    int16_t res_db256 = 0;
    (void)fetch_volume_range_triplet(iface, 0, &min_db256, &max_db256, &res_db256);
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

static esp_err_t read_current_sample_rate(uac2_device_t *dev, uint32_t *sample_rate)
{
    ESP_RETURN_ON_FALSE(dev && sample_rate, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    uint8_t data[4] = {0};
    esp_err_t ret = ctrl_get_cur(dev, dev->clock_source_id, UAC2_CS_SAM_FREQ_CONTROL, 0, data, 4);
    if (ret != ESP_OK) {
        return ret;
    }

    *sample_rate = (uint32_t)data[0] | ((uint32_t)data[1] << 8)
                 | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
    return ESP_OK;
}

static esp_err_t ensure_sample_rate_applied(uac2_device_t *dev, uint32_t sample_rate)
{
    ESP_RETURN_ON_FALSE(dev, ESP_ERR_INVALID_ARG, TAG, "Invalid device");

    validate_sample_rate(dev, sample_rate);
    esp_err_t err = set_sample_rate_internal(dev, sample_rate);
    if (err == ESP_OK) {
        return ESP_OK;
    }

    uint32_t current_rate = 0;
    esp_err_t read_err = read_current_sample_rate(dev, &current_rate);
    if (read_err == ESP_OK && current_rate == sample_rate) {
        ESP_LOGW(TAG, "Sample rate SET failed but device already reports %" PRIu32 " Hz",
                 sample_rate);
        return ESP_OK;
    }

    if (read_err == ESP_OK) {
        ESP_LOGE(TAG, "Requested sample rate %" PRIu32 " Hz but device reports %" PRIu32 " Hz",
                 sample_rate, current_rate);
        return err;
    }

    ESP_LOGE(TAG, "Failed to verify sample rate %" PRIu32 " Hz after SET failure: %s",
             sample_rate, esp_err_to_name(read_err));
    return read_err;
}

static void validate_sample_rate(uac2_device_t *dev, uint32_t sample_rate)
{
    if (dev->clock_source_id == 0) return;

    uint8_t buf[2 + UAC2_MAX_SAMPLE_RATE_RANGES * 12];
    uint16_t actual_len = 0;
    memset(buf, 0, sizeof(buf));
    esp_err_t err = ctrl_get_range(dev, dev->clock_source_id,
                                   UAC2_CS_SAM_FREQ_CONTROL, 0, buf, sizeof(buf), &actual_len);
    if (err != ESP_OK) return;

    uint16_t count = buf[0] | (buf[1] << 8);
    if (count > UAC2_MAX_SAMPLE_RATE_RANGES) count = UAC2_MAX_SAMPLE_RATE_RANGES;
    if (count == 0) return;

    err = validate_range_payload_len("Sample-rate RANGE", actual_len, count,
                                     UAC2_MAX_SAMPLE_RATE_RANGES, 12);
    if (err != ESP_OK) return;

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

    // Cache addr before publishing gone=true. device_open() may tear down a
    // provisional device immediately after seeing dev->gone, so avoid touching
    // dev again after this point.
    uint8_t dev_addr = dev->addr;

    // Mark device as gone — stream_stop_internal will skip SET_INTERFACE and endpoint ops
    atomic_store(&dev->gone, true);

    // Collect interfaces to notify (don't modify the list during iteration)
    uac2_iface_t *to_notify[UAC2_MAX_AS_INTERFACES];
    int notify_count = 0;

    UAC2_ENTER_CRITICAL();
    uac2_iface_t *iface;
    STAILQ_FOREACH(iface, &s_uac2_driver->ifaces_tailq, tailq_entry) {
        if (iface->parent && iface->parent->addr == dev_addr) {
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
                         "call uac2_host_device_close() to free resources", dev_addr, iface->iface_num);
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
    uac2_device_t *dev = heap_caps_calloc(1, sizeof(uac2_device_t), MALLOC_CAP_DEFAULT);
    if (!dev) {
        return ESP_ERR_NO_MEM;
    }

    usb_device_handle_t dev_hdl;
    esp_err_t err = usb_host_device_open(s_uac2_driver->client_handle, addr, &dev_hdl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open USB device addr %d: %s", addr, esp_err_to_name(err));
        heap_caps_free(dev);
        return err;
    }

    dev->dev_hdl = dev_hdl;
    dev->addr = addr;
    dev->opened_cnt = 0;
    atomic_init(&dev->gone, false);

    // Publish the device immediately so DEV_GONE cannot be missed while open
    // is still parsing descriptors and allocating control buffers.
    UAC2_ENTER_CRITICAL();
    STAILQ_INSERT_TAIL(&s_uac2_driver->devices_tailq, dev, tailq_entry);
    UAC2_EXIT_CRITICAL();

    const usb_config_desc_t *config_desc;
    err = usb_host_get_active_config_descriptor(dev_hdl, &config_desc);
    if (err != ESP_OK) {
        goto fail;
    }

    // Reject low-speed
    usb_device_info_t usb_info;
    err = usb_host_device_info(dev_hdl, &usb_info);
    if (err != ESP_OK) {
        goto fail;
    }
    if (usb_info.speed == USB_SPEED_LOW) {
        ESP_LOGE(TAG, "Low-speed devices do not support isochronous transfers");
        err = ESP_ERR_NOT_SUPPORTED;
        goto fail;
    }

    // Parse descriptors
    bool is_uac2 = uac2_parse_config_descriptor(
        (const uint8_t *)config_desc, config_desc->wTotalLength, &dev->desc_info);
    if (!is_uac2) {
        err = ESP_ERR_NOT_SUPPORTED;
        goto fail;
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

    dev->ac_iface_num = dev->desc_info.ac_iface_num;
    dev->clock_source_id = resolve_clock_source(&dev->desc_info);

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

    ESP_LOGI(TAG, "UAC2 device opened: addr=%d VID=0x%04X PID=0x%04X \"%s\"",
             addr, dev->desc_info.vid, dev->desc_info.pid,
             dev->desc_info.product[0] ? dev->desc_info.product : "Unknown");

    *out_dev = dev;
    return ESP_OK;

fail:
    device_destroy(dev);
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

    driver->lifecycle_mutex = xSemaphoreCreateMutex();
    if (!driver->lifecycle_mutex) {
        vSemaphoreDelete(driver->all_events_handled);
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
        vSemaphoreDelete(driver->lifecycle_mutex);
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
            vSemaphoreDelete(driver->lifecycle_mutex);
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
    uac2_driver_t *driver = s_uac2_driver;
    bool uninstalling = driver && driver->end_client_event_handling;
    UAC2_EXIT_CRITICAL();
    if (!driver || uninstalling) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!driver->lifecycle_mutex ||
        xSemaphoreTake(driver->lifecycle_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    UAC2_ENTER_CRITICAL();
    if (driver != s_uac2_driver) {
        UAC2_EXIT_CRITICAL();
        xSemaphoreGive(driver->lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    if (!STAILQ_EMPTY(&driver->devices_tailq) ||
        !STAILQ_EMPTY(&driver->ifaces_tailq)) {
        UAC2_EXIT_CRITICAL();
        xSemaphoreGive(driver->lifecycle_mutex);
        ESP_LOGE(TAG, "Cannot uninstall: devices/interfaces still open");
        return ESP_ERR_INVALID_STATE;
    }
    driver->end_client_event_handling = true;
    UAC2_EXIT_CRITICAL();
    xSemaphoreGive(driver->lifecycle_mutex);

    if (driver->event_handling_started) {
        esp_err_t unblock_err = usb_host_client_unblock(driver->client_handle);
        if (unblock_err != ESP_OK) {
            ESP_LOGE(TAG, "client_unblock failed: %s", esp_err_to_name(unblock_err));
        }
        xSemaphoreTake(driver->all_events_handled, portMAX_DELAY);
    }
    vSemaphoreDelete(driver->all_events_handled);
    esp_err_t dereg_err = usb_host_client_deregister(driver->client_handle);
    if (dereg_err != ESP_OK) {
        ESP_LOGE(TAG, "client_deregister failed: %s", esp_err_to_name(dereg_err));
    }
    vSemaphoreDelete(driver->lifecycle_mutex);
    UAC2_ENTER_CRITICAL();
    if (s_uac2_driver == driver) {
        s_uac2_driver = NULL;
    }
    UAC2_EXIT_CRITICAL();
    heap_caps_free(driver);
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

    esp_err_t err = driver_lifecycle_lock();
    if (err != ESP_OK) return err;
    if (s_uac2_driver->end_client_event_handling) {
        driver_lifecycle_unlock();
        return ESP_ERR_INVALID_STATE;
    }

    // Reject if already open — caller must close first
    uac2_iface_t *existing = get_iface_by_addr(config->addr, config->iface_num);
    if (existing) {
        ESP_LOGE(TAG, "Interface addr=%d iface=%d already open", config->addr, config->iface_num);
        driver_lifecycle_unlock();
        return ESP_ERR_INVALID_STATE;
    }

    // Get or create physical device
    uac2_device_t *dev = get_device_by_addr(config->addr);
    if (dev && atomic_load(&dev->gone) && dev->opened_cnt == 0) {
        device_destroy(dev);
        dev = NULL;
    }
    if (!dev) {
        err = device_create(config->addr, &dev);
        if (err != ESP_OK) {
            driver_lifecycle_unlock();
            return err;
        }
    }
    if (atomic_load(&dev->gone)) {
        if (dev->opened_cnt == 0) {
            device_destroy(dev);
        }
        driver_lifecycle_unlock();
        return ESP_ERR_INVALID_STATE;
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
        driver_lifecycle_unlock();
        return ESP_ERR_NOT_FOUND;
    }

    // Create interface
    uac2_iface_t *iface = heap_caps_calloc(1, sizeof(uac2_iface_t), MALLOC_CAP_DEFAULT);
    if (!iface) {
        UAC2_ENTER_CRITICAL();
        bool orphan = (dev->opened_cnt == 0);
        UAC2_EXIT_CRITICAL();
        if (orphan) device_destroy(dev);
        driver_lifecycle_unlock();
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
    atomic_init(&iface->closing, false);
    atomic_init(&iface->io_users, 0);
    iface->interface_claimed = false;

    iface->api_mutex = xSemaphoreCreateMutex();
    if (!iface->api_mutex) {
        heap_caps_free(iface);
        UAC2_ENTER_CRITICAL();
        bool orphan = (dev->opened_cnt == 0);
        UAC2_EXIT_CRITICAL();
        if (orphan) device_destroy(dev);
        driver_lifecycle_unlock();
        return ESP_ERR_NO_MEM;
    }

    if (atomic_load(&dev->gone)) {
        vSemaphoreDelete(iface->api_mutex);
        heap_caps_free(iface);
        if (dev->opened_cnt == 0) {
            device_destroy(dev);
        }
        driver_lifecycle_unlock();
        return ESP_ERR_INVALID_STATE;
    }

    iface_cache_feature_controls(iface);

    // Add to list and increment refcount
    bool dev_gone = false;
    UAC2_ENTER_CRITICAL();
    dev_gone = atomic_load(&dev->gone);
    if (!dev_gone) {
        STAILQ_INSERT_TAIL(&s_uac2_driver->ifaces_tailq, iface, tailq_entry);
        dev->opened_cnt++;
    }
    UAC2_EXIT_CRITICAL();
    if (dev_gone) {
        vSemaphoreDelete(iface->api_mutex);
        heap_caps_free(iface);
        if (dev->opened_cnt == 0) {
            device_destroy(dev);
        }
        driver_lifecycle_unlock();
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Opened %s interface: addr=%d iface=%d (device refs=%d)",
             dir == UAC2_STREAM_TX ? "TX" : "RX",
             config->addr, config->iface_num, dev->opened_cnt);
    if (iface->has_feature_unit) {
        ESP_LOGI(TAG, "Interface addr=%d iface=%d controls: FU=%d mute=%s volume=%s",
                 config->addr, config->iface_num, iface->feature_unit_id,
                 iface->has_mute ? "yes" : "no", iface->has_volume ? "yes" : "no");
        if (iface->volume_range_valid) {
            ESP_LOGI(TAG, "Interface addr=%d iface=%d volume range: %.2f to %.2f dB (res %.4f dB)",
                     config->addr, config->iface_num,
                     iface->volume_min_db256 / 256.0, iface->volume_max_db256 / 256.0,
                     iface->volume_res_db256 / 256.0);
        }
    }

    *out_handle = iface;
    driver_lifecycle_unlock();
    return ESP_OK;
}

esp_err_t uac2_host_device_close(uac2_host_device_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    esp_err_t err = driver_lifecycle_lock();
    if (err != ESP_OK) return err;
    if (s_uac2_driver->end_client_event_handling) {
        driver_lifecycle_unlock();
        return ESP_ERR_INVALID_STATE;
    }

    uac2_iface_t *iface = get_iface_by_handle(handle);
    if (!iface) {
        driver_lifecycle_unlock();
        ESP_LOGE(TAG, "Handle not in list (already closed?)");
        return ESP_ERR_INVALID_ARG;
    }

    atomic_store(&iface->closing, true);

    // Take api_mutex to serialize with any in-progress API calls
    esp_err_t lock_err = api_lock_allow_closing(iface);
    if (lock_err != ESP_OK) {
        atomic_store(&iface->closing, false);
        ESP_LOGE(TAG, "device_close: api_mutex timeout");
        driver_lifecycle_unlock();
        return lock_err;
    }

    // Re-validate after acquiring lock — a concurrent close may have freed this
    if (!is_interface_in_list(iface)) {
        api_unlock(iface);
        driver_lifecycle_unlock();
        return ESP_ERR_INVALID_ARG;
    }

    // Stop active stream or clean up disconnect-leftover resources.
    esp_err_t stop_err = ESP_OK;
    if (iface->state != UAC2_IFACE_STATE_IDLE || stream_has_resources(iface)) {
        stop_err = stream_stop_internal(iface);
    }

    api_unlock(iface);
    if (stop_err != ESP_OK) {
        ESP_LOGE(TAG, "device_close: stream teardown incomplete: %s", esp_err_to_name(stop_err));
        atomic_store(&iface->closing, false);
        driver_lifecycle_unlock();
        return stop_err;
    }

    esp_err_t io_err = wait_for_iface_io_quiesced(iface, pdMS_TO_TICKS(2000));
    if (io_err != ESP_OK) {
        ESP_LOGE(TAG, "device_close: timed out waiting for pending I/O");
        atomic_store(&iface->closing, false);
        driver_lifecycle_unlock();
        return io_err;
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

    driver_lifecycle_unlock();
    return ESP_OK;
}

esp_err_t uac2_host_device_get_info(uac2_host_device_handle_t handle,
                                    uac2_device_info_t *info)
{
    ESP_RETURN_ON_FALSE(handle && info, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    uac2_iface_t *iface = NULL;
    esp_err_t err = acquire_iface_runtime_ref(handle, &iface);
    ESP_RETURN_ON_FALSE(err == ESP_OK, err, TAG, "Invalid handle");
    *info = iface->parent->desc_info;
    release_iface_io_ref(iface);
    return ESP_OK;
}

esp_err_t uac2_host_get_device_alt_param(uac2_host_device_handle_t handle,
                                         uint8_t alt,
                                         uac2_host_dev_alt_param_t *param)
{
    ESP_RETURN_ON_FALSE(handle && param, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    ESP_RETURN_ON_FALSE(alt > 0, ESP_ERR_INVALID_ARG, TAG, "Alt setting must be >= 1");
    uac2_iface_t *iface = NULL;
    esp_err_t err = acquire_iface_runtime_ref(handle, &iface);
    ESP_RETURN_ON_FALSE(err == ESP_OK, err, TAG, "Invalid handle");

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
            release_iface_io_ref(iface);
            return ESP_OK;
        }
    }
    release_iface_io_ref(iface);
    return ESP_ERR_NOT_FOUND;
}

// ── Public API: Clock control ─────────────────────────────────────

esp_err_t uac2_host_device_get_sample_rate(uac2_host_device_handle_t handle,
                                           uint32_t *sample_rate)
{
    ESP_RETURN_ON_FALSE(handle && sample_rate, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    uac2_iface_t *iface = NULL;
    esp_err_t ret = acquire_locked_iface(handle, &iface);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "Invalid handle");
    uac2_device_t *dev = iface->parent;
    if (dev->clock_source_id == 0) {
        release_locked_iface(iface);
        return ESP_ERR_NOT_SUPPORTED;
    }

    ret = read_current_sample_rate(dev, sample_rate);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Current sample rate: %" PRIu32 " Hz", *sample_rate);
    }
    release_locked_iface(iface);
    return ret;
}

esp_err_t uac2_host_device_set_sample_rate(uac2_host_device_handle_t handle,
                                           uint32_t sample_rate)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uac2_iface_t *iface = NULL;
    esp_err_t ret = acquire_locked_iface(handle, &iface);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "Invalid handle");
    uac2_device_t *dev = iface->parent;
    if (dev->clock_source_id == 0) {
        release_locked_iface(iface);
        return ESP_ERR_NOT_SUPPORTED;
    }
    ret = set_sample_rate_internal(dev, sample_rate);
    release_locked_iface(iface);
    return ret;
}

esp_err_t uac2_host_device_get_sample_rate_range(uac2_host_device_handle_t handle,
                                                 uac2_sample_rate_range_t *ranges,
                                                 uint8_t *num_ranges)
{
    ESP_RETURN_ON_FALSE(handle && ranges && num_ranges, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    uac2_iface_t *iface = NULL;
    esp_err_t ret = acquire_locked_iface(handle, &iface);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "Invalid handle");
    uac2_device_t *dev = iface->parent;
    if (dev->clock_source_id == 0) {
        release_locked_iface(iface);
        return ESP_ERR_NOT_SUPPORTED;
    }

    uint8_t buf[2 + UAC2_MAX_SAMPLE_RATE_RANGES * 12];
    uint16_t actual_len = 0;
    memset(buf, 0, sizeof(buf));
    ret = ctrl_get_range(dev, dev->clock_source_id, UAC2_CS_SAM_FREQ_CONTROL, 0,
                         buf, sizeof(buf), &actual_len);
    if (ret != ESP_OK) {
        release_locked_iface(iface);
        return ret;
    }

    uint16_t count = buf[0] | (buf[1] << 8);
    ret = validate_range_payload_len("Sample-rate RANGE", actual_len, count,
                                     UAC2_MAX_SAMPLE_RATE_RANGES, 12);
    if (ret != ESP_OK) {
        release_locked_iface(iface);
        return ret;
    }
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
    release_locked_iface(iface);
    return ESP_OK;
}

esp_err_t uac2_host_device_get_clock_valid(uac2_host_device_handle_t handle, bool *valid)
{
    ESP_RETURN_ON_FALSE(handle && valid, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    uac2_iface_t *iface = NULL;
    esp_err_t ret = acquire_locked_iface(handle, &iface);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "Invalid handle");
    uac2_device_t *dev = iface->parent;
    if (dev->clock_source_id == 0) {
        release_locked_iface(iface);
        return ESP_ERR_NOT_SUPPORTED;
    }

    uint8_t data = 0;
    ret = ctrl_get_cur(dev, dev->clock_source_id, UAC2_CS_CLOCK_VALID_CONTROL, 0, &data, 1);
    if (ret == ESP_OK) {
        *valid = (data & 0x01) != 0;
    }
    release_locked_iface(iface);
    return ret;
}

// ── Public API: Streaming ─────────────────────────────────────────

esp_err_t uac2_host_device_start(uac2_host_device_handle_t handle,
                                 const uac2_host_stream_config_t *config)
{
    ESP_RETURN_ON_FALSE(handle && config, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    uac2_iface_t *iface = NULL;
    esp_err_t ret = acquire_locked_iface(handle, &iface);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "Invalid handle");

    if (iface->state != UAC2_IFACE_STATE_IDLE) {
        ESP_LOGE(TAG, "Stream already active");
        release_locked_iface(iface);
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
        release_locked_iface(iface);
        return ESP_ERR_NOT_FOUND;
    }

    if (fractional_playback_requires_feedback(iface->dir, as->fb_ep_addr, config->sample_freq)) {
        ESP_LOGE(TAG, "Playback %.2f kHz requires a feedback endpoint on this alt setting",
                 config->sample_freq / 1000.0);
        release_locked_iface(iface);
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (device_has_opposite_direction_stream(iface)) {
        ESP_LOGE(TAG, "ESP32-S3 does not support simultaneous TX and RX audio streams");
        release_locked_iface(iface);
        return ESP_ERR_NOT_SUPPORTED;
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
    if (err != ESP_OK) {
        release_locked_iface(iface);
        return err;
    }

    // Claim interface
    err = usb_host_interface_claim(s_uac2_driver->client_handle, dev->dev_hdl,
                                   iface->iface_num, iface->alt_setting);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to claim iface %d alt %d: %s",
                 iface->iface_num, iface->alt_setting, esp_err_to_name(err));
        stream_resources_free(iface);
        release_locked_iface(iface);
        return err;
    }
    iface->interface_claimed = true;

    // SET_INTERFACE to activate endpoints on device
    err = ctrl_request_no_data(dev,
        USB_BM_REQUEST_TYPE_DIR_OUT | USB_BM_REQUEST_TYPE_TYPE_STANDARD | USB_BM_REQUEST_TYPE_RECIP_INTERFACE,
        USB_B_REQUEST_SET_INTERFACE, iface->alt_setting, iface->iface_num);
    if (err != ESP_OK) {
        esp_err_t cleanup_err = stream_abort_startup(iface);
        if (cleanup_err != ESP_OK) {
            ESP_LOGE(TAG, "Startup abort failed after SET_INTERFACE error %s: %s",
                     esp_err_to_name(err), esp_err_to_name(cleanup_err));
            err = cleanup_err;
        }
        release_locked_iface(iface);
        return err;
    }

    iface->state = UAC2_IFACE_STATE_READY;
    err = ensure_iface_device_available(iface, UAC2_IFACE_STATE_READY,
                                        "device_start after SET_INTERFACE");
    if (err != ESP_OK) {
        esp_err_t cleanup_err = stream_abort_startup(iface);
        if (cleanup_err != ESP_OK) {
            ESP_LOGE(TAG, "Startup abort failed after disconnect during SET_INTERFACE: %s",
                     esp_err_to_name(cleanup_err));
            err = cleanup_err;
        }
        release_locked_iface(iface);
        return err;
    }

    // Validate and set sample rate
    if (dev->clock_source_id != 0 && config->sample_freq > 0) {
        err = ensure_sample_rate_applied(dev, config->sample_freq);
        if (err != ESP_OK) {
            esp_err_t cleanup_err = stream_abort_startup(iface);
            if (cleanup_err != ESP_OK) {
                ESP_LOGE(TAG, "Startup abort failed after sample-rate error %s: %s",
                         esp_err_to_name(err), esp_err_to_name(cleanup_err));
                err = cleanup_err;
            }
            release_locked_iface(iface);
            return err;
        }
        err = ensure_iface_device_available(iface, UAC2_IFACE_STATE_READY,
                                            "device_start after sample-rate setup");
        if (err != ESP_OK) {
            esp_err_t cleanup_err = stream_abort_startup(iface);
            if (cleanup_err != ESP_OK) {
                ESP_LOGE(TAG, "Startup abort failed after disconnect during sample-rate setup: %s",
                         esp_err_to_name(cleanup_err));
                err = cleanup_err;
            }
            release_locked_iface(iface);
            return err;
        }
    }

    // Check for suspend-after-start flag
    if (config->flags & UAC2_FLAG_STREAM_SUSPEND_AFTER_START) {
        ESP_LOGI(TAG, "%s stream started (suspended)", iface->dir == UAC2_STREAM_TX ? "TX" : "RX");
        release_locked_iface(iface);
        return ESP_OK;
    }

    // Submit URBs
    err = stream_submit_urbs(iface);
    if (err != ESP_OK) {
        esp_err_t cleanup_err = stream_abort_startup(iface);
        if (cleanup_err != ESP_OK) {
            ESP_LOGE(TAG, "Startup abort failed after submit error %s: %s",
                     esp_err_to_name(err), esp_err_to_name(cleanup_err));
            err = cleanup_err;
        }
        release_locked_iface(iface);
        return err;
    }
    err = ensure_iface_device_available(iface, UAC2_IFACE_STATE_ACTIVE,
                                        "device_start after URB submit");
    if (err != ESP_OK) {
        esp_err_t cleanup_err = stream_abort_startup(iface);
        if (cleanup_err != ESP_OK) {
            ESP_LOGE(TAG, "Startup abort failed after disconnect during URB submit: %s",
                     esp_err_to_name(cleanup_err));
            err = cleanup_err;
        }
        release_locked_iface(iface);
        return err;
    }

    ESP_LOGI(TAG, "%s stream started (pkt_size=%d, ringbuf=%" PRIu32 ")",
             iface->dir == UAC2_STREAM_TX ? "TX" : "RX",
             iface->packet_size, iface->ringbuf_size);
    release_locked_iface(iface);
    return ESP_OK;
}

// Internal stream_stop — caller must hold api_mutex (or be in close path)
static esp_err_t stream_stop_internal(uac2_iface_t *iface)
{
    // If state is IDLE and no resources allocated, nothing to do.
    // But if state is IDLE with resources still present (disconnect set state
    // to IDLE without freeing), we must still clean up.
    if (iface->state == UAC2_IFACE_STATE_IDLE && !stream_has_resources(iface)) return ESP_OK;

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

    // Release interface — retry for ESP-IDF bug #17707 even after DEV_GONE.
    // The physical device may be gone, but the host library still needs the
    // client-side interface claim released so the bus can reach ALL_FREE and
    // subsequent reconnects can enumerate cleanly.
    if (iface->interface_claimed) {
        esp_err_t rel_err = release_interface_claim(dev, iface, pdMS_TO_TICKS(2000));
        if (rel_err != ESP_OK) {
            return rel_err;
        }
        iface->interface_claimed = false;
    }

    esp_err_t wait_err = wait_for_urbs_quiesced(iface, 2000, "Stream stop", true);
    if (wait_err != ESP_OK) {
        return wait_err;
    }

    stream_resources_free(iface);
    ESP_LOGI(TAG, "%s stream stopped", iface->dir == UAC2_STREAM_TX ? "TX" : "RX");
    return ESP_OK;
}

esp_err_t uac2_host_device_stop(uac2_host_device_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uac2_iface_t *iface = NULL;
    esp_err_t ret = acquire_locked_iface(handle, &iface);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "Invalid handle");
    ret = stream_stop_internal(iface);
    release_locked_iface(iface);
    return ret;
}

esp_err_t uac2_host_device_write(uac2_host_device_handle_t handle,
                                 const uint8_t *data, uint32_t size,
                                 uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(handle && data && size > 0, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    uac2_iface_t *iface = acquire_iface_io_ref(handle);
    ESP_RETURN_ON_FALSE(iface, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    if (!iface->ringbuf || iface->state != UAC2_IFACE_STATE_ACTIVE) {
        release_iface_io_ref(iface);
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

    release_iface_io_ref(iface);
    if (!still_active) return ESP_ERR_INVALID_STATE;
    return ok == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t uac2_host_device_read(uac2_host_device_handle_t handle,
                                uint8_t *data, uint32_t size,
                                uint32_t *bytes_read,
                                uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(handle && data && bytes_read && size > 0, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    uac2_iface_t *iface = acquire_iface_io_ref(handle);
    ESP_RETURN_ON_FALSE(iface, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");

    if (!iface->ringbuf || iface->state != UAC2_IFACE_STATE_ACTIVE) {
        release_iface_io_ref(iface);
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
    release_iface_io_ref(iface);
    return ret;
}

int64_t uac2_host_device_get_start_time(uac2_host_device_handle_t handle)
{
    if (!handle) return 0;
    uac2_iface_t *iface = acquire_iface_io_ref(handle);
    if (!iface) return 0;
    int64_t start = atomic_load(&iface->first_frame_us);
    release_iface_io_ref(iface);
    return start;
}

uint32_t uac2_host_device_get_feedback(uac2_host_device_handle_t handle)
{
    if (!handle) return 0;
    uac2_iface_t *iface = acquire_iface_io_ref(handle);
    if (!iface) return 0;
    uint32_t feedback = atomic_load(&iface->fb_value);
    release_iface_io_ref(iface);
    return feedback;
}

// ── Public API: Volume / Mute ─────────────────────────────────────

esp_err_t uac2_host_device_set_mute(uac2_host_device_handle_t handle,
                                    uint8_t channel, bool mute)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uac2_iface_t *iface = NULL;
    esp_err_t ret = acquire_locked_iface(handle, &iface);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "Invalid handle");
    uac2_device_t *dev = iface->parent;
    if (!iface->has_feature_unit) {
        release_locked_iface(iface);
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (!iface->has_mute) {
        release_locked_iface(iface);
        return ESP_ERR_NOT_SUPPORTED;
    }
    const uac2_feature_unit_t *fu = get_iface_feature_unit(iface);
    ret = validate_feature_channel(fu, channel, fu ? fu->mute_ch_map : 0, "Mute");
    if (ret != ESP_OK) {
        release_locked_iface(iface);
        return ret;
    }
    uint8_t data = mute ? 1 : 0;
    ret = ctrl_set_cur(dev, iface->feature_unit_id, UAC2_FU_MUTE_CONTROL, channel, &data, 1);
    release_locked_iface(iface);
    return ret;
}

esp_err_t uac2_host_device_get_mute(uac2_host_device_handle_t handle,
                                    uint8_t channel, bool *mute)
{
    ESP_RETURN_ON_FALSE(handle && mute, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    uac2_iface_t *iface = NULL;
    esp_err_t ret = acquire_locked_iface(handle, &iface);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "Invalid handle");
    uac2_device_t *dev = iface->parent;
    if (!iface->has_feature_unit) {
        release_locked_iface(iface);
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (!iface->has_mute) {
        release_locked_iface(iface);
        return ESP_ERR_NOT_SUPPORTED;
    }
    const uac2_feature_unit_t *fu = get_iface_feature_unit(iface);
    ret = validate_feature_channel(fu, channel, fu ? fu->mute_ch_map : 0, "Mute");
    if (ret != ESP_OK) {
        release_locked_iface(iface);
        return ret;
    }
    uint8_t data = 0;
    ret = ctrl_get_cur(dev, iface->feature_unit_id, UAC2_FU_MUTE_CONTROL, channel, &data, 1);
    if (ret == ESP_OK) *mute = (data != 0);
    release_locked_iface(iface);
    return ret;
}

esp_err_t uac2_host_device_set_volume(uac2_host_device_handle_t handle,
                                      uint8_t channel, int16_t volume_db256)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uac2_iface_t *iface = NULL;
    esp_err_t ret = acquire_locked_iface(handle, &iface);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "Invalid handle");
    uac2_device_t *dev = iface->parent;
    if (!iface->has_feature_unit) {
        release_locked_iface(iface);
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (!iface->has_volume) {
        release_locked_iface(iface);
        return ESP_ERR_NOT_SUPPORTED;
    }
    const uac2_feature_unit_t *fu = get_iface_feature_unit(iface);
    ret = validate_feature_channel(fu, channel, fu ? fu->volume_ch_map : 0, "Volume");
    if (ret != ESP_OK) {
        release_locked_iface(iface);
        return ret;
    }
    int16_t min_db256 = 0;
    int16_t max_db256 = 0;
    ret = get_volume_range_triplet(iface, channel, &min_db256, &max_db256, NULL);
    if (ret == ESP_OK) {
        if (volume_db256 < min_db256 || volume_db256 > max_db256) {
            ESP_LOGE(TAG, "Volume %.2f dB out of range [%.2f, %.2f]",
                     volume_db256 / 256.0, min_db256 / 256.0, max_db256 / 256.0);
            release_locked_iface(iface);
            return ESP_ERR_INVALID_ARG;
        }
    } else if (ret != ESP_ERR_NOT_FOUND) {
        release_locked_iface(iface);
        return ret;
    }
    uint8_t data[2] = { (uint8_t)(volume_db256 & 0xFF), (uint8_t)((volume_db256 >> 8) & 0xFF) };
    ret = ctrl_set_cur(dev, iface->feature_unit_id, UAC2_FU_VOLUME_CONTROL, channel, data, 2);
    release_locked_iface(iface);
    return ret;
}

esp_err_t uac2_host_device_get_volume(uac2_host_device_handle_t handle,
                                      uint8_t channel, int16_t *volume_db256)
{
    ESP_RETURN_ON_FALSE(handle && volume_db256, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    uac2_iface_t *iface = NULL;
    esp_err_t ret = acquire_locked_iface(handle, &iface);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "Invalid handle");
    uac2_device_t *dev = iface->parent;
    if (!iface->has_feature_unit) {
        release_locked_iface(iface);
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (!iface->has_volume) {
        release_locked_iface(iface);
        return ESP_ERR_NOT_SUPPORTED;
    }
    const uac2_feature_unit_t *fu = get_iface_feature_unit(iface);
    ret = validate_feature_channel(fu, channel, fu ? fu->volume_ch_map : 0, "Volume");
    if (ret != ESP_OK) {
        release_locked_iface(iface);
        return ret;
    }
    uint8_t data[2] = {0};
    ret = ctrl_get_cur(dev, iface->feature_unit_id, UAC2_FU_VOLUME_CONTROL, channel, data, 2);
    if (ret == ESP_OK) *volume_db256 = (int16_t)(data[0] | (data[1] << 8));
    release_locked_iface(iface);
    return ret;
}

esp_err_t uac2_host_device_get_volume_range(uac2_host_device_handle_t handle,
                                            uint8_t channel,
                                            uac2_volume_range_t *ranges,
                                            uint8_t *num_ranges)
{
    ESP_RETURN_ON_FALSE(handle && ranges && num_ranges, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    uac2_iface_t *iface = NULL;
    esp_err_t ret = acquire_locked_iface(handle, &iface);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "Invalid handle");
    uac2_device_t *dev = iface->parent;
    if (!iface->has_feature_unit) {
        release_locked_iface(iface);
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (!iface->has_volume) {
        release_locked_iface(iface);
        return ESP_ERR_NOT_SUPPORTED;
    }
    const uac2_feature_unit_t *fu = get_iface_feature_unit(iface);
    ret = validate_feature_channel(fu, channel, fu ? fu->volume_ch_map : 0, "Volume");
    if (ret != ESP_OK) {
        release_locked_iface(iface);
        return ret;
    }

    uint8_t buf[2 + UAC2_MAX_VOLUME_RANGES * 6];
    uint16_t actual_len = 0;
    memset(buf, 0, sizeof(buf));
    ret = ctrl_get_range(dev, iface->feature_unit_id, UAC2_FU_VOLUME_CONTROL, channel,
                         buf, sizeof(buf), &actual_len);
    if (ret != ESP_OK) {
        release_locked_iface(iface);
        return ret;
    }

    uint16_t count = buf[0] | (buf[1] << 8);
    ret = validate_range_payload_len("Volume RANGE", actual_len, count,
                                     UAC2_MAX_VOLUME_RANGES, 6);
    if (ret != ESP_OK) {
        release_locked_iface(iface);
        return ret;
    }
    if (count > UAC2_MAX_VOLUME_RANGES) count = UAC2_MAX_VOLUME_RANGES;
    for (int i = 0; i < count; i++) {
        const uint8_t *p = buf + 2 + (i * 6);
        ranges[i].min = (int16_t)(p[0] | (p[1] << 8));
        ranges[i].max = (int16_t)(p[2] | (p[3] << 8));
        ranges[i].res = (int16_t)(p[4] | (p[5] << 8));
    }
    *num_ranges = (uint8_t)count;
    if (channel == 0 && count > 0) {
        iface->volume_min_db256 = ranges[0].min;
        iface->volume_max_db256 = ranges[0].max;
        iface->volume_res_db256 = ranges[0].res;
        iface->volume_range_valid = true;
    }
    release_locked_iface(iface);
    return ESP_OK;
}

esp_err_t uac2_host_device_set_volume_percent(uac2_host_device_handle_t handle,
                                              uint8_t channel, uint8_t percent)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    ESP_RETURN_ON_FALSE(percent <= 100, ESP_ERR_INVALID_ARG, TAG, "Percent must be 0-100");
    uac2_iface_t *iface = NULL;
    esp_err_t err = acquire_locked_iface(handle, &iface);
    ESP_RETURN_ON_FALSE(err == ESP_OK, err, TAG, "Invalid handle");
    if (!iface->has_feature_unit || !iface->has_volume) {
        release_locked_iface(iface);
        return ESP_ERR_NOT_SUPPORTED;
    }
    const uac2_feature_unit_t *fu = get_iface_feature_unit(iface);
    err = validate_feature_channel(fu, channel, fu ? fu->volume_ch_map : 0, "Volume");
    if (err != ESP_OK) {
        release_locked_iface(iface);
        return err;
    }
    int16_t min_db256 = 0;
    int16_t max_db256 = 0;
    err = get_volume_range_triplet(iface, channel, &min_db256, &max_db256, NULL);
    if (err != ESP_OK) {
        release_locked_iface(iface);
        return err == ESP_ERR_NOT_FOUND ? ESP_ERR_NOT_SUPPORTED : err;
    }
    int16_t db256 = min_db256 +
        (int16_t)(((int32_t)(max_db256 - min_db256) * percent) / 100);
    uac2_device_t *dev = iface->parent;
    uint8_t data[2] = { (uint8_t)(db256 & 0xFF), (uint8_t)((db256 >> 8) & 0xFF) };
    err = ctrl_set_cur(dev, iface->feature_unit_id, UAC2_FU_VOLUME_CONTROL, channel, data, 2);
    release_locked_iface(iface);
    return err;
}

esp_err_t uac2_host_device_get_volume_percent(uac2_host_device_handle_t handle,
                                              uint8_t channel, uint8_t *percent)
{
    ESP_RETURN_ON_FALSE(handle && percent, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    uac2_iface_t *iface = NULL;
    esp_err_t err = acquire_locked_iface(handle, &iface);
    ESP_RETURN_ON_FALSE(err == ESP_OK, err, TAG, "Invalid handle");
    if (!iface->has_feature_unit || !iface->has_volume) {
        release_locked_iface(iface);
        return ESP_ERR_NOT_SUPPORTED;
    }
    const uac2_feature_unit_t *fu = get_iface_feature_unit(iface);
    err = validate_feature_channel(fu, channel, fu ? fu->volume_ch_map : 0, "Volume");
    if (err != ESP_OK) {
        release_locked_iface(iface);
        return err;
    }
    int16_t min_db256 = 0;
    int16_t max_db256 = 0;
    err = get_volume_range_triplet(iface, channel, &min_db256, &max_db256, NULL);
    if (err != ESP_OK) {
        release_locked_iface(iface);
        return err == ESP_ERR_NOT_FOUND ? ESP_ERR_NOT_SUPPORTED : err;
    }
    uint8_t data[2] = {0};
    err = ctrl_get_cur(iface->parent, iface->feature_unit_id, UAC2_FU_VOLUME_CONTROL, channel, data, 2);
    if (err != ESP_OK) {
        release_locked_iface(iface);
        return err;
    }
    int16_t db256 = (int16_t)(data[0] | (data[1] << 8));
    int32_t range = max_db256 - min_db256;
    if (range <= 0) {
        *percent = 0;
    } else {
        int32_t pct = ((int32_t)(db256 - min_db256) * 100) / range;
        if (pct < 0) pct = 0;
        if (pct > 100) pct = 100;
        *percent = (uint8_t)pct;
    }
    release_locked_iface(iface);
    return ESP_OK;
}

esp_err_t uac2_host_device_set_volume_all_channels(uac2_host_device_handle_t handle,
                                                   int16_t volume_db256)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uac2_iface_t *iface = NULL;
    esp_err_t err = acquire_iface_runtime_ref(handle, &iface);
    ESP_RETURN_ON_FALSE(err == ESP_OK, err, TAG, "Invalid handle");
    if (!iface->has_feature_unit) {
        release_iface_io_ref(iface);
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (!iface->has_volume) {
        release_iface_io_ref(iface);
        return ESP_ERR_NOT_SUPPORTED;
    }

    const uac2_feature_unit_t *fu = get_iface_feature_unit(iface);
    if (!fu) {
        release_iface_io_ref(iface);
        return ESP_ERR_NOT_FOUND;
    }
    uint32_t ch_map = fu->volume_ch_map;
    uint8_t nr_channels = fu->nr_channels;
    release_iface_io_ref(iface);

    for (uint8_t ch = 0; ch <= nr_channels && ch < 32; ch++) {
        if (ch_map & (1u << ch)) {
            err = uac2_host_device_set_volume(handle, ch, volume_db256);
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
    uac2_iface_t *iface = NULL;
    esp_err_t err = acquire_iface_runtime_ref(handle, &iface);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "print_info: invalid handle");
        return;
    }
    uac2_device_t *dev = iface->parent;

    bool locked = (xSemaphoreTake(iface->api_mutex, pdMS_TO_TICKS(500)) == pdTRUE);

    uac2_log_device_info(&dev->desc_info);

    ESP_LOGI(TAG, "--- Runtime State ---");
    ESP_LOGI(TAG, "  Clock source ID: %d", dev->clock_source_id);
    ESP_LOGI(TAG, "  Feature unit: %s (ID=%d, mute=%s, volume=%s)",
             iface->has_feature_unit ? "yes" : "no", iface->feature_unit_id,
             iface->has_mute ? "yes" : "no", iface->has_volume ? "yes" : "no");
    if (iface->volume_range_valid) {
        ESP_LOGI(TAG, "  Volume range: %.2f to %.2f dB (res %.4f dB)",
                 iface->volume_min_db256 / 256.0, iface->volume_max_db256 / 256.0,
                 iface->volume_res_db256 / 256.0);
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
    release_iface_io_ref(iface);
}

// ── Public API: Suspend / Resume ──────────────────────────────────

static esp_err_t stream_deactivate(uac2_iface_t *iface)
{
    uac2_device_t *dev = iface->parent;

    portENTER_CRITICAL(&iface->state_lock);
    if (iface->state == UAC2_IFACE_STATE_ACTIVE) {
        iface->state = UAC2_IFACE_STATE_SUSPENDING;
    }
    portEXIT_CRITICAL(&iface->state_lock);

    if (!atomic_load(&dev->gone)) {
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
    }

    if (iface->interface_claimed) {
        esp_err_t rel_err = release_interface_claim(dev, iface, pdMS_TO_TICKS(2000));
        if (rel_err != ESP_OK) {
            return rel_err;
        }
        iface->interface_claimed = false;
    }

    return wait_for_urbs_quiesced(iface, 2000, "Deactivate", false);
}

esp_err_t uac2_host_device_suspend(uac2_host_device_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uac2_iface_t *iface = NULL;
    esp_err_t ret = acquire_locked_iface(handle, &iface);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "Invalid handle");

    portENTER_CRITICAL(&iface->state_lock);
    uac2_iface_state_t state = iface->state;
    if (state == UAC2_IFACE_STATE_ACTIVE) {
        iface->state = UAC2_IFACE_STATE_SUSPENDING;
    }
    portEXIT_CRITICAL(&iface->state_lock);

    if (state == UAC2_IFACE_STATE_READY) {
        release_locked_iface(iface);
        return ESP_OK;
    }
    if (state != UAC2_IFACE_STATE_ACTIVE) {
        release_locked_iface(iface);
        return ESP_ERR_INVALID_STATE;
    }

    ret = stream_deactivate(iface);
    if (ret != ESP_OK) {
        portENTER_CRITICAL(&iface->state_lock);
        iface->state = UAC2_IFACE_STATE_ERROR;
        portEXIT_CRITICAL(&iface->state_lock);
        release_locked_iface(iface);
        return ret;
    }

    stream_flush_ringbuf(iface);
    stream_reset_runtime_state(iface);

    portENTER_CRITICAL(&iface->state_lock);
    iface->state = UAC2_IFACE_STATE_READY;
    portEXIT_CRITICAL(&iface->state_lock);

    ESP_LOGI(TAG, "%s stream suspended", iface->dir == UAC2_STREAM_TX ? "TX" : "RX");
    release_locked_iface(iface);
    return ESP_OK;
}

esp_err_t uac2_host_device_resume(uac2_host_device_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uac2_iface_t *iface = NULL;
    esp_err_t ret = acquire_locked_iface(handle, &iface);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "Invalid handle");

    portENTER_CRITICAL(&iface->state_lock);
    uac2_iface_state_t state = iface->state;
    portEXIT_CRITICAL(&iface->state_lock);

    if (state == UAC2_IFACE_STATE_ACTIVE) {
        release_locked_iface(iface);
        return ESP_OK;
    }
    if (state != UAC2_IFACE_STATE_READY) {
        release_locked_iface(iface);
        return ESP_ERR_INVALID_STATE;
    }

    uac2_device_t *dev = iface->parent;
    if (atomic_load(&dev->gone)) {
        release_locked_iface(iface);
        return ESP_ERR_INVALID_STATE;
    }

    if (device_has_opposite_direction_stream(iface)) {
        release_locked_iface(iface);
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (fractional_playback_requires_feedback(iface->dir, iface->fb_ep_addr, iface->sample_rate)) {
        release_locked_iface(iface);
        return ESP_ERR_NOT_SUPPORTED;
    }

    if (!iface->interface_claimed) {
        ret = usb_host_interface_claim(s_uac2_driver->client_handle, dev->dev_hdl,
                                       iface->iface_num, iface->alt_setting);
        if (ret != ESP_OK) {
            release_locked_iface(iface);
            return ret;
        }
        iface->interface_claimed = true;
    }

    ret = ctrl_request_no_data(dev,
        USB_BM_REQUEST_TYPE_DIR_OUT | USB_BM_REQUEST_TYPE_TYPE_STANDARD | USB_BM_REQUEST_TYPE_RECIP_INTERFACE,
        USB_B_REQUEST_SET_INTERFACE, iface->alt_setting, iface->iface_num);
    if (ret != ESP_OK) {
        ret = resume_rollback_to_ready(iface, ret, "SET_INTERFACE");
        release_locked_iface(iface);
        return ret;
    }
    ret = ensure_iface_device_available(iface, UAC2_IFACE_STATE_READY,
                                        "device_resume after SET_INTERFACE");
    if (ret != ESP_OK) {
        ret = resume_rollback_to_ready(iface, ret, "disconnect after SET_INTERFACE");
        release_locked_iface(iface);
        return ret;
    }

    if (dev->clock_source_id != 0 && iface->sample_rate > 0) {
        ret = ensure_sample_rate_applied(dev, iface->sample_rate);
        if (ret != ESP_OK) {
            ret = resume_rollback_to_ready(iface, ret, "sample-rate setup");
            release_locked_iface(iface);
            return ret;
        }
        ret = ensure_iface_device_available(iface, UAC2_IFACE_STATE_READY,
                                            "device_resume after sample-rate setup");
        if (ret != ESP_OK) {
            ret = resume_rollback_to_ready(iface, ret, "disconnect after sample-rate setup");
            release_locked_iface(iface);
            return ret;
        }
    }

    ret = stream_submit_urbs(iface);
    if (ret != ESP_OK) {
        ret = resume_rollback_to_ready(iface, ret, "URB submit");
    } else {
        ret = ensure_iface_device_available(iface, UAC2_IFACE_STATE_ACTIVE,
                                            "device_resume after URB submit");
        if (ret != ESP_OK) {
            ret = resume_rollback_to_ready(iface, ret, "disconnect after URB submit");
        } else {
            ESP_LOGI(TAG, "%s stream resumed", iface->dir == UAC2_STREAM_TX ? "TX" : "RX");
        }
    }

    release_locked_iface(iface);
    return ret;
}
