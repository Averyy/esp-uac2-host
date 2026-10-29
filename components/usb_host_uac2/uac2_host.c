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
#include "esp_idf_version.h"
#include "usb/usb_host.h"
#include "usb/uac2_host.h"
#include "uac2_tx_sizing.h"

static const char *TAG = "uac2-host";

// ── UAC2 control request constants ─────────────────────────────────

#define UAC2_FU_MUTE_CONTROL            0x01
#define UAC2_FU_VOLUME_CONTROL          0x02

#define UAC2_CTRL_SET   (USB_BM_REQUEST_TYPE_DIR_OUT | USB_BM_REQUEST_TYPE_TYPE_CLASS | USB_BM_REQUEST_TYPE_RECIP_INTERFACE)
#define UAC2_CTRL_GET   (USB_BM_REQUEST_TYPE_DIR_IN  | USB_BM_REQUEST_TYPE_TYPE_CLASS | USB_BM_REQUEST_TYPE_RECIP_INTERFACE)

_Static_assert(UAC2_CTRL_XFER_MAX_SIZE >= 2 + UAC2_MAX_SAMPLE_RATE_RANGES * 12,
               "UAC2_CTRL_XFER_MAX_SIZE too small for a sample-rate GET_RANGE request");

// The USB Host HCD spreads an isochronous URB of N packets over
// N * 2^(bInterval-1) frames and asserts that this fits its 64-entry ISOC
// descriptor list minus a 3-frame scheduling margin (hcd_dwc.c
// XFER_LIST_LEN_ISOC / XFER_LIST_ISOC_MARGIN). Endpoints that exceed it must
// be rejected before submission or the host aborts.
#define UAC2_HCD_ISOC_MAX_FRAMES        (64 - 3)

// Endpoint bmAttributes bits 3..2: synchronization type (uac2_ep_sync_type_t)
#define UAC2_EP_SYNC_TYPE(attributes)   ((uac2_ep_sync_type_t)(((attributes) >> 2) & 0x03))

// Minimum interval between repeated warnings logged from transfer callbacks.
// Unthrottled logging at 1 ms packet rate would stall the USB event task.
#define UAC2_CB_LOG_INTERVAL_US         1000000

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
    SemaphoreHandle_t ctrl_xfer_done;   // given once per completed submission
    SemaphoreHandle_t ctrl_mutex;
    // Set when a control request timed out. ESP-IDF does not let clients
    // halt/flush EP0, so the URB stays in flight until the device answers or
    // disconnects; its late completion must be consumed before reuse/free.
    // Protected by ctrl_mutex.
    bool ctrl_xfer_abandoned;
    // Allocated with the control URB so orphaning it at destroy cannot fail.
    struct uac2_orphaned_ctrl *orphan_node;

    // Serializes stream activation (device_start/device_resume) and sample-rate
    // changes across every interface of this device: they share one clock, and
    // the ESP32-S3 duplex guard must see a sibling's activation complete.
    SemaphoreHandle_t stream_mutex;
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
    uint8_t  ep_sync_type;          // uac2_ep_sync_type_t
    bool interface_claimed;

    // Feedback endpoint (async playback only)
    uint8_t  fb_ep_addr;
    usb_transfer_t *fb_xfer;
    atomic_uint fb_value;           // 16.16 feedback
    uint32_t nominal_fb;            // 16.16 samples/frame implied by sample_rate
    uint32_t fb_accumulator;        // fractional samples carried between packets: 16.16 with a
                                    // feedback EP, sample_freq % 1000 units without (see
                                    // tx_next_packet_samples); callback context only

    // Isochronous URBs
    usb_transfer_t *xfer[UAC2_NUM_ISOC_URBS];
    int xfer_count;
    atomic_int urbs_in_flight;
    atomic_int consecutive_errors;      // data URBs
    atomic_int fb_consecutive_errors;   // feedback URB; kept apart so a live
                                        // feedback EP cannot mask a dead data EP

    // Packet problems aggregated for throttled logging (callback context only)
    uint32_t log_skipped_pkts;
    uint32_t log_error_pkts;
    uint32_t log_dropped_bytes;
    uint32_t log_failed_urbs;
    int log_last_urb_status;
    int64_t last_issue_log_us;

    // Timing
    _Atomic int64_t first_frame_us;

    // Ring buffer
    RingbufHandle_t ringbuf;
    uint32_t ringbuf_size;
    uint32_t ringbuf_threshold;
    _Atomic bool tx_done_pending;

    // Tasks currently inside device_write/device_read. The ring buffer is only
    // flushed or deleted after the stream has left ACTIVE and this drops to 0.
    atomic_uint rw_users;
} uac2_iface_t;

#define UAC2_SCAN_MAX_DEVICES           8
// Window in which a NEW_DEV event for a device already reported by the
// install-time scan is treated as a duplicate. Re-enumerating a different
// device at the same address takes longer than this (debounce + reset + enum).
#define UAC2_SCAN_DEDUP_WINDOW_MS       500

/**
 * Control URB abandoned by device_destroy while still in flight. The USB
 * device stays open with it: ESP-IDF asserts if the last handle to a device is
 * closed while a control transfer is in flight.
 */
typedef struct uac2_orphaned_ctrl {
    usb_transfer_t *xfer;
    SemaphoreHandle_t done;
    usb_device_handle_t dev_hdl;
    uint8_t addr;
    STAILQ_ENTRY(uac2_orphaned_ctrl) tailq_entry;
} uac2_orphaned_ctrl_t;

/**
 * Singleton driver state — created by uac2_host_install().
 */
typedef struct {
    STAILQ_HEAD(devices, uac2_device) devices_tailq;
    STAILQ_HEAD(interfaces, uac2_interface) ifaces_tailq;
    volatile bool end_client_event_handling;
    // A uac2_host_handle_events() call is running (s_uac2_lock). Uninstall
    // waits only for a running call, so a manual pump that has stopped, or
    // that calls uninstall itself between calls, cannot hang it.
    bool handling_events;
    usb_host_client_handle_t client_handle;
    uac2_host_driver_event_cb_t user_cb;
    void *user_arg;
    SemaphoreHandle_t all_events_handled;
    // Given by the background task as it exits; NULL without one.
    SemaphoreHandle_t event_task_done;

    // Control URBs that outlived their device (protected by s_uac2_lock).
    // Reclaimed on the event task once their completion has been delivered.
    STAILQ_HEAD(orphans, uac2_orphaned_ctrl) orphans_tailq;

    // Install-time scan of already-attached devices (event context only)
    bool initial_scan_done;
    uint8_t scan_dedup_addrs[UAC2_SCAN_MAX_DEVICES];
    int scan_dedup_count;
    TickType_t scan_dedup_start;
} uac2_driver_t;

static uac2_driver_t *s_uac2_driver;

// Serializes install/uninstall/open/close. Static and never deleted, so a call
// racing uninstall cannot block on a freed mutex.
static StaticSemaphore_t s_lifecycle_mutex_buf;
static SemaphoreHandle_t s_lifecycle_mutex;
static atomic_int s_lifecycle_mutex_state;  // 0 = none, 1 = creating, 2 = ready

// ── Forward declarations ──────────────────────────────────────────

static esp_err_t stream_stop_internal(uac2_iface_t *iface);
static void stream_tx_xfer_submit(uac2_iface_t *iface, usb_transfer_t *xfer);
static void stream_flush_ringbuf(uac2_iface_t *iface);
static esp_err_t release_interface_claim(uac2_device_t *dev, uac2_iface_t *iface, TickType_t timeout_ticks);
static esp_err_t stream_deactivate(uac2_iface_t *iface);
static esp_err_t read_current_sample_rate(uac2_device_t *dev, uint32_t *sample_rate);
static esp_err_t validate_sample_rate(uac2_device_t *dev, uint32_t sample_rate);
static void warn_if_rate_not_applied(uac2_device_t *dev, uint32_t sample_rate);
static void device_destroy(uac2_device_t *dev);
static bool consume_scan_duplicate(uint8_t addr);
static void scan_forget_addr(uint8_t addr);
static esp_err_t wait_for_urbs_quiesced(uac2_iface_t *iface, uint32_t timeout_ms,
                                        const char *stage, bool set_error_state);

static inline bool stream_has_resources(const uac2_iface_t *iface)
{
    return iface->xfer_count > 0 ||
           iface->fb_xfer != NULL ||
           iface->ringbuf != NULL;
}

static SemaphoreHandle_t lifecycle_mutex(void)
{
    int expected = 0;
    if (atomic_compare_exchange_strong(&s_lifecycle_mutex_state, &expected, 1)) {
        s_lifecycle_mutex = xSemaphoreCreateMutexStatic(&s_lifecycle_mutex_buf);
        atomic_store(&s_lifecycle_mutex_state, 2);
    } else {
        while (atomic_load(&s_lifecycle_mutex_state) != 2) {
            vTaskDelay(1);
        }
    }
    return s_lifecycle_mutex;
}

/** Takes the lifecycle mutex; fails (without holding it) if not installed. */
static esp_err_t driver_lifecycle_lock(void)
{
    xSemaphoreTake(lifecycle_mutex(), portMAX_DELAY);
    if (!s_uac2_driver) {
        xSemaphoreGive(s_lifecycle_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

static void driver_lifecycle_unlock(void)
{
    xSemaphoreGive(s_lifecycle_mutex);
}

static esp_err_t acquire_iface_runtime_ref(uac2_host_device_handle_t handle, uac2_iface_t **out_iface)
{
    if (!out_iface) return ESP_ERR_INVALID_ARG;
    *out_iface = NULL;
    if (!handle) return ESP_ERR_INVALID_ARG;

    uac2_iface_t *target = (uac2_iface_t *)handle;
    bool found = false;

    UAC2_ENTER_CRITICAL();
    // Checked under the lock uninstall clears it with: these calls accept
    // closed handles, so one can race the final close and uninstall.
    if (!s_uac2_driver) {
        UAC2_EXIT_CRITICAL();
        return ESP_ERR_INVALID_STATE;
    }
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

/**
 * acquire_iface_io_ref() for the streaming calls, which document
 * ESP_ERR_INVALID_STATE (not INVALID_ARG) for a closing or already-closed
 * handle: a writer racing close must see the same error as a stopped stream.
 */
static uac2_iface_t *acquire_stream_io_ref(uac2_host_device_handle_t handle)
{
    uac2_iface_t *iface = acquire_iface_io_ref(handle);
    if (!iface) {
        ESP_LOGD(TAG, "Stream I/O on a closing or closed handle");
    }
    return iface;
}

static void release_iface_io_ref(uac2_iface_t *iface)
{
    if (iface) {
        atomic_fetch_sub(&iface->io_users, 1);
    }
}

static inline bool stream_is_active(uac2_iface_t *iface)
{
    portENTER_CRITICAL(&iface->state_lock);
    bool active = iface->state == UAC2_IFACE_STATE_ACTIVE;
    portEXIT_CRITICAL(&iface->state_lock);
    return active;
}

/** Streaming and the device still attached (telemetry is meaningful). */
static inline bool stream_is_live(uac2_iface_t *iface)
{
    return stream_is_active(iface) && !atomic_load(&iface->parent->gone);
}

/**
 * Register the caller as a ring buffer user (device_write/device_read).
 * Registering before the state check pairs with stream teardown, which leaves
 * ACTIVE before waiting for rw_users to drain: either the caller sees a
 * non-ACTIVE stream here, or teardown waits for the caller to finish.
 * On success the caller must decrement rw_users when done.
 */
static bool acquire_ringbuf_user(uac2_iface_t *iface)
{
    atomic_fetch_add(&iface->rw_users, 1);
    if (stream_is_active(iface) && iface->ringbuf) {
        return true;
    }
    atomic_fetch_sub(&iface->rw_users, 1);
    return false;
}

/**
 * Sleep one polling step of about 5 ms, but at least one tick: at
 * CONFIG_FREERTOS_HZ=100 pdMS_TO_TICKS(5) is 0, and vTaskDelay(0) would not
 * let a lower-priority task (the USB event task, a reader or writer) run.
 */
static inline void poll_delay(void)
{
    TickType_t ticks = pdMS_TO_TICKS(5);
    vTaskDelay(ticks > 0 ? ticks : 1);
}

/**
 * Unlink a closing interface once no runtime references remain. References are
 * only taken under s_uac2_lock while the interface is linked, so checking the
 * count and unlinking under that lock leaves no window for a new reference.
 */
static esp_err_t unlink_iface_when_unreferenced(uac2_iface_t *iface, TickType_t timeout_ticks,
                                                uint8_t *remaining_refs)
{
    TickType_t start = xTaskGetTickCount();
    while (true) {
        UAC2_ENTER_CRITICAL();
        if (atomic_load(&iface->io_users) == 0) {
            STAILQ_REMOVE(&s_uac2_driver->ifaces_tailq, iface, uac2_interface, tailq_entry);
            iface->parent->opened_cnt--;
            *remaining_refs = iface->parent->opened_cnt;
            UAC2_EXIT_CRITICAL();
            return ESP_OK;
        }
        UAC2_EXIT_CRITICAL();
        if ((xTaskGetTickCount() - start) >= timeout_ticks) {
            ESP_LOGE(TAG, "I/O users still active during close: %u",
                     (unsigned)atomic_load(&iface->io_users));
            return ESP_ERR_TIMEOUT;
        }
        poll_delay();
    }
}

/**
 * usb_host_interface_release() refuses with ESP_ERR_INVALID_STATE while the
 * client event task still has endpoint completions to process (ESP-IDF
 * #17707). The upstream fix (IDF 5.4.4 / 5.5.4, espressif/usb 1.2.0) instead
 * sleeps 10 ticks (100 ms at 100 Hz) if any of the client's endpoints has
 * unprocessed completions, and the in-flight count is still decremented after
 * callbacks run. Callers wait for this interface's URBs to quiesce first, so
 * neither normally happens; keep retrying on every version for the rest.
 */
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
    TickType_t start = xTaskGetTickCount();
    while (atomic_load(&iface->urbs_in_flight) > 0 &&
           (xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout_ms)) {
        poll_delay();
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
    // Context is the completion semaphore rather than the device, so a URB
    // orphaned by device_destroy() never touches freed device memory.
    xSemaphoreGive((SemaphoreHandle_t)xfer->context);
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

    if (dev->ctrl_xfer_abandoned) {
        // The URB from a timed-out request is still owned by the host stack.
        // It can be reused only after its late completion has been delivered.
        if (xSemaphoreTake(dev->ctrl_xfer_done, 0) != pdTRUE) {
            ESP_LOGE(TAG, "Previous control transfer still pending on addr %d", dev->addr);
            xSemaphoreGive(dev->ctrl_mutex);
            return ESP_ERR_INVALID_STATE;
        }
        dev->ctrl_xfer_abandoned = false;
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
    xfer->context = dev->ctrl_xfer_done;
    xfer->timeout_ms = UAC2_CTRL_XFER_TIMEOUT_MS;

    esp_err_t err = usb_host_transfer_submit_control(s_uac2_driver->client_handle, xfer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Control submit failed: %s", esp_err_to_name(err));
        xSemaphoreGive(dev->ctrl_mutex);
        return err;
    }

    if (xSemaphoreTake(dev->ctrl_xfer_done, pdMS_TO_TICKS(UAC2_CTRL_XFER_TIMEOUT_MS)) != pdTRUE) {
        // usb_host_endpoint_halt/flush reject EP0 (it is owned by USBH), so a
        // timed-out control transfer cannot be cancelled. Leave it in flight
        // and make the next request / device_destroy() wait for its completion.
        ESP_LOGE(TAG, "Control transfer timeout");
        dev->ctrl_xfer_abandoned = true;
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

/**
 * Tally per-packet results of a COMPLETED isochronous URB. The HCD reports
 * the URB as COMPLETED even when individual packets were SKIPPED (frame slot
 * missed) or failed, so the overall status alone says nothing about delivery.
 * Returns the number of hard (non-skipped) packet errors.
 */
static int stream_tally_isoc_packets(uac2_iface_t *iface, const usb_transfer_t *xfer)
{
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
    iface->log_skipped_pkts += skipped;
    iface->log_error_pkts += errors;
    return errors;
}

/** Log aggregated packet problems at most once per UAC2_CB_LOG_INTERVAL_US. */
static void stream_log_issues_throttled(uac2_iface_t *iface)
{
    if (iface->log_skipped_pkts == 0 && iface->log_error_pkts == 0 &&
        iface->log_dropped_bytes == 0 && iface->log_failed_urbs == 0) {
        return;
    }
    int64_t now = esp_timer_get_time();
    if (now - iface->last_issue_log_us < UAC2_CB_LOG_INTERVAL_US) {
        return;
    }
    iface->last_issue_log_us = now;
    ESP_LOGW(TAG, "%s iface %d: %" PRIu32 " packets skipped, %" PRIu32 " packet errors, "
             "%" PRIu32 " bytes dropped (ringbuf full), %" PRIu32 " failed URBs (last status %d, "
             "consecutive %d)",
             iface->dir == UAC2_STREAM_TX ? "TX" : "RX", iface->iface_num,
             iface->log_skipped_pkts, iface->log_error_pkts, iface->log_dropped_bytes,
             iface->log_failed_urbs, iface->log_last_urb_status,
             atomic_load(&iface->consecutive_errors));
    iface->log_skipped_pkts = 0;
    iface->log_error_pkts = 0;
    iface->log_dropped_bytes = 0;
    iface->log_failed_urbs = 0;
}

/**
 * Count a failed URB. Returns true if the caller should resubmit it, false if
 * the stream hit UAC2_MAX_CONSECUTIVE_ERRORS (URB retired, stream in ERROR).
 */
static bool stream_handle_xfer_error(uac2_iface_t *iface, const usb_transfer_t *xfer)
{
    const char *dir = iface->dir == UAC2_STREAM_TX ? "TX" : "RX";
    int errs = atomic_fetch_add(&iface->consecutive_errors, 1) + 1;
    // A device alternating good and failed URBs never trips the limit, so
    // per-URB warnings are aggregated like the packet counters.
    iface->log_failed_urbs++;
    iface->log_last_urb_status = xfer->status;
    stream_log_issues_throttled(iface);
    if (errs >= UAC2_MAX_CONSECUTIVE_ERRORS) {
        ESP_LOGE(TAG, "%s: %d consecutive errors, stream dead", dir, errs);
        atomic_fetch_sub(&iface->urbs_in_flight, 1);
        // Only a still-running stream dies: a stop that already moved it to
        // IDLE must not be left in ERROR
        portENTER_CRITICAL(&iface->state_lock);
        bool was_active = iface->state == UAC2_IFACE_STATE_ACTIVE;
        if (was_active) iface->state = UAC2_IFACE_STATE_ERROR;
        portEXIT_CRITICAL(&iface->state_lock);
        if (was_active && iface->user_cb) {
            iface->user_cb(iface, UAC2_HOST_DEVICE_EVENT_STREAM_ERROR, iface->user_cb_arg);
        }
        return false;
    }
    if (iface->user_cb) {
        iface->user_cb(iface, UAC2_HOST_DEVICE_EVENT_TRANSFER_ERROR, iface->user_cb_arg);
    }
    return true;
}

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
        int errors = stream_tally_isoc_packets(iface, xfer);
        stream_log_issues_throttled(iface);
        if (errors < xfer->num_isoc_packets) {
            atomic_store(&iface->consecutive_errors, 0);
            stream_tx_xfer_submit(iface, xfer);
        } else if (stream_handle_xfer_error(iface, xfer)) {
            // Every packet hard-failed: count it so a dead link reaches STREAM_ERROR
            stream_tx_xfer_submit(iface, xfer);
        }
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

    default:
        if (stream_handle_xfer_error(iface, xfer)) {
            stream_tx_xfer_submit(iface, xfer);
        }
        break;
    }
}

/**
 * Sample frames for the next playback packet. With a feedback endpoint, follow
 * the device's reported rate (nominal until the first report). Without one the
 * endpoint is adaptive or synchronous (start refuses fractional rates on async
 * endpoints without feedback), so send exactly sample_freq / 1000 per frame.
 */
static uint32_t tx_next_packet_samples(uac2_iface_t *iface, uint32_t max_samples)
{
    if (iface->fb_ep_addr == 0) {
        return uac2_tx_next_nominal_samples(iface->sample_rate, &iface->fb_accumulator,
                                            max_samples);
    }
    uint32_t rate_q16 = uac2_tx_rate_q16(iface->nominal_fb, atomic_load(&iface->fb_value));
    return uac2_tx_next_packet_samples(rate_q16, &iface->fb_accumulator, max_samples);
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

    uint32_t frame_bytes = (uint32_t)iface->channels * iface->sub_slot_size;
    uint32_t max_samples = frame_bytes ? iface->ep_mps / frame_bytes : 0;
    uint32_t bytes_filled = 0;

    for (int i = 0; i < num_pkts; i++) {
        uint16_t pkt_size;
        if (iface->sample_rate != 0 && max_samples != 0) {
            pkt_size = (uint16_t)(tx_next_packet_samples(iface, max_samples) * frame_bytes);
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
    case USB_TRANSFER_STATUS_COMPLETED: {
        int errors = stream_tally_isoc_packets(iface, xfer);
        if (errors >= xfer->num_isoc_packets) {
            // Every packet hard-failed: count it so a dead link reaches STREAM_ERROR
            stream_log_issues_throttled(iface);
            if (stream_handle_xfer_error(iface, xfer)) {
                esp_err_t sub_err = usb_host_transfer_submit(xfer);
                if (sub_err != ESP_OK) {
                    ESP_LOGW(TAG, "RX resubmit failed: %s", esp_err_to_name(sub_err));
                    atomic_fetch_sub(&iface->urbs_in_flight, 1);
                }
            }
            break;
        }
        atomic_store(&iface->consecutive_errors, 0);
        for (int i = 0; i < xfer->num_isoc_packets; i++) {
            usb_isoc_packet_desc_t *pkt = &xfer->isoc_packet_desc[i];
            if (pkt->status == USB_TRANSFER_STATUS_COMPLETED && pkt->actual_num_bytes > 0) {
                uint8_t *pkt_data = xfer->data_buffer + (i * iface->ep_mps);
                BaseType_t ok = xRingbufferSend(iface->ringbuf, pkt_data,
                                                pkt->actual_num_bytes, 0);
                if (ok != pdTRUE) {
                    iface->log_dropped_bytes += pkt->actual_num_bytes;
                }
            }
        }
        stream_log_issues_throttled(iface);
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
    }

    case USB_TRANSFER_STATUS_NO_DEVICE:
    case USB_TRANSFER_STATUS_CANCELED:
        atomic_fetch_sub(&iface->urbs_in_flight, 1);
        if (xfer->status == USB_TRANSFER_STATUS_NO_DEVICE &&
            iface->user_cb && !atomic_exchange(&iface->disconnect_fired, true)) {
            iface->user_cb(iface, UAC2_HOST_DEVICE_EVENT_DISCONNECTED, iface->user_cb_arg);
        }
        return;

    default:
        if (stream_handle_xfer_error(iface, xfer)) {
            esp_err_t sub_err = usb_host_transfer_submit(xfer);
            if (sub_err != ESP_OK) {
                ESP_LOGW(TAG, "RX resubmit failed: %s", esp_err_to_name(sub_err));
                atomic_fetch_sub(&iface->urbs_in_flight, 1);
            }
        }
        break;
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

    if (xfer->status == USB_TRANSFER_STATUS_NO_DEVICE ||
        xfer->status == USB_TRANSFER_STATUS_CANCELED) {
        atomic_fetch_sub(&iface->urbs_in_flight, 1);
        if (xfer->status == USB_TRANSFER_STATUS_NO_DEVICE &&
            iface->user_cb && !atomic_exchange(&iface->disconnect_fired, true)) {
            iface->user_cb(iface, UAC2_HOST_DEVICE_EVENT_DISCONNECTED, iface->user_cb_arg);
        }
        return;
    }

    // HCD reports the URB COMPLETED even when its only packet failed; that
    // packet's bytes are not a feedback value. A SKIPPED packet (frame slot
    // missed under load) is not a device error either, matching the data
    // path: keep the last value and resubmit without counting it.
    usb_transfer_status_t pkt_status = xfer->isoc_packet_desc[0].status;
    bool fb_ok = xfer->status == USB_TRANSFER_STATUS_COMPLETED &&
                 pkt_status == USB_TRANSFER_STATUS_COMPLETED;
    bool fb_skipped = xfer->status == USB_TRANSFER_STATUS_COMPLETED &&
                      pkt_status == USB_TRANSFER_STATUS_SKIPPED;
    if (!fb_ok && !fb_skipped) {
        int errs = atomic_fetch_add(&iface->fb_consecutive_errors, 1) + 1;
        ESP_LOGD(TAG, "Feedback transfer error, status=%d/%d (consecutive: %d)",
                 xfer->status, xfer->isoc_packet_desc[0].status, errs);
        if (errs >= UAC2_MAX_CONSECUTIVE_ERRORS) {
            ESP_LOGE(TAG, "Feedback: %d consecutive errors, stream dead", errs);
            atomic_fetch_sub(&iface->urbs_in_flight, 1);
            portENTER_CRITICAL(&iface->state_lock);
            bool was_active = iface->state == UAC2_IFACE_STATE_ACTIVE;
            if (was_active) iface->state = UAC2_IFACE_STATE_ERROR;
            portEXIT_CRITICAL(&iface->state_lock);
            if (was_active && iface->user_cb) {
                iface->user_cb(iface, UAC2_HOST_DEVICE_EVENT_STREAM_ERROR, iface->user_cb_arg);
            }
            return;
        }
    }

    if (fb_ok) {
        atomic_store(&iface->fb_consecutive_errors, 0);
        int actual = xfer->isoc_packet_desc[0].actual_num_bytes;
        if (actual > 4) actual = 4;  // clamp to buffer size
        uint32_t fb_q16;
        if (uac2_feedback_decode_q16(xfer->data_buffer, actual, &fb_q16)) {
            atomic_store(&iface->fb_value, fb_q16);
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
        // The requested 4 bytes, not data_buffer_size: ESP-IDF before 5.4.2
        // can report more than was allocated when heap poisoning is enabled
        xfer->isoc_packet_desc[0].num_bytes = 4;
        xfer->num_bytes = 4;
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

/**
 * Alt settings of iface_num that match dir and the requested format, best
 * first: highest bit depth first (only one depth when bit_resolution is set),
 * descriptor order within a depth. Returns how many were written to out,
 * which must hold UAC2_MAX_AS_INTERFACES entries.
 */
static int find_matching_as_ifaces(const uac2_device_info_t *info,
                                   uac2_stream_dir_t dir,
                                   const uac2_host_stream_config_t *config,
                                   uint8_t iface_num,
                                   const uac2_as_iface_t **out)
{
    int n = 0;

    for (int i = 0; i < info->num_as_ifaces && i < UAC2_MAX_AS_INTERFACES; i++) {
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

        int pos = n++;
        while (pos > 0 && out[pos - 1]->bit_resolution < as->bit_resolution) {
            out[pos] = out[pos - 1];
            pos--;
        }
        out[pos] = as;
    }

    return n;
}

/**
 * With the USB Host Library's FIFO presets the ESP32-S3 cannot carry TX and RX
 * audio at once. CONFIG_UAC2_ALLOW_DUPLEX lifts this when the application
 * configured a FIFO split with room for both; the HCD then refuses any
 * endpoint that does not fit when the interface is claimed. Caller must hold
 * dev->stream_mutex, so a sibling's start cannot be half-way through.
 */
static bool device_has_opposite_direction_stream(const uac2_iface_t *iface)
{
#if CONFIG_IDF_TARGET_ESP32S3 && !CONFIG_UAC2_ALLOW_DUPLEX
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

/**
 * True if any interface on the device has a started (or suspended) stream.
 * Caller must hold dev->stream_mutex.
 */
static bool device_has_stream(const uac2_device_t *dev)
{
    bool busy = false;
    UAC2_ENTER_CRITICAL();
    uac2_iface_t *iface = NULL;
    STAILQ_FOREACH(iface, &s_uac2_driver->ifaces_tailq, tailq_entry) {
        if (iface->parent == dev &&
            (iface->state != UAC2_IFACE_STATE_IDLE || stream_has_resources(iface))) {
            busy = true;
            break;
        }
    }
    UAC2_EXIT_CRITICAL();
    return busy;
}

/**
 * Whether another started or suspended stream on iface's device runs at a
 * rate other than sample_rate. The device has one clock, so applying
 * sample_rate would change it under that stream. Caller must hold the
 * device's stream_mutex (sample_rate is only written under it).
 */
static bool device_has_stream_at_other_rate(const uac2_iface_t *iface, uint32_t sample_rate)
{
    bool conflict = false;
    UAC2_ENTER_CRITICAL();
    uac2_iface_t *other = NULL;
    STAILQ_FOREACH(other, &s_uac2_driver->ifaces_tailq, tailq_entry) {
        if (other == iface || other->parent != iface->parent) {
            continue;
        }
        if ((other->state != UAC2_IFACE_STATE_IDLE || stream_has_resources(other)) &&
            other->sample_rate != sample_rate) {
            conflict = true;
            break;
        }
    }
    UAC2_EXIT_CRITICAL();
    return conflict;
}

/**
 * The HCD schedules an isochronous packet every 2^(bInterval-1) frames.
 * Audio data endpoints must be 1 at full speed (one packet per 1 ms frame,
 * which the packet sizing assumes); the URB must also fit the HCD ISOC list.
 */
static bool iso_ep_interval_supported(uint8_t b_interval, int num_packets, bool is_data_ep)
{
    if (b_interval < 1 || b_interval > 16) {
        return false;
    }
    if (is_data_ep && b_interval != 1) {
        return false;
    }
    uint32_t frames = (1u << (b_interval - 1)) * (uint32_t)num_packets;
    return frames <= UAC2_HCD_ISOC_MAX_FRAMES;
}

/**
 * An asynchronous sink runs its own clock, so a fractional rate needs its
 * feedback to stay in step. Adaptive and synchronous sinks follow the host's
 * frame timing and play the nominal 44/45-frame pattern without feedback.
 * Endpoints that declare no synchronization type are treated as asynchronous.
 */
static bool fractional_playback_requires_feedback(uac2_stream_dir_t dir,
                                                  uint8_t fb_ep_addr,
                                                  uac2_ep_sync_type_t sync_type,
                                                  uint32_t sample_rate)
{
    return dir == UAC2_STREAM_TX &&
           fb_ep_addr == 0 &&
           sync_type != UAC2_EP_SYNC_ADAPTIVE &&
           sync_type != UAC2_EP_SYNC_SYNC &&
           sample_rate > 0 &&
           (sample_rate % 1000) != 0;
}

static uint16_t calc_packet_size(uint32_t sample_rate, uint8_t channels,
                                 uint8_t sub_slot_size)
{
    // 64-bit: an absurd sample_freq with a many-channel descriptor would
    // otherwise wrap to a small size that passes the MPS check
    uint64_t samples_per_frame = ((uint64_t)sample_rate + 999) / 1000;
    uint64_t result = samples_per_frame * channels * sub_slot_size;
    if (result > UINT16_MAX) {
        ESP_LOGE(TAG, "Packet size overflow: %" PRIu64, result);
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
    iface->ep_sync_type = UAC2_EP_SYNC_TYPE(as->ep_attributes);
    iface->fb_ep_addr = as->fb_ep_addr;
    // Use atomic_store (not atomic_init) because the iface struct persists
    // across start/stop cycles. atomic_init on an already-initialized atomic is UB.
    atomic_store(&iface->fb_value, 0);
    iface->nominal_fb = uac2_tx_nominal_q16(config->sample_freq);
    iface->fb_accumulator = 0;
    atomic_store(&iface->urbs_in_flight, 0);
    atomic_store(&iface->consecutive_errors, 0);
    atomic_store(&iface->fb_consecutive_errors, 0);
    atomic_store(&iface->first_frame_us, 0);
    atomic_store(&iface->tx_done_pending, false);
    iface->log_skipped_pkts = 0;
    iface->log_error_pkts = 0;
    iface->log_dropped_bytes = 0;
    iface->log_failed_urbs = 0;
    iface->last_issue_log_us = 0;

    iface->packet_size = calc_packet_size(config->sample_freq,
                                          as->nr_channels, as->sub_slot_size);
    if (iface->packet_size == 0 || iface->packet_size > iface->ep_mps) {
        ESP_LOGE(TAG, "Packet size %d invalid (MPS %d)", iface->packet_size, iface->ep_mps);
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
            iface->xfer_count = 0;  // or stream_has_resources() keeps the interface "busy"
            vRingbufferDelete(iface->ringbuf);
            iface->ringbuf = NULL;
            return err;
        }
        iface->xfer_count = i + 1;
    }

    // Feedback URB. An async endpoint without feedback would slowly drift
    // against the device clock, so failing to allocate it fails the start.
    if (iface->fb_ep_addr != 0 && iface->dir == UAC2_STREAM_TX) {
        esp_err_t err = usb_host_transfer_alloc(4, 1, &iface->fb_xfer);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to allocate feedback URB: %s", esp_err_to_name(err));
            for (int i = 0; i < iface->xfer_count; i++) {
                usb_host_transfer_free(iface->xfer[i]);
                iface->xfer[i] = NULL;
            }
            iface->xfer_count = 0;
            vRingbufferDelete(iface->ringbuf);
            iface->ringbuf = NULL;
            return err;
        }
    }

    return ESP_OK;
}

/**
 * Wake any task blocked in device_write/device_read and wait for it to leave.
 * The caller must already have moved the stream out of ACTIVE, so callers that
 * arrive later bail out before touching the ring buffer.
 */
static esp_err_t wait_for_rw_users_drained(uac2_iface_t *iface, uint32_t timeout_ms,
                                           const char *stage)
{
    TickType_t start = xTaskGetTickCount();
    while (atomic_load(&iface->rw_users) > 0) {
        if (iface->ringbuf) {
            if (iface->dir == UAC2_STREAM_TX) {
                stream_flush_ringbuf(iface);    // make room for a blocked writer
            } else {
                uint8_t dummy = 0;
                xRingbufferSend(iface->ringbuf, &dummy, 1, 0);  // wake a blocked reader
            }
        }
        if ((xTaskGetTickCount() - start) >= pdMS_TO_TICKS(timeout_ms)) {
            ESP_LOGE(TAG, "%s: %u read/write callers still active after %" PRIu32 "ms — retaining ring buffer",
                     stage, (unsigned)atomic_load(&iface->rw_users), timeout_ms);
            return ESP_ERR_TIMEOUT;
        }
        poll_delay();
    }
    return ESP_OK;
}

static esp_err_t stream_resources_free(uac2_iface_t *iface)
{
    esp_err_t err = wait_for_rw_users_drained(iface, 2000, "Stream free");
    if (err != ESP_OK) {
        return err;
    }

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
        vRingbufferDelete(iface->ringbuf);
        iface->ringbuf = NULL;
    }
    return ESP_OK;
}

static esp_err_t stream_submit_urbs(uac2_iface_t *iface)
{
    uac2_device_t *dev = iface->parent;
    int num_pkts = UAC2_NUM_PACKETS_PER_URB;

    portENTER_CRITICAL(&iface->state_lock);
    iface->state = UAC2_IFACE_STATE_ACTIVE;
    portEXIT_CRITICAL(&iface->state_lock);

    // Prepare every URB before submitting any: a completed TX URB's callback
    // advances fb_accumulator, which the silent packets below also use.
    uint32_t frame_bytes = (uint32_t)iface->channels * iface->sub_slot_size;
    uint32_t max_samples = frame_bytes ? iface->ep_mps / frame_bytes : 0;
    for (int i = 0; i < iface->xfer_count; i++) {
        usb_transfer_t *xfer = iface->xfer[i];
        xfer->device_handle = dev->dev_hdl;
        xfer->bEndpointAddress = iface->ep_addr;
        xfer->context = iface;
        xfer->timeout_ms = 0;

        if (iface->dir == UAC2_STREAM_TX) {
            // Silence, sized like stream_tx_xfer_submit() so a fractional
            // rate (44.1 kHz) does not start with rounded-up packets
            xfer->callback = stream_tx_xfer_done;
            int total = 0;
            for (int j = 0; j < num_pkts; j++) {
                int pkt_size = iface->packet_size;
                if (iface->sample_rate != 0 && max_samples != 0) {
                    pkt_size = (int)(tx_next_packet_samples(iface, max_samples) * frame_bytes);
                }
                xfer->isoc_packet_desc[j].num_bytes = pkt_size;
                total += pkt_size;
            }
            memset(xfer->data_buffer, 0, total);
            xfer->num_bytes = total;
        } else {
            xfer->callback = stream_rx_xfer_done;
            xfer->num_bytes = iface->ep_mps * num_pkts;
            for (int j = 0; j < num_pkts; j++) {
                xfer->isoc_packet_desc[j].num_bytes = iface->ep_mps;
            }
        }
    }

    for (int i = 0; i < iface->xfer_count; i++) {
        usb_transfer_t *xfer = iface->xfer[i];
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
            ESP_LOGE(TAG, "Feedback URB submit failed: %s", esp_err_to_name(err));
            return err;     // caller rolls back the data URBs already submitted
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

    // Submitted URBs complete without resubmitting once the state is IDLE
    esp_err_t wait_err = wait_for_urbs_quiesced(iface, 2000, "Startup abort", true);
    if (wait_err != ESP_OK) {
        return wait_err;
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

    esp_err_t free_err = stream_resources_free(iface);
    if (free_err != ESP_OK) {
        return free_err;
    }
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
    atomic_store(&iface->fb_consecutive_errors, 0);
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
        cleanup_err = wait_for_rw_users_drained(iface, 2000, "Resume rollback");
    }
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

    esp_err_t err = validate_sample_rate(dev, sample_rate);
    if (err != ESP_OK) {
        return err;
    }
    err = set_sample_rate_internal(dev, sample_rate);
    if (err == ESP_OK) {
        warn_if_rate_not_applied(dev, sample_rate);
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

/**
 * Check sample_rate against the clock's advertised RANGE, honouring each
 * range's resolution. A device may acknowledge a rate it cannot run and play
 * silence, so a rate the device does not list is refused (unless
 * CONFIG_UAC2_ALLOW_UNADVERTISED_RATES). If the list cannot be read, or has more
 * ranges than were read, the device gets the benefit of the doubt.
 */
static esp_err_t validate_sample_rate(uac2_device_t *dev, uint32_t sample_rate)
{
    if (dev->clock_source_id == 0) return ESP_OK;

    uint8_t buf[2 + UAC2_MAX_SAMPLE_RATE_RANGES * 12];
    uint16_t actual_len = 0;
    memset(buf, 0, sizeof(buf));
    esp_err_t err = ctrl_get_range(dev, dev->clock_source_id,
                                   UAC2_CS_SAM_FREQ_CONTROL, 0, buf, sizeof(buf), &actual_len);
    if (err != ESP_OK) return ESP_OK;

    uint16_t advertised = buf[0] | (buf[1] << 8);
    uint16_t count = advertised;
    if (count > UAC2_MAX_SAMPLE_RATE_RANGES) count = UAC2_MAX_SAMPLE_RATE_RANGES;
    if (count == 0) return ESP_OK;

    err = validate_range_payload_len("Sample-rate RANGE", actual_len, count,
                                     UAC2_MAX_SAMPLE_RATE_RANGES, 12);
    if (err != ESP_OK) return ESP_OK;

    for (int i = 0; i < count; i++) {
        const uint8_t *p = buf + 2 + (i * 12);
        uint32_t min = (uint32_t)p[0] | ((uint32_t)p[1] << 8)
                     | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
        uint32_t max = (uint32_t)p[4] | ((uint32_t)p[5] << 8)
                     | ((uint32_t)p[6] << 16) | ((uint32_t)p[7] << 24);
        uint32_t res = (uint32_t)p[8] | ((uint32_t)p[9] << 8)
                     | ((uint32_t)p[10] << 16) | ((uint32_t)p[11] << 24);
        if (min == max) {
            if (sample_rate == min) return ESP_OK;
        } else if (sample_rate >= min && sample_rate <= max &&
                   (res == 0 || (sample_rate - min) % res == 0)) {
            return ESP_OK;
        }
    }
    if (advertised > count) {
        // Only the first UAC2_MAX_SAMPLE_RATE_RANGES were read: no proof of exclusion
        ESP_LOGW(TAG, "Sample rate %" PRIu32 " Hz not in the first %u of %u advertised ranges "
                 "(continuing)", sample_rate, (unsigned)count, (unsigned)advertised);
        return ESP_OK;
    }
#if CONFIG_UAC2_ALLOW_UNADVERTISED_RATES
    ESP_LOGW(TAG, "Sample rate %" PRIu32 " Hz not in device's advertised ranges "
             "(continuing: CONFIG_UAC2_ALLOW_UNADVERTISED_RATES)", sample_rate);
    return ESP_OK;
#else
    ESP_LOGE(TAG, "Sample rate %" PRIu32 " Hz is not in the device's advertised ranges",
             sample_rate);
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

/** Whether the resolved clock source's frequency control can be read (bmControls D1..0). */
static bool clock_freq_readable(const uac2_device_t *dev)
{
    const uac2_device_info_t *info = &dev->desc_info;
    for (int i = 0; i < info->num_clock_sources; i++) {
        if (info->clock_sources[i].clock_id == dev->clock_source_id) {
            return (info->clock_sources[i].controls & 0x03) != 0;
        }
    }
    return false;
}

/**
 * Read the clock back after a successful SET. A mismatch is only logged: some
 * devices report a new rate only after a delay, so (like Linux) keep going.
 */
static void warn_if_rate_not_applied(uac2_device_t *dev, uint32_t sample_rate)
{
    if (!clock_freq_readable(dev)) return;
    uint32_t current_rate = 0;
    if (read_current_sample_rate(dev, &current_rate) == ESP_OK && current_rate != sample_rate) {
        ESP_LOGW(TAG, "Device acknowledged %" PRIu32 " Hz but its clock reports %" PRIu32 " Hz",
                 sample_rate, current_rate);
    }
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
    uac2_iface_t *to_notify[UAC2_MAX_AS_INTERFACES];
    int notify_count = 0;
    uint8_t dev_addr = 0;

    // Find the device, mark it gone, and take a reference on each of its
    // interfaces in one critical section. device_destroy() and device_close()
    // unlink under the same lock, and close only unlinks an interface with no
    // references, so nothing collected here can be freed until it is released
    // below. The device itself is not touched after the critical section.
    UAC2_ENTER_CRITICAL();
    uac2_device_t *dev = NULL;
    STAILQ_FOREACH(dev, &s_uac2_driver->devices_tailq, tailq_entry) {
        if (dev->dev_hdl == dev_hdl) {
            break;
        }
    }
    if (dev) {
        dev_addr = dev->addr;
        // stream_stop_internal skips SET_INTERFACE and endpoint ops once gone
        atomic_store(&dev->gone, true);
        uac2_iface_t *iface;
        STAILQ_FOREACH(iface, &s_uac2_driver->ifaces_tailq, tailq_entry) {
            if (iface->parent == dev && notify_count < UAC2_MAX_AS_INTERFACES) {
                atomic_fetch_add(&iface->io_users, 1);
                to_notify[notify_count++] = iface;
            }
        }
    }
    UAC2_EXIT_CRITICAL();
    if (!dev) return ESP_OK;  // Not a UAC2 device we track
    scan_forget_addr(dev_addr);

    for (int i = 0; i < notify_count; i++) {
        uac2_iface_t *iface = to_notify[i];

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
        release_iface_io_ref(iface);
    }

    return ESP_OK;
}

static void client_event_cb(const usb_host_client_event_msg_t *event, void *arg)
{
    if (event->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
        if (consume_scan_duplicate(event->new_dev.address)) {
            ESP_LOGD(TAG, "NEW_DEV addr %d already reported by install-time scan",
                     event->new_dev.address);
            return;
        }
        uac2_host_device_connected(event->new_dev.address);
    } else if (event->event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
        uac2_host_device_disconnected(event->dev_gone.dev_hdl);
    }
}

// ── Event handler task ────────────────────────────────────────────

static esp_err_t handle_events_for(uac2_driver_t *expected, TickType_t timeout);

static void event_handler_task(void *arg)
{
    // Bound to the installation that created it. Uninstall waits for
    // event_task_done before freeing that driver, so the give is safe.
    uac2_driver_t *driver = (uac2_driver_t *)arg;
    ESP_LOGD(TAG, "UAC2 event handling start");
    while (handle_events_for(driver, portMAX_DELAY) == ESP_OK) {
    }
    ESP_LOGD(TAG, "UAC2 event handling stop");
    xSemaphoreGive(driver->event_task_done);
    vTaskDelete(NULL);
}

// ── Physical device management (internal) ─────────────────────────

/**
 * Hand dev's in-flight control URB, its completion semaphore and the USB
 * device handle to the driver; the device stays open until reclaimed.
 */
static void orphan_ctrl_xfer(uac2_device_t *dev)
{
    uac2_orphaned_ctrl_t *node = dev->orphan_node;
    dev->orphan_node = NULL;
    node->xfer = dev->ctrl_xfer;
    node->done = dev->ctrl_xfer_done;
    node->dev_hdl = dev->dev_hdl;
    node->addr = dev->addr;
    UAC2_ENTER_CRITICAL();
    STAILQ_INSERT_TAIL(&s_uac2_driver->orphans_tailq, node, tailq_entry);
    UAC2_EXIT_CRITICAL();
    // The URB may have completed, and the event task run its reclaim pass,
    // since device_destroy() saw it pending. Wake the task so the next pass
    // sees this node instead of waiting for an unrelated USB event.
    usb_host_client_unblock(s_uac2_driver->client_handle);
    // node may already be reclaimed and freed by the event task here
    ESP_LOGW(TAG, "Control transfer still in flight at close of addr %d — keeping the device "
             "open until it completes (device responds or is unplugged)", dev->addr);
}

static bool addr_has_orphaned_ctrl(uint8_t addr)
{
    bool found = false;
    UAC2_ENTER_CRITICAL();
    uac2_orphaned_ctrl_t *node;
    STAILQ_FOREACH(node, &s_uac2_driver->orphans_tailq, tailq_entry) {
        if (node->addr == addr) {
            found = true;
            break;
        }
    }
    UAC2_EXIT_CRITICAL();
    return found;
}

/**
 * Drop this client's handle to a device that has carried control transfers.
 *
 * The in-tree USB Host Library of ESP-IDF 5.x hands a control completion to
 * the client before decrementing the device's in-flight count (usbh.c
 * handle_ep0_dequeue), and usbh_dev_close() asserts that count is 0 when the
 * last handle closes. A client or caller task with higher priority than the
 * USB Host library task can therefore get here before that bookkeeping runs.
 * Blocking briefly lets the library task finish it. espressif/usb >= 1.6.0,
 * which the manifest requires on IDF 6.x, retires the transfer first.
 */
static void close_device_after_ctrl(usb_device_handle_t dev_hdl)
{
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 0, 0)
    vTaskDelay(pdMS_TO_TICKS(10) > 0 ? pdMS_TO_TICKS(10) : 1);
#endif
    usb_host_device_close(s_uac2_driver->client_handle, dev_hdl);
}

/**
 * Free orphaned control URBs whose completion has been delivered. Runs on the
 * event task right after usb_host_client_handle_events(), so any callback for
 * these URBs has already returned. Only this function removes entries;
 * device_destroy() only appends.
 */
static void reclaim_orphaned_ctrl_xfers(void)
{
    uac2_driver_t *driver = s_uac2_driver;
    UAC2_ENTER_CRITICAL();
    uac2_orphaned_ctrl_t *node = STAILQ_FIRST(&driver->orphans_tailq);
    UAC2_EXIT_CRITICAL();
    while (node) {
        // Nodes are appended concurrently but only freed here, so node and
        // its successor stay valid across the unlocked section.
        UAC2_ENTER_CRITICAL();
        uac2_orphaned_ctrl_t *next = STAILQ_NEXT(node, tailq_entry);
        UAC2_EXIT_CRITICAL();

        if (xSemaphoreTake(node->done, 0) == pdTRUE) {
            UAC2_ENTER_CRITICAL();
            STAILQ_REMOVE(&driver->orphans_tailq, node, uac2_orphaned_ctrl, tailq_entry);
            UAC2_EXIT_CRITICAL();
            usb_host_transfer_free(node->xfer);
            vSemaphoreDelete(node->done);
            close_device_after_ctrl(node->dev_hdl);
            ESP_LOGI(TAG, "Reclaimed orphaned control transfer, addr %d closed", node->addr);
            heap_caps_free(node);
        }
        node = next;
    }
}

/**
 * Report devices that finished enumerating before this client registered.
 * USB Host only sends NEW_DEV to clients registered at the time, so without
 * this a driver (re)installed while a device stays attached never sees it.
 */
static void scan_attached_devices(void)
{
    uac2_driver_t *driver = s_uac2_driver;
    uint8_t addrs[UAC2_SCAN_MAX_DEVICES];
    int count = 0;
    if (usb_host_device_addr_list_fill(sizeof(addrs), addrs, &count) != ESP_OK) {
        return;
    }
    if (count > UAC2_SCAN_MAX_DEVICES) {
        count = UAC2_SCAN_MAX_DEVICES;
    }

    driver->scan_dedup_count = 0;
    driver->scan_dedup_start = xTaskGetTickCount();
    for (int i = 0; i < count; i++) {
        driver->scan_dedup_addrs[driver->scan_dedup_count++] = addrs[i];
        ESP_LOGD(TAG, "Already-attached device at addr %d", addrs[i]);
        uac2_host_device_connected(addrs[i]);
    }
}

/**
 * A device that finished enumerating just after this client registered shows
 * up in the install-time scan and also produces a NEW_DEV event. Drop that
 * duplicate so the user callback fires once per interface. Matching by
 * address is safe: ESP-IDF assigns addresses round-robin up to 127 (enum.c
 * get_next_dev_addr), so a re-enumerated device never reuses its address
 * within the window; scan_forget_addr() also drops it on DEV_GONE.
 */
static bool consume_scan_duplicate(uint8_t addr)
{
    uac2_driver_t *driver = s_uac2_driver;
    if (driver->scan_dedup_count == 0) {
        return false;
    }
    if ((xTaskGetTickCount() - driver->scan_dedup_start) > pdMS_TO_TICKS(UAC2_SCAN_DEDUP_WINDOW_MS)) {
        driver->scan_dedup_count = 0;
        return false;
    }
    for (int i = 0; i < driver->scan_dedup_count; i++) {
        if (driver->scan_dedup_addrs[i] == addr) {
            driver->scan_dedup_addrs[i] = driver->scan_dedup_addrs[--driver->scan_dedup_count];
            return true;
        }
    }
    return false;
}

/** A scanned device that went away cannot produce the duplicate NEW_DEV. Event task only. */
static void scan_forget_addr(uint8_t addr)
{
    uac2_driver_t *driver = s_uac2_driver;
    for (int i = 0; i < driver->scan_dedup_count; i++) {
        if (driver->scan_dedup_addrs[i] == addr) {
            driver->scan_dedup_addrs[i] = driver->scan_dedup_addrs[--driver->scan_dedup_count];
            return;
        }
    }
}

static esp_err_t device_create(uint8_t addr, uac2_device_t **out_dev)
{
    if (addr_has_orphaned_ctrl(addr)) {
        ESP_LOGE(TAG, "Device addr %d still has a timed-out control transfer pending; "
                 "it cannot be reopened until it responds or is replugged", addr);
        return ESP_ERR_INVALID_STATE;
    }

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
    dev->orphan_node = heap_caps_calloc(1, sizeof(uac2_orphaned_ctrl_t), MALLOC_CAP_DEFAULT);
    if (!dev->orphan_node) { err = ESP_ERR_NO_MEM; goto fail; }
    err = usb_host_transfer_alloc(sizeof(usb_setup_packet_t) + UAC2_CTRL_XFER_MAX_SIZE,
                                  0, &dev->ctrl_xfer);
    if (err != ESP_OK) goto fail;

    dev->ctrl_xfer_done = xSemaphoreCreateBinary();
    if (!dev->ctrl_xfer_done) { err = ESP_ERR_NO_MEM; goto fail; }

    dev->ctrl_mutex = xSemaphoreCreateMutex();
    if (!dev->ctrl_mutex) { err = ESP_ERR_NO_MEM; goto fail; }

    dev->stream_mutex = xSemaphoreCreateMutex();
    if (!dev->stream_mutex) { err = ESP_ERR_NO_MEM; goto fail; }

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
    // ctrl_request always releases the mutex within its own bounded timeouts.
    bool ctrl_in_flight = false;
    if (dev->ctrl_mutex) {
        xSemaphoreTake(dev->ctrl_mutex, portMAX_DELAY);
        ctrl_in_flight = dev->ctrl_xfer_abandoned &&
                         xSemaphoreTake(dev->ctrl_xfer_done, 0) != pdTRUE;
        xSemaphoreGive(dev->ctrl_mutex);
    }

    if (dev->ctrl_mutex) vSemaphoreDelete(dev->ctrl_mutex);
    if (dev->stream_mutex) vSemaphoreDelete(dev->stream_mutex);

    if (ctrl_in_flight) {
        // A timed-out control URB is still owned by the host stack. Freeing it
        // now would let its late completion touch freed memory, and closing the
        // device with it in flight trips an ESP-IDF assert. Hand both to the
        // driver to release on the event task once the URB completes.
        orphan_ctrl_xfer(dev);
    } else {
        if (dev->ctrl_xfer) usb_host_transfer_free(dev->ctrl_xfer);
        if (dev->ctrl_xfer_done) vSemaphoreDelete(dev->ctrl_xfer_done);
        close_device_after_ctrl(dev->dev_hdl);
    }
    if (dev->orphan_node) heap_caps_free(dev->orphan_node);
    ESP_LOGI(TAG, "Physical device addr %d closed", dev->addr);
    heap_caps_free(dev);
}

// ── Public API: Driver lifecycle ──────────────────────────────────

esp_err_t uac2_host_install(const uac2_host_driver_config_t *config)
{
    ESP_RETURN_ON_FALSE(config, ESP_ERR_INVALID_ARG, TAG, "Config is NULL");
    ESP_RETURN_ON_FALSE(config->callback, ESP_ERR_INVALID_ARG, TAG, "Callback is NULL");

    if (config->create_background_task) {
        ESP_RETURN_ON_FALSE(config->stack_size != 0, ESP_ERR_INVALID_ARG, TAG, "Wrong stack size");
        ESP_RETURN_ON_FALSE(config->task_priority != 0, ESP_ERR_INVALID_ARG, TAG, "Wrong task priority");
    }

    SemaphoreHandle_t lock = lifecycle_mutex();
    xSemaphoreTake(lock, portMAX_DELAY);
    if (s_uac2_driver) {
        xSemaphoreGive(lock);
        ESP_LOGE(TAG, "UAC2 driver already installed");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = ESP_ERR_NO_MEM;
    uac2_driver_t *driver = heap_caps_calloc(1, sizeof(uac2_driver_t), MALLOC_CAP_DEFAULT);
    if (!driver) goto fail;

    driver->user_cb = config->callback;
    driver->user_arg = config->callback_arg;
    driver->end_client_event_handling = false;
    STAILQ_INIT(&driver->devices_tailq);
    STAILQ_INIT(&driver->ifaces_tailq);
    STAILQ_INIT(&driver->orphans_tailq);

    driver->all_events_handled = xSemaphoreCreateBinary();
    if (!driver->all_events_handled) goto fail;
    if (config->create_background_task) {
        driver->event_task_done = xSemaphoreCreateBinary();
        if (!driver->event_task_done) goto fail;
    }

    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .async.client_event_callback = client_event_cb,
        .async.callback_arg = NULL,
        .max_num_event_msg = 16,
    };
    err = usb_host_client_register(&client_config, &driver->client_handle);
    if (err != ESP_OK) {
        driver->client_handle = NULL;
        goto fail;
    }

    UAC2_ENTER_CRITICAL();
    s_uac2_driver = driver;
    UAC2_EXIT_CRITICAL();

    if (config->create_background_task) {
        BaseType_t task_created = xTaskCreatePinnedToCore(
            event_handler_task, "uac2_host", config->stack_size,
            driver, config->task_priority, NULL, config->core_id);
        if (!task_created) {
            UAC2_ENTER_CRITICAL();
            s_uac2_driver = NULL;
            UAC2_EXIT_CRITICAL();
            err = ESP_ERR_NO_MEM;
            goto fail;
        }
    }

    xSemaphoreGive(lock);
    ESP_LOGI(TAG, "UAC2 Host driver installed, v%d.%d.%d",
             UAC2_HOST_VER_MAJOR, UAC2_HOST_VER_MINOR, UAC2_HOST_VER_PATCH);
    return ESP_OK;

fail:
    if (driver) {
        if (driver->client_handle) usb_host_client_deregister(driver->client_handle);
        if (driver->event_task_done) vSemaphoreDelete(driver->event_task_done);
        if (driver->all_events_handled) vSemaphoreDelete(driver->all_events_handled);
        heap_caps_free(driver);
    }
    xSemaphoreGive(lock);
    return err;
}

esp_err_t uac2_host_uninstall(void)
{
    SemaphoreHandle_t lock = lifecycle_mutex();
    xSemaphoreTake(lock, portMAX_DELAY);

    UAC2_ENTER_CRITICAL();
    uac2_driver_t *driver = s_uac2_driver;
    if (!driver) {
        UAC2_EXIT_CRITICAL();
        xSemaphoreGive(lock);
        return ESP_OK;
    }
    if (driver->end_client_event_handling) {
        // Another uninstall is already tearing this driver down.
        UAC2_EXIT_CRITICAL();
        xSemaphoreGive(lock);
        return ESP_ERR_INVALID_STATE;
    }
    if (!STAILQ_EMPTY(&driver->devices_tailq) ||
        !STAILQ_EMPTY(&driver->ifaces_tailq)) {
        UAC2_EXIT_CRITICAL();
        xSemaphoreGive(lock);
        ESP_LOGE(TAG, "Cannot uninstall: devices/interfaces still open");
        return ESP_ERR_INVALID_STATE;
    }
    if (!STAILQ_EMPTY(&driver->orphans_tailq)) {
        // Deregistering now would leave the host stack completing a URB for a
        // client that no longer exists. The URB completes once the hung device
        // answers or is disconnected.
        UAC2_EXIT_CRITICAL();
        xSemaphoreGive(lock);
        ESP_LOGE(TAG, "Cannot uninstall: a timed-out control transfer is still pending");
        return ESP_ERR_INVALID_STATE;
    }
    driver->end_client_event_handling = true;
    bool wait_for_events = driver->handling_events;
    UAC2_EXIT_CRITICAL();
    // Released while waiting: driver callbacks run inside handle_events and
    // may hand off to tasks that call open(), which now fails fast.
    xSemaphoreGive(lock);

    if (wait_for_events) {
        esp_err_t unblock_err = usb_host_client_unblock(driver->client_handle);
        if (unblock_err != ESP_OK) {
            ESP_LOGE(TAG, "client_unblock failed: %s", esp_err_to_name(unblock_err));
        }
        xSemaphoreTake(driver->all_events_handled, portMAX_DELAY);
    }
    if (driver->event_task_done) {
        // Even if it never got to run yet: its next call sees the end flag.
        xSemaphoreTake(driver->event_task_done, portMAX_DELAY);
    }

    xSemaphoreTake(lock, portMAX_DELAY);
    esp_err_t dereg_err = usb_host_client_deregister(driver->client_handle);
    if (dereg_err != ESP_OK) {
        ESP_LOGE(TAG, "client_deregister failed: %s", esp_err_to_name(dereg_err));
    }
    UAC2_ENTER_CRITICAL();
    s_uac2_driver = NULL;
    UAC2_EXIT_CRITICAL();
    xSemaphoreGive(lock);

    if (driver->event_task_done) vSemaphoreDelete(driver->event_task_done);
    vSemaphoreDelete(driver->all_events_handled);
    heap_caps_free(driver);
    ESP_LOGI(TAG, "UAC2 Host driver uninstalled");
    return ESP_OK;
}

/**
 * One event-pump iteration. With expected != NULL (the background task) it
 * only runs for that installation. The driver pointer is read and the call
 * registered under the static lock, so a call racing uninstall is either
 * waited for or never touches the driver.
 */
static esp_err_t handle_events_for(uac2_driver_t *expected, TickType_t timeout)
{
    UAC2_ENTER_CRITICAL();
    uac2_driver_t *driver = s_uac2_driver;
    if (!driver || (expected && driver != expected)) {
        UAC2_EXIT_CRITICAL();
        return ESP_ERR_INVALID_STATE;
    }
    if (driver->end_client_event_handling) {
        UAC2_EXIT_CRITICAL();
        return ESP_FAIL;
    }
    if (driver->handling_events) {
        // usb_host_client_handle_events() supports one caller per client
        UAC2_EXIT_CRITICAL();
        return ESP_ERR_INVALID_STATE;
    }
    driver->handling_events = true;
    bool scan = !driver->initial_scan_done;
    driver->initial_scan_done = true;
    UAC2_EXIT_CRITICAL();

    if (scan) {
        // Done here rather than in install so driver callbacks for already-
        // attached devices run in the same context as NEW_DEV callbacks.
        scan_attached_devices();
    }
    esp_err_t ret = usb_host_client_handle_events(driver->client_handle, timeout);
    reclaim_orphaned_ctrl_xfers();

    UAC2_ENTER_CRITICAL();
    bool ending = driver->end_client_event_handling;
    driver->handling_events = false;
    UAC2_EXIT_CRITICAL();
    if (ending) {
        // The end flag was set while this call ran, so uninstall is waiting.
        xSemaphoreGive(driver->all_events_handled);
        return ESP_FAIL;
    }
    return ret;
}

esp_err_t uac2_host_handle_events(TickType_t timeout)
{
    return handle_events_for(NULL, timeout);
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

    uac2_device_t *dev = iface->parent;
    uint8_t remaining = 0;
    esp_err_t io_err = unlink_iface_when_unreferenced(iface, pdMS_TO_TICKS(2000), &remaining);
    if (io_err != ESP_OK) {
        ESP_LOGE(TAG, "device_close: timed out waiting for pending I/O");
        atomic_store(&iface->closing, false);
        driver_lifecycle_unlock();
        return io_err;
    }

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
    // The clock is shared by every interface on the device, and a running or
    // suspended stream sizes its packets from the rate it was started with.
    // stream_mutex keeps a sibling from starting between the check and the SET.
    xSemaphoreTake(dev->stream_mutex, portMAX_DELAY);
    if (device_has_stream(dev)) {
        ESP_LOGE(TAG, "Stop streams on this device before changing the sample rate "
                 "(device_start applies the stream's sample_freq)");
        ret = ESP_ERR_INVALID_STATE;
    } else {
        ret = validate_sample_rate(dev, sample_rate);
        if (ret == ESP_OK) {
            ret = set_sample_rate_internal(dev, sample_rate);
        }
        if (ret == ESP_OK) {
            warn_if_rate_not_applied(dev, sample_rate);
        }
    }
    xSemaphoreGive(dev->stream_mutex);
    release_locked_iface(iface);
    return ret;
}

esp_err_t uac2_host_device_get_sample_rate_range(uac2_host_device_handle_t handle,
                                                 uac2_sample_rate_range_t *ranges,
                                                 uint8_t *num_ranges)
{
    ESP_RETURN_ON_FALSE(handle && ranges && num_ranges, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    uint8_t capacity = *num_ranges ? *num_ranges : UAC2_MAX_SAMPLE_RATE_RANGES;
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
    if (count > capacity) {
        ESP_LOGW(TAG, "Device reports %u sample-rate ranges, caller has room for %u",
                 (unsigned)count, (unsigned)capacity);
        count = capacity;
    }

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

/**
 * Descriptor checks for carrying this stream on an alt setting, before
 * anything is allocated. ESP_ERR_NOT_SUPPORTED if it cannot.
 */
static esp_err_t check_alt_usable(const uac2_iface_t *iface, const uac2_as_iface_t *as,
                                  const uac2_host_stream_config_t *config)
{
    if (fractional_playback_requires_feedback(iface->dir, as->fb_ep_addr,
                                              UAC2_EP_SYNC_TYPE(as->ep_attributes),
                                              config->sample_freq)) {
        ESP_LOGE(TAG, "Playback %.2f kHz on alt %d requires a feedback endpoint "
                 "(asynchronous endpoint without one)",
                 config->sample_freq / 1000.0, as->alt_setting);
        return ESP_ERR_NOT_SUPPORTED;
    }

    // Endpoint intervals come from the (untrusted) device descriptor and reach
    // the HCD unmodified through usb_host_interface_claim().
    if (!iso_ep_interval_supported(as->ep_interval, UAC2_NUM_PACKETS_PER_URB, true)) {
        ESP_LOGE(TAG, "Data EP 0x%02X bInterval=%d unsupported (full-speed audio requires 1)",
                 as->ep_addr, as->ep_interval);
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (iface->dir == UAC2_STREAM_TX && as->fb_ep_addr != 0 &&
        !iso_ep_interval_supported(as->fb_ep_interval, 1, false)) {
        ESP_LOGE(TAG, "Feedback EP 0x%02X bInterval=%d unsupported by the USB host controller",
                 as->fb_ep_addr, as->fb_ep_interval);
        return ESP_ERR_NOT_SUPPORTED;
    }
    return ESP_OK;
}

/**
 * Body of device_start. Caller holds the api lock and dev->stream_mutex;
 * this function never releases either.
 *
 * *alt_specific is set when the failure belongs to this alt setting (packet
 * size above its MPS, or the host could not allocate its endpoints) and the
 * stream was left IDLE with nothing allocated, so another alt may be tried.
 */
static esp_err_t stream_start_reserved(uac2_iface_t *iface, const uac2_as_iface_t *as,
                                       const uac2_host_stream_config_t *config,
                                       bool *alt_specific)
{
    uac2_device_t *dev = iface->parent;
    *alt_specific = false;

    ESP_LOGI(TAG, "Starting %s stream: iface %d alt %d, %dch %d-bit, ep 0x%02X (MPS %d)",
             iface->dir == UAC2_STREAM_TX ? "TX" : "RX",
             as->interface_num, as->alt_setting,
             as->nr_channels, as->bit_resolution,
             as->ep_addr, as->ep_max_packet_size);

    // Allocate stream resources
    esp_err_t err = stream_resources_alloc(iface, as, config);
    if (err != ESP_OK) {
        *alt_specific = err == ESP_ERR_INVALID_SIZE && !stream_has_resources(iface);
        return err;
    }

    // Claim interface
    err = usb_host_interface_claim(s_uac2_driver->client_handle, dev->dev_hdl,
                                   iface->iface_num, iface->alt_setting);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to claim iface %d alt %d: %s",
                 iface->iface_num, iface->alt_setting, esp_err_to_name(err));
        esp_err_t free_err = stream_resources_free(iface);
        if (free_err != ESP_OK) {
            err = free_err;     // resources retained; device_close retries cleanup
        } else {
            // The HCD refuses endpoints it cannot fit, e.g. an MPS above the
            // FIFO the host configured ("EP MPS exceeds supported limit")
            *alt_specific = err == ESP_ERR_NOT_SUPPORTED;
        }
        return err;
    }
    iface->interface_claimed = true;
    iface->state = UAC2_IFACE_STATE_READY;

    // Set the clock before activating the alt setting, the order Linux
    // snd-usb-audio uses for UAC2 (snd_usb_endpoint_prepare): some devices need
    // the rate first, and the endpoint is never enabled without a clock (a
    // source of start-up noise on some DACs)
    if (dev->clock_source_id != 0 && config->sample_freq > 0) {
        err = ensure_sample_rate_applied(dev, config->sample_freq);
        if (err != ESP_OK) {
            esp_err_t cleanup_err = stream_abort_startup(iface);
            if (cleanup_err != ESP_OK) {
                ESP_LOGE(TAG, "Startup abort failed after sample-rate error %s: %s",
                         esp_err_to_name(err), esp_err_to_name(cleanup_err));
                err = cleanup_err;
            }
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
            return err;
        }
    }

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
        return err;
    }
    err = ensure_iface_device_available(iface, UAC2_IFACE_STATE_READY,
                                        "device_start after SET_INTERFACE");
    if (err != ESP_OK) {
        esp_err_t cleanup_err = stream_abort_startup(iface);
        if (cleanup_err != ESP_OK) {
            ESP_LOGE(TAG, "Startup abort failed after disconnect during SET_INTERFACE: %s",
                     esp_err_to_name(cleanup_err));
            err = cleanup_err;
        }
        return err;
    }

    // Check for suspend-after-start flag
    if (config->flags & UAC2_FLAG_STREAM_SUSPEND_AFTER_START) {
        ESP_LOGI(TAG, "%s stream started (suspended)", iface->dir == UAC2_STREAM_TX ? "TX" : "RX");
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
        return err;
    }

    ESP_LOGI(TAG, "%s stream started (pkt_size=%d, ringbuf=%" PRIu32 ")",
             iface->dir == UAC2_STREAM_TX ? "TX" : "RX",
             iface->packet_size, iface->ringbuf_size);
    return ESP_OK;
}

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

    // A disconnect leaves the stream IDLE with its resources until close()
    // (or stop()) reclaims them; allocating over them would leak them and
    // reset the in-flight count under callbacks that may still be pending
    if (atomic_load(&dev->gone) || stream_has_resources(iface)) {
        ESP_LOGE(TAG, "Cannot start: %s", atomic_load(&dev->gone)
                 ? "device disconnected (close the handle)"
                 : "previous stream not torn down (call stop or close)");
        release_locked_iface(iface);
        return ESP_ERR_INVALID_STATE;
    }

    // AS alt settings of this interface that carry the requested format, best first
    const uac2_as_iface_t *candidates[UAC2_MAX_AS_INTERFACES];
    int num_candidates = find_matching_as_ifaces(&dev->desc_info, iface->dir, config,
                                                 iface->iface_num, candidates);
    if (num_candidates == 0) {
        ESP_LOGE(TAG, "No matching AS interface for %s %dch/%dbit",
                 iface->dir == UAC2_STREAM_TX ? "TX" : "RX",
                 config->channels, config->bit_resolution);
        release_locked_iface(iface);
        return ESP_ERR_NOT_FOUND;
    }

    xSemaphoreTake(dev->stream_mutex, portMAX_DELAY);
    if (device_has_opposite_direction_stream(iface)) {
        ESP_LOGE(TAG, "ESP32-S3: a stream in the other direction is running "
                 "(simultaneous TX and RX needs CONFIG_UAC2_ALLOW_DUPLEX and a custom FIFO split)");
        ret = ESP_ERR_NOT_SUPPORTED;
    } else if (device_has_stream_at_other_rate(iface, config->sample_freq)) {
        ESP_LOGE(TAG, "Another stream on this device runs at a different sample rate (shared clock)");
        ret = ESP_ERR_INVALID_STATE;
    } else {
        // Move on to the next alt only when this one cannot carry the stream
        // (descriptor checks, packet size, endpoints the host cannot fit); a
        // device-level failure (control request, disconnect) ends the attempt
        for (int i = 0; i < num_candidates; i++) {
            bool alt_specific = true;
            ret = check_alt_usable(iface, candidates[i], config);
            if (ret == ESP_OK) {
                ret = stream_start_reserved(iface, candidates[i], config, &alt_specific);
            }
            if (ret == ESP_OK || !alt_specific) {
                break;
            }
            if (i + 1 < num_candidates) {
                ESP_LOGW(TAG, "Alt %d cannot carry the stream (%s), trying alt %d",
                         candidates[i]->alt_setting, esp_err_to_name(ret),
                         candidates[i + 1]->alt_setting);
            }
        }
    }
    xSemaphoreGive(dev->stream_mutex);
    release_locked_iface(iface);
    return ret;
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

    // Let the flushed URBs' callbacks run before releasing (see release_interface_claim)
    esp_err_t wait_err = wait_for_urbs_quiesced(iface, 2000, "Stream stop", true);
    if (wait_err != ESP_OK) {
        return wait_err;
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

    esp_err_t free_err = stream_resources_free(iface);
    if (free_err != ESP_OK) {
        portENTER_CRITICAL(&iface->state_lock);
        iface->state = UAC2_IFACE_STATE_ERROR;
        portEXIT_CRITICAL(&iface->state_lock);
        return free_err;
    }
    // Everything is torn down: IDLE again even if a callback that raced the
    // start of this stop marked the stream ERROR
    portENTER_CRITICAL(&iface->state_lock);
    iface->state = UAC2_IFACE_STATE_IDLE;
    portEXIT_CRITICAL(&iface->state_lock);
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
    uac2_iface_t *iface = acquire_stream_io_ref(handle);
    if (!iface) return ESP_ERR_INVALID_STATE;

    if (!acquire_ringbuf_user(iface)) {
        release_iface_io_ref(iface);
        return ESP_ERR_INVALID_STATE;
    }

    BaseType_t ok = xRingbufferSend(iface->ringbuf, data, size,
                                    pdMS_TO_TICKS(timeout_ms));

    bool still_active = stream_is_active(iface);
    if (ok == pdTRUE && still_active) {
        atomic_store(&iface->tx_done_pending, false);
    }

    atomic_fetch_sub(&iface->rw_users, 1);
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
    *bytes_read = 0;
    uac2_iface_t *iface = acquire_stream_io_ref(handle);
    if (!iface) return ESP_ERR_INVALID_STATE;

    if (!acquire_ringbuf_user(iface)) {
        release_iface_io_ref(iface);
        return ESP_ERR_INVALID_STATE;
    }

    size_t item_size = 0;
    void *item = xRingbufferReceiveUpTo(iface->ringbuf, &item_size,
                                        pdMS_TO_TICKS(timeout_ms), size);

    esp_err_t ret;
    bool still_active = stream_is_active(iface);
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

    atomic_fetch_sub(&iface->rw_users, 1);
    release_iface_io_ref(iface);
    return ret;
}

int64_t uac2_host_device_get_start_time(uac2_host_device_handle_t handle)
{
    if (!handle) return 0;
    uac2_iface_t *iface = acquire_iface_io_ref(handle);
    if (!iface) return 0;
    // The cached value outlives stop/unplug; documented as 0 without a stream
    int64_t start = stream_is_live(iface) ? atomic_load(&iface->first_frame_us) : 0;
    release_iface_io_ref(iface);
    return start;
}

uint32_t uac2_host_device_get_feedback(uac2_host_device_handle_t handle)
{
    if (!handle) return 0;
    uac2_iface_t *iface = acquire_iface_io_ref(handle);
    if (!iface) return 0;
    uint32_t feedback = stream_is_live(iface) ? atomic_load(&iface->fb_value) : 0;
    release_iface_io_ref(iface);
    return feedback;
}

esp_err_t uac2_host_device_get_stream_info(uac2_host_device_handle_t handle,
                                           uac2_host_stream_info_t *info)
{
    ESP_RETURN_ON_FALSE(handle && info, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    uac2_iface_t *iface = NULL;
    esp_err_t ret = acquire_locked_iface(handle, &iface);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "Invalid handle");

    portENTER_CRITICAL(&iface->state_lock);
    uac2_iface_state_t state = iface->state;
    portEXIT_CRITICAL(&iface->state_lock);
    if (state != UAC2_IFACE_STATE_ACTIVE && state != UAC2_IFACE_STATE_READY) {
        release_locked_iface(iface);
        return ESP_ERR_INVALID_STATE;
    }

    *info = (uac2_host_stream_info_t) {
        .alt_setting = iface->alt_setting,
        .channels = iface->channels,
        .bit_resolution = iface->bit_resolution,
        .sub_slot_size = iface->sub_slot_size,
        .sample_freq = iface->sample_rate,
        .ep_max_packet_size = iface->ep_mps,
        .ep_addr = iface->ep_addr,
        .fb_ep_addr = iface->fb_ep_addr,
        .sync_type = (uac2_ep_sync_type_t)iface->ep_sync_type,
        .active = state == UAC2_IFACE_STATE_ACTIVE,
    };
    release_locked_iface(iface);
    return ESP_OK;
}

esp_err_t uac2_host_device_get_buffer_status(uac2_host_device_handle_t handle,
                                             uac2_host_buffer_status_t *status)
{
    ESP_RETURN_ON_FALSE(handle && status, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    uac2_iface_t *iface = acquire_stream_io_ref(handle);
    if (!iface) return ESP_ERR_INVALID_STATE;

    if (!acquire_ringbuf_user(iface)) {
        release_iface_io_ref(iface);
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t capacity = iface->ringbuf_size;
    size_t free_bytes = xRingbufferGetCurFreeSize(iface->ringbuf);
    uint32_t queued = free_bytes < capacity ? capacity - (uint32_t)free_bytes : 0;
    uint64_t byte_rate = (uint64_t)iface->sample_rate * iface->channels * iface->sub_slot_size;
    // urbs_in_flight also counts the feedback URB
    int urbs = atomic_load(&iface->urbs_in_flight);
    if (urbs > iface->xfer_count) urbs = iface->xfer_count;
    if (urbs < 0) urbs = 0;

    *status = (uac2_host_buffer_status_t) {
        .ring_bytes = queued,
        .ring_capacity = capacity,
        .ring_us = byte_rate ? (uint32_t)(((uint64_t)queued * 1000000) / byte_rate) : 0,
        .in_flight_us = (uint32_t)urbs * UAC2_NUM_PACKETS_PER_URB * 1000,
    };

    atomic_fetch_sub(&iface->rw_users, 1);
    release_iface_io_ref(iface);
    return ESP_OK;
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
    uint8_t capacity = *num_ranges ? *num_ranges : UAC2_MAX_VOLUME_RANGES;
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
    if (count > capacity) {
        ESP_LOGW(TAG, "Device reports %u volume ranges, caller has room for %u",
                 (unsigned)count, (unsigned)capacity);
        count = capacity;
    }
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
    int16_t res_db256 = 0;
    err = get_volume_range_triplet(iface, channel, &min_db256, &max_db256, &res_db256);
    if (err != ESP_OK) {
        release_locked_iface(iface);
        return err == ESP_ERR_NOT_FOUND ? ESP_ERR_NOT_SUPPORTED : err;
    }
    int32_t span = (int32_t)max_db256 - min_db256;
    int32_t offset = span > 0 ? (span * percent) / 100 : 0;
    if (res_db256 > 0 && offset > 0) {
        // Snap to the device's step grid so the request is a supported value
        offset = ((offset + res_db256 / 2) / res_db256) * res_db256;
        if (offset > span) {
            offset -= res_db256;
        }
    }
    int16_t db256 = (int16_t)(min_db256 + offset);
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

/**
 * Write mute or volume to every channel in ch_map (bit 0 = master). Some
 * devices advertise a master control that STALLs while their per-channel
 * controls work, so a failed master write does not stop the channel writes;
 * it is only reported if there were no channel controls to fall back to.
 */
static esp_err_t set_all_channels(uac2_host_device_handle_t handle, uint32_t ch_map,
                                  uint8_t nr_channels, bool is_mute, int16_t volume_db256,
                                  bool mute)
{
    esp_err_t master_err = ESP_OK;
    bool wrote_channel = false;
    for (uint8_t ch = 0; ch <= nr_channels && ch < 32; ch++) {
        if (!(ch_map & (1u << ch))) {
            continue;
        }
        esp_err_t err = is_mute ? uac2_host_device_set_mute(handle, ch, mute)
                                : uac2_host_device_set_volume(handle, ch, volume_db256);
        if (err != ESP_OK) {
            if (ch != 0) return err;
            master_err = err;
            continue;
        }
        if (ch != 0) wrote_channel = true;
    }
    if (master_err != ESP_OK) {
        if (!wrote_channel) return master_err;
        ESP_LOGW(TAG, "Master %s failed (%s); set per channel instead",
                 is_mute ? "mute" : "volume", esp_err_to_name(master_err));
    }
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

    return set_all_channels(handle, ch_map, nr_channels, false, volume_db256, false);
}

esp_err_t uac2_host_device_set_mute_all_channels(uac2_host_device_handle_t handle, bool mute)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid handle");
    uac2_iface_t *iface = NULL;
    esp_err_t err = acquire_iface_runtime_ref(handle, &iface);
    ESP_RETURN_ON_FALSE(err == ESP_OK, err, TAG, "Invalid handle");
    if (!iface->has_feature_unit || !iface->has_mute) {
        release_iface_io_ref(iface);
        return ESP_ERR_NOT_SUPPORTED;
    }

    const uac2_feature_unit_t *fu = get_iface_feature_unit(iface);
    if (!fu) {
        release_iface_io_ref(iface);
        return ESP_ERR_NOT_FOUND;
    }
    uint32_t ch_map = fu->mute_ch_map;
    uint8_t nr_channels = fu->nr_channels;
    release_iface_io_ref(iface);

    return set_all_channels(handle, ch_map, nr_channels, true, 0, mute);
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

    esp_err_t wait_err = wait_for_urbs_quiesced(iface, 2000, "Deactivate", false);
    if (wait_err != ESP_OK) {
        return wait_err;
    }

    if (iface->interface_claimed) {
        esp_err_t rel_err = release_interface_claim(dev, iface, pdMS_TO_TICKS(2000));
        if (rel_err != ESP_OK) {
            return rel_err;
        }
        iface->interface_claimed = false;
    }
    return ESP_OK;
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
    if (ret == ESP_OK) {
        // A writer that raced the deactivate must finish before the flush, or
        // its stale audio would be played after resume.
        ret = wait_for_rw_users_drained(iface, 2000, "Suspend");
    }
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

/**
 * Body of device_resume. Caller holds the api lock and dev->stream_mutex;
 * this function never releases either.
 */
static esp_err_t stream_resume_reserved(uac2_iface_t *iface)
{
    uac2_device_t *dev = iface->parent;
    esp_err_t ret;

    if (!iface->interface_claimed) {
        ret = usb_host_interface_claim(s_uac2_driver->client_handle, dev->dev_hdl,
                                       iface->iface_num, iface->alt_setting);
        if (ret != ESP_OK) {
            return ret;
        }
        iface->interface_claimed = true;
    }

    // Clock first, then the alt setting, as in stream_start_reserved()
    if (dev->clock_source_id != 0 && iface->sample_rate > 0) {
        ret = ensure_sample_rate_applied(dev, iface->sample_rate);
        if (ret != ESP_OK) {
            return resume_rollback_to_ready(iface, ret, "sample-rate setup");
        }
        ret = ensure_iface_device_available(iface, UAC2_IFACE_STATE_READY,
                                            "device_resume after sample-rate setup");
        if (ret != ESP_OK) {
            return resume_rollback_to_ready(iface, ret, "disconnect after sample-rate setup");
        }
    }

    ret = ctrl_request_no_data(dev,
        USB_BM_REQUEST_TYPE_DIR_OUT | USB_BM_REQUEST_TYPE_TYPE_STANDARD | USB_BM_REQUEST_TYPE_RECIP_INTERFACE,
        USB_B_REQUEST_SET_INTERFACE, iface->alt_setting, iface->iface_num);
    if (ret != ESP_OK) {
        return resume_rollback_to_ready(iface, ret, "SET_INTERFACE");
    }
    ret = ensure_iface_device_available(iface, UAC2_IFACE_STATE_READY,
                                        "device_resume after SET_INTERFACE");
    if (ret != ESP_OK) {
        return resume_rollback_to_ready(iface, ret, "disconnect after SET_INTERFACE");
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

    return ret;
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

    if (fractional_playback_requires_feedback(iface->dir, iface->fb_ep_addr, iface->ep_sync_type,
                                              iface->sample_rate)) {
        release_locked_iface(iface);
        return ESP_ERR_NOT_SUPPORTED;
    }

    xSemaphoreTake(dev->stream_mutex, portMAX_DELAY);
    if (device_has_opposite_direction_stream(iface)) {
        ESP_LOGE(TAG, "ESP32-S3: a stream in the other direction is running "
                 "(simultaneous TX and RX needs CONFIG_UAC2_ALLOW_DUPLEX and a custom FIFO split)");
        ret = ESP_ERR_NOT_SUPPORTED;
    } else if (device_has_stream_at_other_rate(iface, iface->sample_rate)) {
        ESP_LOGE(TAG, "Another stream on this device runs at a different sample rate (shared clock)");
        ret = ESP_ERR_INVALID_STATE;
    } else {
        ret = stream_resume_reserved(iface);
    }
    xSemaphoreGive(dev->stream_mutex);
    release_locked_iface(iface);
    return ret;
}
