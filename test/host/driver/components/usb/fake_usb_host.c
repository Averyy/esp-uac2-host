/*
 * SPDX-License-Identifier: MIT
 *
 * Fake ESP-IDF USB Host Library for running the UAC2 driver on the ESP-IDF
 * Linux target. It implements the client/device/interface/transfer subset of
 * usb/usb_host.h that the driver calls, against one simulated device on a
 * 1 ms "bus" task, and mirrors the real library where the driver depends on
 * it (see the notes on each function). Where the real library would assert or
 * corrupt memory, it counts a violation in fake_usb_stats_t instead.
 */

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_idf_version.h"
#include "usb/usb_host.h"
#include "usb/usb_helpers.h"
#include "fake_usb.h"
#include "minidsp_2x4hd_descriptors.h"

static const char *TAG = "fake_usb";

#define MAX_IFACES      8
#define DONE_QUEUE_LEN  128
#define EVENT_QUEUE_LEN 16

// UAC2 entities of the captured miniDSP 2x4 HD
#define ENT_CLOCK_SOURCE    41
#define ENT_CLOCK_SELECTOR  40
#define ENT_FU_PLAYBACK     10
#define ENT_FU_CAPTURE      11

typedef struct fake_xfer {
    struct fake_xfer *next;     // pending list
    struct fake_xfer *all_next; // every allocated transfer
    bool in_flight;             // submitted, not completed
    bool queued;                // completed, callback not started (in the done queue)
    bool freed_while_queued;    // client freed it early; released when dequeued
    bool hung;                  // control transfer left unanswered
    uint64_t due_frame;
    bool is_feedback;
    bool is_out_data;
    usb_transfer_t xfer;        // last: ends in a flexible array
} fake_xfer_t;

#define WRAP(x) ((fake_xfer_t *)((uint8_t *)(x) - offsetof(fake_xfer_t, xfer)))

typedef struct {
    bool gone;
    uint8_t addr;
    int open_count;             // opens by the (single) client
    int ctrl_in_flight;
    bool claimed[MAX_IFACES];
    uint8_t claimed_alt[MAX_IFACES];
    bool halted[32];
    uint64_t ep_next_frame[32];   // URBs on one endpoint run back to back, as in the HCD
    int ctrl_retire_pending;      // completed control transfers not yet retired (IDF 5.x order)
    int ep_busy[32];              // completions whose callbacks have not returned (usb_host.c
                                  // num_urb_inflight); kept per endpoint, never in the transfer
    uint8_t *cfg;
    uint16_t cfg_len;
    uint32_t sample_rate;
    bool rate_set_since_alt;      // a sample-rate SET arrived since the last SET_INTERFACE(alt != 0)
    uint8_t mute[2][8];
    int16_t volume[2][8];
} fake_dev_t;

typedef struct {
    usb_host_client_event_cb_t cb;
    void *arg;
    volatile bool handling;     // inside usb_host_client_handle_events()
    SemaphoreHandle_t event_sem;
    QueueHandle_t done_q;
    QueueHandle_t event_q;
} fake_client_t;

static SemaphoreHandle_t s_lock;
static TaskHandle_t s_lock_holder;      // diagnostics only
static TaskHandle_t s_bus_task;
static fake_client_t *s_client;
static fake_dev_t *s_dev;
static uint8_t s_next_addr = 1;
static fake_xfer_t *s_pending;
static fake_xfer_t *s_all_xfers;
static uint64_t s_frame;
static fake_usb_stats_t s_stats;

static uint32_t s_feedback_q16 = 0x00300000;
static usb_transfer_status_t s_fb_status;
static int s_fb_status_count;
static usb_transfer_status_t s_out_fail_status;
static int s_out_fail_count;
static int s_release_busy;
static struct {
    bool armed;
    uint8_t entity;
    uint8_t cs;
    bool get;
} s_hang;
static uint16_t s_max_iso_out_mps;
static uint8_t s_capture_next;          // next byte of the capture stream's counter pattern
static int s_alloc_fail_after = -1;     // transfer allocations left before one fails (-1: never)
static uint8_t s_out_prev;              // last nonzero playback byte (0: none yet)
#define MAX_CLOCK_RATES 21              // 2 + 21 * 12 bytes fit the reply scratch buffer
static uint32_t s_clock_rates[MAX_CLOCK_RATES];
static int s_clock_rate_count;
static struct {
    bool armed;
    uint8_t entity;
    uint8_t cs;
    uint8_t ch;
    bool get;
} s_stall;

// UTF-16 "miniDSP 2x4HD [fake]" string descriptor
static const uint16_t s_product_str[] = {
    (0x03 << 8) | (2 + 2 * 20),
    'm', 'i', 'n', 'i', 'D', 'S', 'P', ' ', '2', 'x', '4', 'H', 'D', ' ', '[', 'f', 'a', 'k', 'e', ']',
};

static int ep_index(uint8_t ep_addr)
{
    return (ep_addr & 0x0F) + ((ep_addr & 0x80) ? 16 : 0);
}

static void lock(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_lock_holder = xTaskGetCurrentTaskHandle();
}

static void unlock(void)
{
    s_lock_holder = NULL;
    xSemaphoreGive(s_lock);
}

static void wake_client(void)
{
    if (s_client) {
        xSemaphoreGive(s_client->event_sem);
    }
}

// ── Pending transfers ────────────────────────────────────────────

static void pending_remove(fake_xfer_t *fx)
{
    for (fake_xfer_t **pp = &s_pending; *pp; pp = &(*pp)->next) {
        if (*pp == fx) {
            *pp = fx->next;
            break;
        }
    }
    fx->next = NULL;
}

static void pending_append(fake_xfer_t *fx)
{
    fx->next = NULL;
    fake_xfer_t **pp = &s_pending;
    while (*pp) {
        pp = &(*pp)->next;
    }
    *pp = fx;
}

/** Hand a finished transfer to the client; its callback runs in handle_events. Caller holds the lock. */
static void complete_locked(fake_xfer_t *fx)
{
    pending_remove(fx);
    fx->in_flight = false;
    fx->hung = false;
    fx->queued = true;
    if (s_dev && fx->xfer.bEndpointAddress != 0) {
        s_dev->ep_busy[ep_index(fx->xfer.bEndpointAddress)]++;
    }
    if (s_client) {
        usb_transfer_t *x = &fx->xfer;
        if (xQueueSend(s_client->done_q, &x, 0) != pdTRUE) {
            ESP_LOGE(TAG, "done queue full");
        }
        xSemaphoreGive(s_client->event_sem);
    }
}

static int out_data_in_flight_locked(void)
{
    int n = 0;
    for (fake_xfer_t *fx = s_pending; fx; fx = fx->next) {
        if (fx->is_out_data) n++;
    }
    return n;
}

/** Endpoint addresses of a claimed interface's alt setting; returns the count. */
static int iface_eps_locked(uint8_t iface_num, uint8_t *eps, int max)
{
    if (!s_dev || iface_num >= MAX_IFACES || !s_dev->claimed[iface_num]) return 0;
    const usb_config_desc_t *cfg = (const usb_config_desc_t *)s_dev->cfg;
    int off = 0;
    const usb_intf_desc_t *intf = usb_parse_interface_descriptor(cfg, iface_num, s_dev->claimed_alt[iface_num], &off);
    int n = 0;
    for (int e = 0; intf && e < intf->bNumEndpoints && n < max; e++) {
        int eoff = off;
        const usb_ep_desc_t *ep = usb_parse_endpoint_descriptor_by_index(intf, e, cfg->wTotalLength, &eoff);
        if (ep) eps[n++] = ep->bEndpointAddress;
    }
    return n;
}

/**
 * As in usb_host.c, an endpoint stays busy until its completions' callbacks
 * have returned, not just until the hardware finished the transfer.
 */
static bool iface_has_in_flight_locked(uint8_t iface_num)
{
    uint8_t eps[8];
    int n = iface_eps_locked(iface_num, eps, 8);
    for (int e = 0; e < n; e++) {
        if (s_dev->ep_busy[ep_index(eps[e])] > 0) return true;
    }
    for (fake_xfer_t *fx = s_pending; fx; fx = fx->next) {
        for (int e = 0; e < n; e++) {
            if (eps[e] == fx->xfer.bEndpointAddress) return true;
        }
    }
    return false;
}

/**
 * Transfers on the interface's endpoints that are submitted or completed but
 * not yet handed to their callback: what espressif/usb's release checks
 * (pending_ep_tailq) before sleeping. A callback already running is excluded.
 */
static bool iface_has_unprocessed_locked(uint8_t iface_num)
{
    uint8_t eps[8];
    int n = iface_eps_locked(iface_num, eps, 8);
    for (fake_xfer_t *fx = s_all_xfers; fx; fx = fx->all_next) {
        if (!(fx->in_flight || fx->queued) || fx->freed_while_queued) continue;
        for (int e = 0; e < n; e++) {
            if (eps[e] == fx->xfer.bEndpointAddress) return true;
        }
    }
    return false;
}

/** Endpoint descriptor for ep_addr in a claimed alt setting, or NULL. */
static const usb_ep_desc_t *claimed_ep_locked(uint8_t ep_addr)
{
    const usb_config_desc_t *cfg = (const usb_config_desc_t *)s_dev->cfg;
    for (int i = 0; i < MAX_IFACES; i++) {
        if (!s_dev->claimed[i]) continue;
        int off = 0;
        const usb_intf_desc_t *intf = usb_parse_interface_descriptor(cfg, i, s_dev->claimed_alt[i], &off);
        if (!intf) continue;
        for (int e = 0; e < intf->bNumEndpoints; e++) {
            int eoff = off;
            const usb_ep_desc_t *ep = usb_parse_endpoint_descriptor_by_index(intf, e, cfg->wTotalLength, &eoff);
            if (ep && ep->bEndpointAddress == ep_addr) return ep;
        }
    }
    return NULL;
}

static void free_device_locked(void)
{
    free(s_dev->cfg);
    free(s_dev);
    s_dev = NULL;
    s_stats.device_freed = true;
}

// ── Bus ──────────────────────────────────────────────────────────

static void complete_iso_locked(fake_xfer_t *fx)
{
    usb_transfer_t *x = &fx->xfer;
    int total = 0;
    if (fx->is_feedback) {
        bool fail = s_fb_status_count > 0;
        if (fail) s_fb_status_count--;
        usb_isoc_packet_desc_t *p = &x->isoc_packet_desc[0];
        if (fail) {
            p->status = s_fb_status;
            p->actual_num_bytes = 0;
        } else {
            int n = p->num_bytes < 4 ? p->num_bytes : 4;
            for (int i = 0; i < n; i++) {
                x->data_buffer[i] = (uint8_t)(s_feedback_q16 >> (8 * i));
            }
            p->status = USB_TRANSFER_STATUS_COMPLETED;
            p->actual_num_bytes = n;
            total = n;
        }
    } else if (fx->is_out_data) {
        bool fail = s_out_fail_count > 0;
        if (fail) s_out_fail_count--;
        for (int i = 0; i < x->num_isoc_packets; i++) {
            usb_isoc_packet_desc_t *p = &x->isoc_packet_desc[i];
            if (fail) {
                p->status = s_out_fail_status;
                p->actual_num_bytes = 0;
                continue;
            }
            p->status = USB_TRANSFER_STATUS_COMPLETED;
            p->actual_num_bytes = p->num_bytes;
            // OUT packets are contiguous in the buffer (hcd_dwc.c _buffer_fill_isoc)
            const uint8_t *data = x->data_buffer + total;
            for (int k = 0; k < p->num_bytes; k++) {
                uint8_t b = data[k];
                if (b == 0) {
                    if (s_out_prev != 0) s_stats.out_silence_after_data++;
                    continue;
                }
                if (s_out_prev != 0 && b != (s_out_prev == 255 ? 1 : s_out_prev + 1)) {
                    s_stats.out_pattern_breaks++;
                }
                s_out_prev = b;
                s_stats.out_nonzero_bytes++;
            }
            total += p->num_bytes;
            s_stats.out_packets++;
            s_stats.out_bytes += (uint32_t)p->num_bytes;
            if ((uint32_t)p->num_bytes < s_stats.out_min_packet) s_stats.out_min_packet = p->num_bytes;
            if ((uint32_t)p->num_bytes > s_stats.out_max_packet) s_stats.out_max_packet = p->num_bytes;
            if (s_stats.first_out_count < 32) {
                s_stats.first_out_packets[s_stats.first_out_count++] = (uint16_t)p->num_bytes;
            }
        }
    } else {
        // Capture: 288 bytes (48 kHz 24-bit stereo) per packet, filled with a
        // running byte counter so tests can check order and continuity. As in
        // the HCD (_buffer_fill_isoc), each packet starts after the previous
        // packets' requested num_bytes, not their actual length.
        int offset = 0;
        for (int i = 0; i < x->num_isoc_packets; i++) {
            usb_isoc_packet_desc_t *p = &x->isoc_packet_desc[i];
            int n = p->num_bytes < 288 ? p->num_bytes : 288;
            for (int k = 0; k < n; k++) {
                x->data_buffer[offset + k] = s_capture_next++;
            }
            p->status = USB_TRANSFER_STATUS_COMPLETED;
            p->actual_num_bytes = n;
            offset += p->num_bytes;
            total += n;
        }
    }
    x->actual_num_bytes = total;
    x->status = USB_TRANSFER_STATUS_COMPLETED;
    complete_locked(fx);
}

static void bus_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, 1);
        lock();
        s_frame++;
        if (s_dev && s_dev->ctrl_retire_pending) {
            s_dev->ctrl_in_flight -= s_dev->ctrl_retire_pending;
            s_dev->ctrl_retire_pending = 0;
        }
        fake_xfer_t *fx = s_pending;
        while (fx) {
            fake_xfer_t *next = fx->next;
            bool halted = s_dev && s_dev->halted[ep_index(fx->xfer.bEndpointAddress)];
            if (!fx->hung && fx->xfer.bEndpointAddress != 0 && !halted && fx->due_frame <= s_frame) {
                complete_iso_locked(fx);
            }
            fx = next;
        }
        unlock();
    }
}

// ── Test control ─────────────────────────────────────────────────

void fake_usb_reset(void)
{
    if (!s_lock) {
        // A plain binary semaphore, not a mutex: the fake's lock is contended by
        // tasks from priority 1 to 23, and priority inheritance through it on
        // the FreeRTOS POSIX port produced port-level asserts and stalls.
        s_lock = xSemaphoreCreateBinary();
        xSemaphoreGive(s_lock);
        xTaskCreate(bus_task, "fake_bus", 4096, NULL, configMAX_PRIORITIES - 2, &s_bus_task);
    }
    lock();
    if (s_client) {
        ESP_LOGE(TAG, "reset with a client still registered");
    }
    while (s_pending) {
        pending_remove(s_pending);
    }
    if (s_dev) {
        free(s_dev->cfg);
        free(s_dev);
        s_dev = NULL;
    }
    int allocated = s_stats.transfers_allocated;
    memset(&s_stats, 0, sizeof(s_stats));
    s_stats.transfers_allocated = allocated;   // leaks carry over and stay visible
    s_stats.out_min_packet = UINT32_MAX;
    s_feedback_q16 = 0x00300000;
    s_fb_status_count = 0;
    s_out_fail_count = 0;
    s_release_busy = 0;
    s_hang.armed = false;
    s_max_iso_out_mps = 0;
    s_stall.armed = false;
    s_alloc_fail_after = -1;
    s_out_prev = 0;
    s_clock_rates[0] = 44100;
    s_clock_rates[1] = 48000;
    s_clock_rate_count = 2;
    unlock();
}

void fake_usb_attach(const uint8_t *config_desc, uint16_t len)
{
    if (!config_desc) {
        config_desc = minidsp_2x4hd_config1_desc;
        len = sizeof(minidsp_2x4hd_config1_desc);
    }
    lock();
    if (s_dev) {
        ESP_LOGE(TAG, "attach while a device is still allocated");
        unlock();
        return;
    }
    s_dev = calloc(1, sizeof(*s_dev));
    s_dev->cfg = malloc(len);
    memcpy(s_dev->cfg, config_desc, len);
    s_dev->cfg_len = len;
    s_dev->addr = s_next_addr++;
    s_dev->sample_rate = 48000;
    s_stats.device_freed = false;
    if (s_client) {
        usb_host_client_event_msg_t msg = {.event = USB_HOST_CLIENT_EVENT_NEW_DEV, .new_dev.address = s_dev->addr};
        xQueueSend(s_client->event_q, &msg, 0);
        xSemaphoreGive(s_client->event_sem);
    }
    unlock();
}

void fake_usb_detach(void)
{
    lock();
    if (!s_dev || s_dev->gone) {
        unlock();
        return;
    }
    s_dev->gone = true;
    s_dev->ctrl_in_flight -= s_dev->ctrl_retire_pending;
    s_dev->ctrl_retire_pending = 0;
    // Every pending transfer of the device retires with NO_DEVICE
    while (s_pending) {
        fake_xfer_t *fx = s_pending;
        if (fx->xfer.bEndpointAddress == 0) {
            s_dev->ctrl_in_flight--;
        }
        fx->xfer.status = USB_TRANSFER_STATUS_NO_DEVICE;
        fx->xfer.actual_num_bytes = 0;
        complete_locked(fx);
    }
    if (s_dev->open_count > 0 && s_client) {
        usb_host_client_event_msg_t msg = {.event = USB_HOST_CLIENT_EVENT_DEV_GONE,
                                           .dev_gone.dev_hdl = (usb_device_handle_t)s_dev};
        xQueueSend(s_client->event_q, &msg, 0);
        xSemaphoreGive(s_client->event_sem);
    } else if (s_dev->open_count == 0) {
        free_device_locked();
    }
    unlock();
}

void fake_usb_set_feedback_q16(uint32_t q16) { lock(); s_feedback_q16 = q16; unlock(); }

void fake_usb_set_feedback_packet_status(usb_transfer_status_t status, int count)
{
    lock(); s_fb_status = status; s_fb_status_count = count; unlock();
}

void fake_usb_fail_out_urbs(usb_transfer_status_t packet_status, int count)
{
    lock(); s_out_fail_status = packet_status; s_out_fail_count = count; unlock();
}

void fake_usb_hang_ctrl(uint8_t entity, uint8_t control_selector, bool get)
{
    lock();
    s_hang.armed = true;
    s_hang.entity = entity;
    s_hang.cs = control_selector;
    s_hang.get = get;
    unlock();
}

void fake_usb_set_release_busy(int count) { lock(); s_release_busy = count; unlock(); }

void fake_usb_set_max_iso_out_mps(uint16_t limit) { lock(); s_max_iso_out_mps = limit; unlock(); }

void fake_usb_fail_transfer_alloc_after(int n) { lock(); s_alloc_fail_after = n; unlock(); }

void fake_usb_set_clock_rates(const uint32_t *rates, int count)
{
    lock();
    if (count <= 0) {
        s_clock_rates[0] = 44100;
        s_clock_rates[1] = 48000;
        s_clock_rate_count = 2;
    } else {
        s_clock_rate_count = count < MAX_CLOCK_RATES ? count : MAX_CLOCK_RATES;
        memcpy(s_clock_rates, rates, s_clock_rate_count * sizeof(uint32_t));
    }
    unlock();
}

void fake_usb_stall_ctrl(uint8_t entity, uint8_t control_selector, uint8_t channel, bool get)
{
    lock();
    s_stall.armed = true;
    s_stall.entity = entity;
    s_stall.cs = control_selector;
    s_stall.ch = channel;
    s_stall.get = get;
    unlock();
}

uint16_t fake_usb_no_feedback_config(uint8_t sync_type, uint8_t *out, uint16_t out_size)
{
    const uint8_t *in = minidsp_2x4hd_config1_desc;
    uint16_t in_len = sizeof(minidsp_2x4hd_config1_desc);
    uint16_t len = 0;
    int last_intf = -1;
    for (uint16_t off = 0; off < in_len && in[off] >= 2; off += in[off]) {
        const uint8_t *d = in + off;
        if (d[1] == USB_B_DESCRIPTOR_TYPE_ENDPOINT && (d[3] & 0x30) == 0x10) {
            // Feedback endpoint (usage type 01): drop it from its interface
            if (last_intf >= 0) out[last_intf + 4]--;
            continue;
        }
        if (len + d[0] > out_size) return 0;
        memcpy(out + len, d, d[0]);
        if (d[1] == USB_B_DESCRIPTOR_TYPE_INTERFACE) {
            last_intf = len;
        } else if (d[1] == USB_B_DESCRIPTOR_TYPE_ENDPOINT && (d[2] & 0x80) == 0 && (d[3] & 0x03) == 0x01) {
            out[len + 3] = (uint8_t)((d[3] & ~0x0C) | ((sync_type & 0x03) << 2));
        }
        len += d[0];
    }
    out[2] = len & 0xFF;
    out[3] = len >> 8;
    return len;
}

fake_usb_stats_t fake_usb_stats(void)
{
    lock();
    fake_usb_stats_t st = s_stats;
    st.client_registered = s_client != NULL;
    st.device_open_count = s_dev ? s_dev->open_count : 0;
    st.interfaces_claimed = 0;
    if (s_dev) {
        for (int i = 0; i < MAX_IFACES; i++) st.interfaces_claimed += s_dev->claimed[i];
    }
    unlock();
    return st;
}

void fake_usb_clear_out_stats(void)
{
    lock();
    s_stats.out_packets = 0;
    s_stats.out_bytes = 0;
    s_stats.out_min_packet = UINT32_MAX;
    s_stats.out_max_packet = 0;
    s_stats.first_out_count = 0;
    s_stats.out_nonzero_bytes = 0;
    s_stats.out_pattern_breaks = 0;
    s_stats.out_silence_after_data = 0;
    s_out_prev = 0;
    unlock();
}

void fake_usb_debug_dump(void)
{
    TaskHandle_t holder = s_lock_holder;
    printf("  fake lock holder: %s\n", holder ? pcTaskGetName(holder) : "none");
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(200)) != pdTRUE) {
        printf("  fake: lock not available, pending list not dumped\n");
        return;
    }
    s_lock_holder = xTaskGetCurrentTaskHandle();
    int n = 0;
    for (fake_xfer_t *fx = s_pending; fx && n < 64; fx = fx->next, n++) {
        printf("  pending ep 0x%02X in_flight %d hung %d due %llu (frame %llu)\n",
               fx->xfer.bEndpointAddress, fx->in_flight, fx->hung,
               (unsigned long long)fx->due_frame, (unsigned long long)s_frame);
    }
    if (n == 64) printf("  pending list has 64+ entries (cycle?)\n");
    printf("  done queue %u, client %p, dev %p ctrl_in_flight %d\n",
           s_client ? (unsigned)uxQueueMessagesWaiting(s_client->done_q) : 0, (void *)s_client,
           (void *)s_dev, s_dev ? s_dev->ctrl_in_flight : -1);
    unlock();
}

// ── Control requests (the simulated miniDSP) ─────────────────────

/** Fill the response or apply a SET. Returns the data length, or -1 to STALL. */
static int handle_ctrl_locked(const usb_setup_packet_t *setup, uint8_t *data)
{
    bool get = setup->bmRequestType & 0x80;
    // Standard SET_INTERFACE
    if (setup->bmRequestType == 0x01 && setup->bRequest == 0x0B) {
        s_stats.set_interface_calls++;
        if (setup->wValue != 0) {
            if (!s_dev->rate_set_since_alt) s_stats.alt_enabled_without_rate++;
            s_dev->rate_set_since_alt = false;
        }
        return 0;
    }
    if ((setup->bmRequestType & 0x60) != 0x20) {
        return -1;
    }
    uint8_t entity = setup->wIndex >> 8;
    uint8_t cs = setup->wValue >> 8;
    uint8_t ch = setup->wValue & 0xFF;
    bool cur = setup->bRequest == 0x01;
    bool range = setup->bRequest == 0x02;
    if (s_stall.armed && entity == s_stall.entity && cs == s_stall.cs && ch == s_stall.ch &&
        get == s_stall.get) {
        return -1;
    }

    if (entity == ENT_CLOCK_SOURCE && cs == 0x01) {
        if (get && cur) {
            memcpy(data, &s_dev->sample_rate, 4);
            return 4;
        }
        if (get && range) {
            // Discrete triplets (min, max, res = 0); the transfer gets the
            // first wLength bytes, so a long list arrives truncated as on a device
            data[0] = (uint8_t)s_clock_rate_count;
            data[1] = 0;
            for (int i = 0; i < s_clock_rate_count; i++) {
                const uint32_t triplet[3] = {s_clock_rates[i], s_clock_rates[i], 0};
                memcpy(data + 2 + i * 12, triplet, 12);
            }
            return 2 + s_clock_rate_count * 12;
        }
        if (!get && cur) {
            uint32_t rate;
            memcpy(&rate, data, 4);
            s_stats.set_rate_calls++;
            s_stats.last_set_rate = rate;
            if (rate != s_dev->sample_rate && out_data_in_flight_locked() > 0) {
                s_stats.set_rate_while_streaming++;
            }
            s_dev->sample_rate = rate;
            s_dev->rate_set_since_alt = true;
            return 0;
        }
    }
    if (entity == ENT_CLOCK_SOURCE && cs == 0x02 && get && cur) {
        data[0] = 1;
        return 1;
    }
    if (entity == ENT_CLOCK_SELECTOR && cs == 0x01 && get && cur) {
        data[0] = 1;
        return 1;
    }
    if ((entity == ENT_FU_PLAYBACK || entity == ENT_FU_CAPTURE) && ch < 8) {
        int fu = entity == ENT_FU_CAPTURE;
        if (cs == 0x01 && cur) {
            if (get) {
                data[0] = s_dev->mute[fu][ch];
                return 1;
            }
            s_dev->mute[fu][ch] = data[0];
            return 0;
        }
        if (cs == 0x02 && cur) {
            if (get) {
                memcpy(data, &s_dev->volume[fu][ch], 2);
                return 2;
            }
            memcpy(&s_dev->volume[fu][ch], data, 2);
            return 0;
        }
        if (cs == 0x02 && range && get) {
            const int16_t r[3] = {-90 * 256, 0, 256};
            data[0] = 1;
            data[1] = 0;
            memcpy(data + 2, r, 6);
            return 8;
        }
    }
    return -1;
}

static void answer_ctrl_locked(fake_xfer_t *fx)
{
    usb_transfer_t *x = &fx->xfer;
    usb_setup_packet_t setup;
    memcpy(&setup, x->data_buffer, sizeof(setup));
    bool get = setup.bmRequestType & 0x80;
    // The device model works in a scratch buffer; only wLength bytes (already
    // checked against the transfer size at submit) reach the transfer
    uint8_t scratch[256] = {0};
    if (!get) {
        memcpy(scratch, x->data_buffer + sizeof(setup), setup.wLength < sizeof(scratch) ? setup.wLength : sizeof(scratch));
    }
    int len = handle_ctrl_locked(&setup, scratch);
    if (len < 0) {
        x->status = USB_TRANSFER_STATUS_STALL;
        x->actual_num_bytes = sizeof(setup);
    } else {
        if (get) {
            if (len > setup.wLength) len = setup.wLength;
            memcpy(x->data_buffer + sizeof(setup), scratch, len);
        }
        x->status = USB_TRANSFER_STATUS_COMPLETED;
        x->actual_num_bytes = sizeof(setup) + (get ? len : setup.wLength);
    }
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 0, 0)
    // In-tree IDF 5.x order (usbh.c handle_ep0_dequeue): the client is notified
    // first and the in-flight count drops afterwards, here on the next frame.
    // A close right after the completion therefore trips the violation check.
    s_dev->ctrl_retire_pending++;
#else
    // espressif/usb >= 1.6.0: retired before the client is notified
    s_dev->ctrl_in_flight--;
#endif
    complete_locked(fx);
}

bool fake_usb_release_hung_ctrl(void)
{
    lock();
    for (fake_xfer_t *fx = s_pending; fx; fx = fx->next) {
        if (fx->hung) {
            answer_ctrl_locked(fx);
            unlock();
            return true;
        }
    }
    unlock();
    return false;
}

// ── usb_host.h: clients ──────────────────────────────────────────

esp_err_t usb_host_client_register(const usb_host_client_config_t *client_config,
                                   usb_host_client_handle_t *client_hdl_ret)
{
    if (!client_config || !client_hdl_ret) return ESP_ERR_INVALID_ARG;
    lock();
    if (s_client) {
        unlock();
        return ESP_ERR_NOT_SUPPORTED;   // the fake supports one client
    }
    fake_client_t *c = calloc(1, sizeof(*c));
    c->cb = client_config->async.client_event_callback;
    c->arg = client_config->async.callback_arg;
    c->event_sem = xSemaphoreCreateBinary();
    c->done_q = xQueueCreate(DONE_QUEUE_LEN, sizeof(usb_transfer_t *));
    c->event_q = xQueueCreate(EVENT_QUEUE_LEN, sizeof(usb_host_client_event_msg_t));
    s_client = c;
    unlock();
    *client_hdl_ret = (usb_host_client_handle_t)c;
    return ESP_OK;
}

esp_err_t usb_host_client_deregister(usb_host_client_handle_t client_hdl)
{
    fake_client_t *c = (fake_client_t *)client_hdl;
    lock();
    if (c != s_client) {
        unlock();
        return ESP_ERR_INVALID_ARG;
    }
    // Real library: refuse with devices open, completions not yet handled, or
    // a usb_host_client_handle_events() call in progress
    if ((s_dev && s_dev->open_count > 0) || uxQueueMessagesWaiting(c->done_q) > 0 || c->handling) {
        unlock();
        return ESP_ERR_INVALID_STATE;
    }
    s_client = NULL;
    unlock();
    vSemaphoreDelete(c->event_sem);
    vQueueDelete(c->done_q);
    vQueueDelete(c->event_q);
    free(c);
    return ESP_OK;
}

esp_err_t usb_host_client_handle_events(usb_host_client_handle_t client_hdl, TickType_t timeout_ticks)
{
    fake_client_t *c = (fake_client_t *)client_hdl;
    lock();
    c->handling = true;
    unlock();
    // As in the real library: ESP_ERR_TIMEOUT only if a blocking wait saw nothing
    esp_err_t ret = timeout_ticks == 0 ? ESP_OK : ESP_ERR_TIMEOUT;
    while (xSemaphoreTake(c->event_sem, timeout_ticks) == pdTRUE) {
        usb_transfer_t *x;
        while (xQueueReceive(c->done_q, &x, 0) == pdTRUE) {
            fake_xfer_t *fx = WRAP(x);
            lock();
            if (fx->freed_while_queued) {
                // Skipping the callback still retires the endpoint's completion
                if (s_dev && x->bEndpointAddress != 0) s_dev->ep_busy[ep_index(x->bEndpointAddress)]--;
                for (fake_xfer_t **pp = &s_all_xfers; *pp; pp = &(*pp)->all_next) {
                    if (*pp == fx) {
                        *pp = fx->all_next;
                        break;
                    }
                }
                unlock();
                free(x->data_buffer);
                free(fx);
                continue;
            }
            fx->queued = false;
            // As in usb_host.c, nothing touches the transfer once its callback
            // runs (the client may free it); only the endpoint count drops after
            fake_dev_t *dev = s_dev;
            int idx = x->bEndpointAddress != 0 ? ep_index(x->bEndpointAddress) : -1;
            unlock();
            x->callback(x);
            if (idx >= 0) {
                lock();
                if (s_dev == dev && s_dev) s_dev->ep_busy[idx]--;
                unlock();
            }
        }
        usb_host_client_event_msg_t msg;
        while (xQueueReceive(c->event_q, &msg, 0) == pdTRUE) {
            c->cb(&msg, c->arg);
        }
        ret = ESP_OK;
        timeout_ticks = 0;
    }
    lock();
    c->handling = false;
    unlock();
    return ret;
}

esp_err_t usb_host_client_unblock(usb_host_client_handle_t client_hdl)
{
    fake_client_t *c = (fake_client_t *)client_hdl;
    xSemaphoreGive(c->event_sem);
    return ESP_OK;
}

// ── usb_host.h: devices ──────────────────────────────────────────

static fake_dev_t *check_dev_locked(usb_device_handle_t dev_hdl)
{
    return (s_dev && (usb_device_handle_t)s_dev == dev_hdl) ? s_dev : NULL;
}

esp_err_t usb_host_device_addr_list_fill(int list_len, uint8_t *dev_addr_list, int *num_dev_ret)
{
    lock();
    int n = 0;
    if (s_dev && !s_dev->gone && list_len > 0) {
        dev_addr_list[n++] = s_dev->addr;
    }
    unlock();
    *num_dev_ret = n;
    return ESP_OK;
}

esp_err_t usb_host_device_open(usb_host_client_handle_t client_hdl, uint8_t dev_addr,
                               usb_device_handle_t *dev_hdl_ret)
{
    (void)client_hdl;
    lock();
    if (!s_dev || s_dev->gone || s_dev->addr != dev_addr) {
        unlock();
        return ESP_ERR_NOT_FOUND;
    }
    if (s_dev->open_count > 0) {
        unlock();
        return ESP_ERR_INVALID_STATE;   // a client cannot open a device twice
    }
    s_dev->open_count++;
    *dev_hdl_ret = (usb_device_handle_t)s_dev;
    unlock();
    return ESP_OK;
}

esp_err_t usb_host_device_close(usb_host_client_handle_t client_hdl, usb_device_handle_t dev_hdl)
{
    (void)client_hdl;
    lock();
    fake_dev_t *d = check_dev_locked(dev_hdl);
    if (!d || d->open_count == 0) {
        unlock();
        return ESP_ERR_INVALID_STATE;
    }
    for (int i = 0; i < MAX_IFACES; i++) {
        if (d->claimed[i]) {
            unlock();
            return ESP_ERR_INVALID_STATE;
        }
    }
    d->open_count--;
    if (d->open_count == 0) {
        if (d->ctrl_in_flight > 0) {
            // usbh_dev_close() asserts here in ESP-IDF
            ESP_LOGE(TAG, "VIOLATION: last close with %d control transfer(s) in flight", d->ctrl_in_flight);
            s_stats.close_with_ctrl_in_flight++;
        }
        if (d->gone) {
            free_device_locked();
        }
    }
    unlock();
    return ESP_OK;
}

esp_err_t usb_host_device_info(usb_device_handle_t dev_hdl, usb_device_info_t *dev_info)
{
    lock();
    fake_dev_t *d = check_dev_locked(dev_hdl);
    if (!d) {
        unlock();
        return ESP_ERR_INVALID_ARG;
    }
    memset(dev_info, 0, sizeof(*dev_info));
    dev_info->speed = USB_SPEED_FULL;
    dev_info->dev_addr = d->addr;
    dev_info->bMaxPacketSize0 = 64;
    dev_info->bConfigurationValue = 1;
    dev_info->str_desc_product = (const usb_str_desc_t *)s_product_str;
    unlock();
    return ESP_OK;
}

esp_err_t usb_host_get_device_descriptor(usb_device_handle_t dev_hdl, const usb_device_desc_t **device_desc)
{
    lock();
    fake_dev_t *d = check_dev_locked(dev_hdl);
    unlock();
    if (!d) return ESP_ERR_INVALID_ARG;
    *device_desc = (const usb_device_desc_t *)minidsp_2x4hd_device_desc;
    return ESP_OK;
}

esp_err_t usb_host_get_active_config_descriptor(usb_device_handle_t dev_hdl, const usb_config_desc_t **config_desc)
{
    lock();
    fake_dev_t *d = check_dev_locked(dev_hdl);
    unlock();
    if (!d) return ESP_ERR_INVALID_ARG;
    *config_desc = (const usb_config_desc_t *)d->cfg;
    return ESP_OK;
}

// ── usb_host.h: interfaces and endpoints ─────────────────────────

esp_err_t usb_host_interface_claim(usb_host_client_handle_t client_hdl, usb_device_handle_t dev_hdl,
                                   uint8_t bInterfaceNumber, uint8_t bAlternateSetting)
{
    (void)client_hdl;
    lock();
    fake_dev_t *d = check_dev_locked(dev_hdl);
    esp_err_t ret = ESP_OK;
    int off = 0;
    if (!d || d->open_count == 0 || bInterfaceNumber >= MAX_IFACES) {
        ret = ESP_ERR_INVALID_ARG;
    } else if (d->gone) {
        ret = ESP_ERR_INVALID_STATE;
    } else if (d->claimed[bInterfaceNumber]) {
        ret = ESP_ERR_INVALID_STATE;
    } else {
        const usb_config_desc_t *cfg = (const usb_config_desc_t *)d->cfg;
        const usb_intf_desc_t *intf = usb_parse_interface_descriptor(cfg, bInterfaceNumber,
                                                                     bAlternateSetting, &off);
        if (!intf) {
            ret = ESP_ERR_NOT_FOUND;
        } else {
            // The HCD refuses an endpoint whose MPS exceeds the FIFO it was given
            for (int i = 0; s_max_iso_out_mps && i < intf->bNumEndpoints; i++) {
                int ep_off = off;
                const usb_ep_desc_t *ep = usb_parse_endpoint_descriptor_by_index(
                    intf, i, cfg->wTotalLength, &ep_off);
                if (ep && (ep->bEndpointAddress & 0x80) == 0 &&
                    (ep->bmAttributes & 0x03) == USB_BM_ATTRIBUTES_XFER_ISOC &&
                    USB_EP_DESC_GET_MPS(ep) > s_max_iso_out_mps) {
                    ret = ESP_ERR_NOT_SUPPORTED;
                }
            }
            if (ret == ESP_OK) {
                d->claimed[bInterfaceNumber] = true;
                d->claimed_alt[bInterfaceNumber] = bAlternateSetting;
            }
        }
    }
    unlock();
    return ret;
}

esp_err_t usb_host_interface_release(usb_host_client_handle_t client_hdl, usb_device_handle_t dev_hdl,
                                     uint8_t bInterfaceNumber)
{
    (void)client_hdl;
    lock();
    fake_dev_t *d = check_dev_locked(dev_hdl);
    esp_err_t ret = ESP_OK;
    if (d && bInterfaceNumber < MAX_IFACES && d->claimed[bInterfaceNumber] &&
        iface_has_unprocessed_locked(bInterfaceNumber)) {
        s_stats.release_with_pending++;
    }
    if (!d || bInterfaceNumber >= MAX_IFACES || !d->claimed[bInterfaceNumber]) {
        ret = ESP_ERR_INVALID_ARG;
    } else if (s_release_busy > 0) {
        s_release_busy--;
        ret = ESP_ERR_INVALID_STATE;     // ESP-IDF #17707
    } else if (iface_has_in_flight_locked(bInterfaceNumber)) {
        ret = ESP_ERR_INVALID_STATE;     // endpoints still have transfers
    } else {
        uint8_t eps[8];
        int n = iface_eps_locked(bInterfaceNumber, eps, 8);
        for (int e = 0; e < n; e++) {
            d->halted[ep_index(eps[e])] = false;
            d->ep_next_frame[ep_index(eps[e])] = 0;
        }
        d->claimed[bInterfaceNumber] = false;
    }
    unlock();
    return ret;
}

esp_err_t usb_host_endpoint_halt(usb_device_handle_t dev_hdl, uint8_t bEndpointAddress)
{
    if ((bEndpointAddress & 0x0F) == 0) return ESP_ERR_INVALID_ARG;   // EP0 is owned by USBH
    lock();
    fake_dev_t *d = check_dev_locked(dev_hdl);
    esp_err_t ret = ESP_OK;
    if (!d) {
        ret = ESP_ERR_INVALID_ARG;
    } else if (!d->gone && !claimed_ep_locked(bEndpointAddress)) {
        ret = ESP_ERR_NOT_FOUND;
    } else {
        d->halted[ep_index(bEndpointAddress)] = true;
    }
    unlock();
    return ret;
}

esp_err_t usb_host_endpoint_flush(usb_device_handle_t dev_hdl, uint8_t bEndpointAddress)
{
    if ((bEndpointAddress & 0x0F) == 0) return ESP_ERR_INVALID_ARG;
    lock();
    fake_dev_t *d = check_dev_locked(dev_hdl);
    if (!d) {
        unlock();
        return ESP_ERR_INVALID_ARG;
    }
    if (!d->gone && !d->halted[ep_index(bEndpointAddress)]) {
        unlock();
        return ESP_ERR_INVALID_STATE;    // the HCD only flushes a halted pipe
    }
    fake_xfer_t *fx = s_pending;
    while (fx) {
        fake_xfer_t *next = fx->next;
        if (fx->xfer.bEndpointAddress == bEndpointAddress) {
            fx->xfer.status = USB_TRANSFER_STATUS_CANCELED;
            fx->xfer.actual_num_bytes = 0;
            complete_locked(fx);
        }
        fx = next;
    }
    unlock();
    return ESP_OK;
}

esp_err_t usb_host_endpoint_clear(usb_device_handle_t dev_hdl, uint8_t bEndpointAddress)
{
    if ((bEndpointAddress & 0x0F) == 0) return ESP_ERR_INVALID_ARG;
    lock();
    fake_dev_t *d = check_dev_locked(dev_hdl);
    esp_err_t ret = ESP_OK;
    if (!d) {
        ret = ESP_ERR_INVALID_ARG;
    } else if (!claimed_ep_locked(bEndpointAddress)) {
        ret = ESP_ERR_NOT_FOUND;
    } else if (d->gone || !d->halted[ep_index(bEndpointAddress)]) {
        ret = ESP_ERR_INVALID_STATE;   // the HCD clears only a halted pipe on a live port
    } else {
        d->halted[ep_index(bEndpointAddress)] = false;
        d->ep_next_frame[ep_index(bEndpointAddress)] = 0;
    }
    unlock();
    return ret;
}

// ── usb_host.h: transfers ────────────────────────────────────────

esp_err_t usb_host_transfer_alloc(size_t data_buffer_size, int num_isoc_packets, usb_transfer_t **transfer)
{
    lock();
    bool fail = s_alloc_fail_after == 0;
    if (s_alloc_fail_after >= 0) s_alloc_fail_after--;
    unlock();
    if (fail) return ESP_ERR_NO_MEM;
    fake_xfer_t *fx = calloc(1, sizeof(fake_xfer_t) + num_isoc_packets * sizeof(usb_isoc_packet_desc_t));
    uint8_t *buf = calloc(1, data_buffer_size ? data_buffer_size : 1);
    if (!fx || !buf) {
        free(fx);
        free(buf);
        return ESP_ERR_NO_MEM;
    }
    *(uint8_t **)&fx->xfer.data_buffer = buf;
    *(size_t *)&fx->xfer.data_buffer_size = data_buffer_size;
    *(int *)&fx->xfer.num_isoc_packets = num_isoc_packets;
    lock();
    s_stats.transfers_allocated++;
    fx->all_next = s_all_xfers;
    s_all_xfers = fx;
    unlock();
    *transfer = &fx->xfer;
    return ESP_OK;
}

esp_err_t usb_host_transfer_free(usb_transfer_t *transfer)
{
    if (!transfer) return ESP_OK;
    fake_xfer_t *fx = WRAP(transfer);
    lock();
    if (fx->in_flight || fx->queued) {
        // The real library frees it anyway; the HCD or the client's pending
        // callback later touches freed memory
        ESP_LOGE(TAG, "VIOLATION: freeing a transfer still %s (EP 0x%02X)",
                 fx->in_flight ? "in flight" : "awaiting its callback", transfer->bEndpointAddress);
        s_stats.free_while_in_flight++;
        pending_remove(fx);
        if (fx->queued) {
            // Its pointer is still in the done queue: release it from there
            fx->freed_while_queued = true;
            s_stats.transfers_allocated--;
            unlock();
            return ESP_OK;
        }
    }
    for (fake_xfer_t **pp = &s_all_xfers; *pp; pp = &(*pp)->all_next) {
        if (*pp == fx) {
            *pp = fx->all_next;
            break;
        }
    }
    s_stats.transfers_allocated--;
    unlock();
    free(transfer->data_buffer);
    free(fx);
    return ESP_OK;
}

/** usbh.c transfer_check_usb_compliance() and the HCD's ISOC list limit. */
static bool iso_transfer_valid_locked(const usb_transfer_t *x, const usb_ep_desc_t *ep)
{
    int total = 0;
    bool is_in = ep->bEndpointAddress & 0x80;
    for (int i = 0; i < x->num_isoc_packets; i++) {
        total += x->isoc_packet_desc[i].num_bytes;
        if (is_in && x->isoc_packet_desc[i].num_bytes % ep->wMaxPacketSize != 0) {
            ESP_LOGE(TAG, "INVALID: ISOC IN packet not a multiple of MPS (EP 0x%02X)", ep->bEndpointAddress);
            s_stats.invalid_transfers++;
            return false;
        }
    }
    if (total != x->num_bytes) {
        ESP_LOGE(TAG, "INVALID: ISOC num_bytes %d != packet total %d (EP 0x%02X)", x->num_bytes, total, ep->bEndpointAddress);
        s_stats.invalid_transfers++;
        return false;
    }
    // IDF 5.4 accepts up to 64 frames at enqueue but asserts above 61 when filling
    if (x->num_isoc_packets * (1 << (ep->bInterval - 1)) > 61) {
        ESP_LOGE(TAG, "INVALID: ISOC transfer spans more than 61 frames (EP 0x%02X)", ep->bEndpointAddress);
        s_stats.invalid_transfers++;
        return false;
    }
    return true;
}

esp_err_t usb_host_transfer_submit(usb_transfer_t *transfer)
{
    fake_xfer_t *fx = WRAP(transfer);
    lock();
    fake_dev_t *d = check_dev_locked(transfer->device_handle);
    esp_err_t ret = ESP_OK;
    const usb_ep_desc_t *ep = NULL;
    if (!d || fx->in_flight || fx->queued) {
        ret = ESP_ERR_INVALID_ARG;
    } else if (d->gone) {
        ret = ESP_ERR_INVALID_STATE;
    } else if (!(ep = claimed_ep_locked(transfer->bEndpointAddress))) {
        ret = ESP_ERR_NOT_FOUND;
    } else if (d->halted[ep_index(transfer->bEndpointAddress)]) {
        ret = ESP_ERR_INVALID_STATE;
    } else if (transfer->num_isoc_packets <= 0 ||
               transfer->num_bytes > (int)transfer->data_buffer_size) {
        s_stats.invalid_transfers++;
        ret = ESP_ERR_INVALID_SIZE;
    } else if (!iso_transfer_valid_locked(transfer, ep)) {
        ret = ESP_ERR_INVALID_ARG;
    } else {
        int interval = 1 << (ep->bInterval - 1);
        fx->is_feedback = (ep->bEndpointAddress & 0x80) && ((ep->bmAttributes >> 4) & 0x03) == 0x01;
        fx->is_out_data = !(ep->bEndpointAddress & 0x80);
        fx->hung = false;
        uint64_t *next = &d->ep_next_frame[ep_index(ep->bEndpointAddress)];
        uint64_t start = *next > s_frame ? *next : s_frame;
        fx->due_frame = start + (uint64_t)transfer->num_isoc_packets * interval;
        *next = fx->due_frame;
        fx->in_flight = true;
        pending_append(fx);
    }
    if (ret != ESP_OK) s_stats.submit_rejected++;
    unlock();
    return ret;
}

esp_err_t usb_host_transfer_submit_control(usb_host_client_handle_t client_hdl, usb_transfer_t *transfer)
{
    (void)client_hdl;
    fake_xfer_t *fx = WRAP(transfer);
    lock();
    fake_dev_t *d = check_dev_locked(transfer->device_handle);
    if (!d || fx->in_flight || fx->queued || (transfer->bEndpointAddress & 0x7F) != 0 ||
        transfer->num_bytes < (int)sizeof(usb_setup_packet_t) ||
        transfer->num_bytes > (int)transfer->data_buffer_size) {
        s_stats.submit_rejected++;
        s_stats.invalid_transfers++;
        unlock();
        return ESP_ERR_INVALID_ARG;
    }
    {
        // usbh.c transfer_check_usb_compliance() for control transfers
        usb_setup_packet_t chk;
        memcpy(&chk, transfer->data_buffer, sizeof(chk));
        int need = (int)sizeof(chk) + chk.wLength;
        bool in = chk.bmRequestType & 0x80;
        if (in ? transfer->num_bytes < need : transfer->num_bytes != need) {
            ESP_LOGE(TAG, "INVALID: control num_bytes %d vs wLength %d", transfer->num_bytes, chk.wLength);
            s_stats.submit_rejected++;
            s_stats.invalid_transfers++;
            unlock();
            return ESP_ERR_INVALID_ARG;
        }
    }
    if (d->gone) {
        s_stats.submit_rejected++;
        unlock();
        return ESP_ERR_INVALID_STATE;
    }
    s_stats.ctrl_requests++;
    d->ctrl_in_flight++;
    fx->in_flight = true;
    fx->is_feedback = false;
    fx->is_out_data = false;
    pending_append(fx);

    usb_setup_packet_t setup;
    memcpy(&setup, transfer->data_buffer, sizeof(setup));
    bool get = setup.bmRequestType & 0x80;
    if (s_hang.armed && (setup.bmRequestType & 0x60) == 0x20 &&
        (setup.wIndex >> 8) == s_hang.entity && (setup.wValue >> 8) == s_hang.cs && get == s_hang.get) {
        s_hang.armed = false;
        fx->hung = true;         // answered by fake_usb_release_hung_ctrl() or detach
    } else {
        answer_ctrl_locked(fx);
    }
    unlock();
    return ESP_OK;
}
