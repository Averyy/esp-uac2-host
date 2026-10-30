/*
 * SPDX-License-Identifier: MIT
 *
 * Runs the UAC2 driver (components/usb_host_uac2, unmodified) on the ESP-IDF
 * Linux target against the fake USB Host Library in components/usb. Covers
 * paths that real hardware cannot trigger on demand: control timeouts and
 * orphaned URBs, unplug at chosen moments, dead endpoints, feedback drift,
 * ESP-IDF #17707 release failures, concurrency and driver lifecycle.
 *
 * Every test ends by checking the fake for leaked transfers, open devices,
 * claimed interfaces and USB Host contract violations.
 */

#include <setjmp.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "unity.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "usb/uac2_host.h"
#include "fake_usb.h"
#include "minidsp_2x4hd_descriptors.h"

#define TX_IFACE        1
#define RX_IFACE        2
#define CLOCK_SOURCE    41
#define CS_CLOCK_VALID  0x02

typedef struct {
    uint8_t addr;
    uint8_t iface;
    uac2_host_driver_event_t event;
} connect_evt_t;

static QueueHandle_t s_connects;
static volatile int64_t s_test_started_us;
static const char *volatile s_test_name;
static const char *volatile s_step;     // what the test task is blocked in, for the watchdog
static volatile uint32_t s_writer_loops;
static volatile int s_writer_last_err;

typedef struct writer writer_t;
static writer_t *volatile s_active_writer;   // stopped by tearDown if a test failed mid-stream
typedef struct hammer hammer_t;
static hammer_t *volatile s_active_hammer;   // likewise
static atomic_int s_dev_events[5];
static uac2_host_device_handle_t s_open[4];

static const uac2_host_stream_config_t k_tx_48k = {.channels = 2, .bit_resolution = 24, .sample_freq = 48000};
static const uac2_host_stream_config_t k_rx_48k = {.channels = 2, .bit_resolution = 24, .sample_freq = 48000};

// ── Helpers ──────────────────────────────────────────────────────

static void driver_cb(uint8_t addr, uint8_t iface_num, const uac2_host_driver_event_t event, void *arg)
{
    (void)arg;
    connect_evt_t e = {addr, iface_num, event};
    xQueueSend(s_connects, &e, 0);
}

// Set by a test: the next TX_DONE callback blocks this long, holding a URB
// callback between its ACTIVE check and its resubmit
static atomic_int s_block_tx_done_ms;
static atomic_bool s_tx_done_blocking;

// esp_restart() runs shutdown handlers newest first. Registered before
// install, this one runs after the driver's and jumps back instead of
// restarting (no signal-mask restore, see CMakeLists.txt). Per thread: two
// tasks may restart at once.
static _Thread_local sigjmp_buf s_restart_jmp;
static void restart_jump_back(void)
{
    siglongjmp(s_restart_jmp, 1);
}

static void restart_and_return(void)
{
    if (sigsetjmp(s_restart_jmp, 0) == 0) esp_restart();
}

// Registered first, so it runs last: a restart no test catches would end the
// run with status 0 (the Linux port's esp_restart() exits 0)
static void unexpected_restart(void)
{
    printf("UNEXPECTED esp_restart() in %s\n", s_test_name ? s_test_name : "?");
    fflush(stdout);
    exit(1);
}

// Set by a test: the next TX_DONE callback calls esp_restart() (from the
// driver's event task) and records how long the handlers took
static atomic_bool s_restart_in_tx_done;
static atomic_llong s_restart_in_tx_done_us;

static void device_cb(uac2_host_device_handle_t dev, const uac2_host_device_event_t event, void *arg)
{
    (void)dev;
    (void)arg;
    if ((int)event >= 0 && event < 5) atomic_fetch_add(&s_dev_events[event], 1);
    int block_ms = event == UAC2_HOST_DEVICE_EVENT_TX_DONE ? atomic_exchange(&s_block_tx_done_ms, 0) : 0;
    if (block_ms > 0) {
        atomic_store(&s_tx_done_blocking, true);
        vTaskDelay(pdMS_TO_TICKS(block_ms));
        atomic_store(&s_tx_done_blocking, false);
    }
    if (event == UAC2_HOST_DEVICE_EVENT_TX_DONE && atomic_exchange(&s_restart_in_tx_done, false)) {
        int64_t t0 = esp_timer_get_time();
        restart_and_return();
        atomic_store(&s_restart_in_tx_done_us, esp_timer_get_time() - t0);
    }
}

static int dev_events(uac2_host_device_event_t event)
{
    return atomic_load(&s_dev_events[event]);
}

static void install(bool background_task)
{
    uac2_host_driver_config_t cfg = {
        .create_background_task = background_task,
        .task_priority = 5,
        .stack_size = 8192,
        .core_id = 0,
        .callback = driver_cb,
    };
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_install(&cfg));
}

static bool wait_connect(uac2_host_driver_event_t want, uint8_t *addr, uint8_t *iface, int timeout_ms)
{
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    connect_evt_t e;
    while (esp_timer_get_time() < deadline) {
        if (xQueueReceive(s_connects, &e, pdMS_TO_TICKS(10)) == pdTRUE && e.event == want) {
            if (addr) *addr = e.addr;
            if (iface) *iface = e.iface;
            return true;
        }
    }
    return false;
}

static void track(uac2_host_device_handle_t dev, bool add)
{
    for (int i = 0; i < 4; i++) {
        if (add ? s_open[i] == NULL : s_open[i] == dev) {
            s_open[i] = add ? dev : NULL;
            return;
        }
    }
}

static esp_err_t try_open(uint8_t addr, uint8_t iface, uac2_host_device_handle_t *dev)
{
    uac2_host_device_config_t cfg = {.addr = addr, .iface_num = iface, .callback = device_cb};
    esp_err_t err = uac2_host_device_open(&cfg, dev);
    if (err == ESP_OK) track(*dev, true);
    return err;
}

static uac2_host_device_handle_t open_iface(uint8_t addr, uint8_t iface)
{
    uac2_host_device_handle_t dev = NULL;
    TEST_ASSERT_EQUAL(ESP_OK, try_open(addr, iface, &dev));
    return dev;
}

static uac2_host_device_handle_t open_iface_buffered(uint8_t addr, uint8_t iface,
                                                     uint32_t buffer_size, uint32_t threshold)
{
    uac2_host_device_config_t cfg = {.addr = addr, .iface_num = iface, .callback = device_cb,
                                     .buffer_size = buffer_size, .buffer_threshold = threshold};
    uac2_host_device_handle_t dev = NULL;
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_open(&cfg, &dev));
    track(dev, true);
    return dev;
}

/** close() can refuse while teardown finishes; retry like the harness does. */
static esp_err_t close_iface(uac2_host_device_handle_t dev)
{
    esp_err_t err = ESP_FAIL;
    s_step = "close";
    for (int i = 0; i < 20; i++) {
        err = uac2_host_device_close(dev);
        if (err == ESP_OK || err == ESP_ERR_INVALID_ARG) break;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (err == ESP_OK) track(dev, false);
    s_step = NULL;
    return err;
}

/** Device with this configuration (NULL: the captured miniDSP) attached and driver installed. */
static uint8_t attach_config_and_install(const uint8_t *config_desc, uint16_t len)
{
    fake_usb_attach(config_desc, len);
    install(true);
    uint8_t addr = 0;
    uint8_t iface = 0;
    TEST_ASSERT_TRUE(wait_connect(UAC2_HOST_DRIVER_EVENT_TX_CONNECTED, &addr, &iface, 1000));
    TEST_ASSERT_EQUAL(TX_IFACE, iface);
    return addr;
}

/** Device attached and driver installed; returns the playback interface's address. */
static uint8_t attach_and_install(void)
{
    return attach_config_and_install(NULL, 0);
}

static bool wait_until(bool (*cond)(void), int timeout_ms)
{
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (esp_timer_get_time() < deadline) {
        if (cond()) return true;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return cond();
}

static bool device_closed(void) { return fake_usb_stats().device_open_count == 0; }
static bool device_freed(void)  { return fake_usb_stats().device_freed; }

// Writes 10 ms chunks until told to stop or the stream ends
struct writer {
    uac2_host_device_handle_t dev;
    volatile bool stop;
    volatile bool done;
    esp_err_t end_err;
    uint32_t writes;
    TaskHandle_t waiter;
};

// Shared with a task, so not on a test's stack (a failed assert leaves it)
static writer_t s_writer;

// Calls set_sample_rate() through a second handle until told to stop
struct hammer {
    uac2_host_device_handle_t dev;
    volatile bool stop;
    volatile bool done;
    uint32_t ok;
    uint32_t rejected;
    uint32_t other;
    TaskHandle_t waiter;
};

static void writer_task(void *arg)
{
    writer_t *w = (writer_t *)arg;
    // 499 frames (~10.4 ms of 48 kHz 24-bit stereo): not a divisor of the
    // ring buffer size, so the driver reads across its wrap point mid-packet
    static uint8_t chunk[2994];
    uint8_t next = 1;
    bool filled = false;
    w->end_err = ESP_OK;
    while (!w->stop) {
        if (!filled) {
            // Byte counter 1..255, never 0 (the driver's silence padding), so
            // the fake can check that playback data arrives intact and in order
            for (size_t i = 0; i < sizeof(chunk); i++) {
                chunk[i] = next;
                next = next == 255 ? 1 : next + 1;
            }
            filled = true;
        }
        esp_err_t err = uac2_host_device_write(w->dev, chunk, sizeof(chunk), 50);
        s_writer_loops++;
        s_writer_last_err = err;
        if (err == ESP_OK) {
            w->writes++;
            filled = false;
        } else if (err != ESP_ERR_TIMEOUT) {
            w->end_err = err;
            break;
        }
    }
    w->done = true;
    xTaskNotifyGive(w->waiter);
    vTaskDelete(NULL);
}

static void writer_start(writer_t *w, uac2_host_device_handle_t dev)
{
    memset(w, 0, sizeof(*w));
    w->dev = dev;
    w->waiter = xTaskGetCurrentTaskHandle();
    (void)ulTaskNotifyTake(pdTRUE, 0);
    s_active_writer = w;
    TEST_ASSERT_EQUAL(pdPASS, xTaskCreate(writer_task, "writer", 8192, w, 6, NULL));
}

static void writer_join(writer_t *w)
{
    w->stop = true;
    s_step = "writer_join";
    TEST_ASSERT_NOT_EQUAL(0, ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000)));
    s_active_writer = NULL;
    s_step = NULL;
}

/** Wait for the writer to stop by itself (the stream ended), then reap it. */
static void writer_wait_end(writer_t *w, int timeout_ms)
{
    for (int i = 0; i < timeout_ms / 10 && !w->done; i++) vTaskDelay(pdMS_TO_TICKS(10));
    TEST_ASSERT_TRUE_MESSAGE(w->done, "writer did not see the stream end");
    writer_join(w);
}

static uint32_t q16_from_millihertz(uint64_t mhz)
{
    return (uint32_t)((mhz << 16) / 1000000);
}

// A hung test would otherwise hang the whole run (and CI): report which one
// and what every task is doing, then abort.
static void watchdog_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        int64_t started = s_test_started_us;
        if (started == 0 || esp_timer_get_time() - started < 30 * 1000000LL) continue;
        printf("HANG: %s still running after 30 s, test task in: %s\n",
               s_test_name ? s_test_name : "?", s_step ? s_step : "?");
        printf("  writer: %u write calls, last result %d\n", (unsigned)s_writer_loops, s_writer_last_err);
        fake_usb_debug_dump();
        TaskStatus_t tasks[24];
        UBaseType_t n = uxTaskGetSystemState(tasks, 24, NULL);
        static const char *state[] = {"running", "ready", "blocked", "suspended", "deleted", "invalid"};
        for (UBaseType_t i = 0; i < n; i++) {
            printf("  task %-16s %s prio %u\n", tasks[i].pcTaskName,
                   state[tasks[i].eCurrentState <= eInvalid ? tasks[i].eCurrentState : eInvalid],
                   (unsigned)tasks[i].uxCurrentPriority);
        }
        fake_usb_stats_t st = fake_usb_stats();
        printf("  fake: open=%d claimed=%d transfers=%d client=%d freed=%d\n", st.device_open_count,
               st.interfaces_claimed, st.transfers_allocated, st.client_registered, st.device_freed);
        fflush(stdout);
        abort();
    }
}

// ── Unity hooks ──────────────────────────────────────────────────

void setUp(void)
{
    s_test_name = Unity.CurrentTestName;
    s_test_started_us = esp_timer_get_time();
    if (!s_connects) s_connects = xQueueCreate(16, sizeof(connect_evt_t));
    xQueueReset(s_connects);
    for (int i = 0; i < 5; i++) atomic_store(&s_dev_events[i], 0);
    atomic_store(&s_block_tx_done_ms, 0);
    atomic_store(&s_restart_in_tx_done, false);
    atomic_store(&s_restart_in_tx_done_us, -1);
    memset(s_open, 0, sizeof(s_open));
    fake_usb_reset();
}

void tearDown(void)
{
    // A test that failed mid-stream leaves its writer running; it lives on
    // that test's stack, so stop it before anything else
    writer_t *w = s_active_writer;
    if (w) {
        w->stop = true;
        for (int i = 0; i < 200 && !w->done; i++) vTaskDelay(pdMS_TO_TICKS(10));
        s_active_writer = NULL;
    }
    hammer_t *h = s_active_hammer;
    if (h) {
        h->stop = true;
        for (int i = 0; i < 200 && !h->done; i++) vTaskDelay(pdMS_TO_TICKS(10));
        s_active_hammer = NULL;
    }
    // Best-effort cleanup so a failed test does not cascade
    (void)esp_unregister_shutdown_handler(restart_jump_back);
    for (int i = 0; i < 4; i++) {
        if (s_open[i]) (void)close_iface(s_open[i]);
    }
    s_step = "teardown uninstall";
    if (fake_usb_stats().client_registered) {
        fake_usb_detach();   // releases an orphan left behind by a failed test
        for (int i = 0; i < 20 && uac2_host_uninstall() != ESP_OK; i++) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
    fake_usb_stats_t st = fake_usb_stats();
    TEST_ASSERT_FALSE_MESSAGE(st.client_registered, "driver still installed");
    TEST_ASSERT_EQUAL_MESSAGE(0, st.free_while_in_flight, "transfer freed while in flight");
    TEST_ASSERT_EQUAL_MESSAGE(0, st.close_with_ctrl_in_flight, "device closed with EP0 busy");
    TEST_ASSERT_EQUAL_MESSAGE(0, st.invalid_transfers, "transfer the real library would reject");
    TEST_ASSERT_EQUAL_MESSAGE(0, st.release_with_pending, "interface released with its transfers unfinished");
    TEST_ASSERT_EQUAL_MESSAGE(0, st.halt_while_active, "endpoint halted with a transfer executing");
    TEST_ASSERT_EQUAL_MESSAGE(0, st.alt0_with_urbs_in_flight, "SET_INTERFACE(0) with transfers in flight");
    TEST_ASSERT_EQUAL_MESSAGE(0, st.submit_on_inactive_alt, "transfer submitted to an alt the device is not running");
    TEST_ASSERT_EQUAL_MESSAGE(0, st.ctrl_in_unrounded, "IN control transfer not rounded to bMaxPacketSize0");
    TEST_ASSERT_EQUAL_MESSAGE(0, st.ctrl_bad_wlength, "GET CUR wLength not the control's size");
    TEST_ASSERT_EQUAL_MESSAGE(0, st.ctrl_wrong_interface, "class request not sent to the AC interface");
    TEST_ASSERT_EQUAL_MESSAGE(0, st.device_open_count, "device left open");
    TEST_ASSERT_EQUAL_MESSAGE(0, st.transfers_allocated, "transfers leaked");
    s_test_started_us = 0;
}

// ── Discovery and lifecycle ──────────────────────────────────────

static void test_attached_device_reported_by_install_scan(void)
{
    fake_usb_attach(NULL, 0);
    install(true);
    uint8_t iface = 0;
    TEST_ASSERT_TRUE(wait_connect(UAC2_HOST_DRIVER_EVENT_TX_CONNECTED, NULL, &iface, 1000));
    TEST_ASSERT_EQUAL(TX_IFACE, iface);
    TEST_ASSERT_TRUE(wait_connect(UAC2_HOST_DRIVER_EVENT_RX_CONNECTED, NULL, &iface, 1000));
    TEST_ASSERT_EQUAL(RX_IFACE, iface);
    // NEW_DEV for the same device must not report it twice
    TEST_ASSERT_FALSE(wait_connect(UAC2_HOST_DRIVER_EVENT_TX_CONNECTED, NULL, NULL, 300));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void test_device_attached_after_install_reported(void)
{
    install(true);
    TEST_ASSERT_FALSE(wait_connect(UAC2_HOST_DRIVER_EVENT_TX_CONNECTED, NULL, NULL, 100));
    fake_usb_attach(NULL, 0);
    TEST_ASSERT_TRUE(wait_connect(UAC2_HOST_DRIVER_EVENT_TX_CONNECTED, NULL, NULL, 1000));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void test_reinstall_with_device_attached(void)
{
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
    install(true);
    TEST_ASSERT_TRUE(wait_connect(UAC2_HOST_DRIVER_EVENT_TX_CONNECTED, &addr, NULL, 1000));
    dev = open_iface(addr, TX_IFACE);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &k_tx_48k));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(dev));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void test_uninstall_refused_while_interface_open(void)
{
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, uac2_host_uninstall());
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());   // not installed: OK
}

static void test_malformed_configuration_not_reported(void)
{
    uint8_t cfg[sizeof(minidsp_2x4hd_config1_desc)];
    memcpy(cfg, minidsp_2x4hd_config1_desc, sizeof(cfg));
    for (int off = 0; off < (int)sizeof(cfg); off += cfg[off]) {
        if (cfg[off + 1] == 0x05) {   // first endpoint descriptor
            cfg[off] = 0;
            break;
        }
    }
    fake_usb_attach(cfg, sizeof(cfg));
    install(true);
    TEST_ASSERT_FALSE(wait_connect(UAC2_HOST_DRIVER_EVENT_TX_CONNECTED, NULL, NULL, 300));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

typedef struct {
    esp_err_t result;
    TaskHandle_t waiter;
} uninstall_job_t;

static void uninstall_task(void *arg)
{
    uninstall_job_t *job = (uninstall_job_t *)arg;
    job->result = uac2_host_uninstall();
    xTaskNotifyGive(job->waiter);
    vTaskDelete(NULL);
}

static void test_concurrent_uninstalls(void)
{
    attach_and_install();
    static uninstall_job_t a, b;
    a = (uninstall_job_t){.result = ESP_FAIL, .waiter = xTaskGetCurrentTaskHandle()};
    b = a;
    (void)ulTaskNotifyTake(pdTRUE, 0);
    xTaskCreate(uninstall_task, "un_a", 4096, &a, 6, NULL);
    xTaskCreate(uninstall_task, "un_b", 4096, &b, 6, NULL);
    int done = 0;
    while (done < 2 && ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(3000))) done++;
    TEST_ASSERT_EQUAL(2, done);
    // One tears it down; the other either saw it in progress or already gone
    TEST_ASSERT_TRUE(a.result == ESP_OK || a.result == ESP_ERR_INVALID_STATE);
    TEST_ASSERT_TRUE(b.result == ESP_OK || b.result == ESP_ERR_INVALID_STATE);
    TEST_ASSERT_TRUE(a.result == ESP_OK || b.result == ESP_OK);
    TEST_ASSERT_FALSE(fake_usb_stats().client_registered);
}

// ── Manual event pump ────────────────────────────────────────────

typedef struct {
    int iterations_before_uninstall;   // > 0: pump calls uninstall itself
    esp_err_t last_ret;
    esp_err_t uninstall_ret;
    TaskHandle_t waiter;
} pump_t;

static void pump_task(void *arg)
{
    pump_t *p = (pump_t *)arg;
    if (p->iterations_before_uninstall > 0) {
        for (int i = 0; i < p->iterations_before_uninstall; i++) {
            p->last_ret = uac2_host_handle_events(pdMS_TO_TICKS(10));
        }
        p->uninstall_ret = uac2_host_uninstall();   // between calls, from the pump task
    } else {
        do {
            p->last_ret = uac2_host_handle_events(portMAX_DELAY);
        } while (p->last_ret != ESP_FAIL && p->last_ret != ESP_ERR_INVALID_STATE);
    }
    xTaskNotifyGive(p->waiter);
    vTaskDelete(NULL);
}

static void test_manual_pump_uninstall_from_pump_task(void)
{
    fake_usb_attach(NULL, 0);
    install(false);
    static pump_t p;
    p = (pump_t){.iterations_before_uninstall = 30, .waiter = xTaskGetCurrentTaskHandle()};
    (void)ulTaskNotifyTake(pdTRUE, 0);
    xTaskCreate(pump_task, "pump", 8192, &p, 5, NULL);
    TEST_ASSERT_NOT_EQUAL(0, ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(3000)));
    TEST_ASSERT_EQUAL(ESP_OK, p.uninstall_ret);
    TEST_ASSERT_TRUE(wait_connect(UAC2_HOST_DRIVER_EVENT_TX_CONNECTED, NULL, NULL, 10));
}

static void test_manual_pump_uninstall_from_other_task(void)
{
    fake_usb_attach(NULL, 0);
    install(false);
    static pump_t p;
    p = (pump_t){.waiter = xTaskGetCurrentTaskHandle()};
    (void)ulTaskNotifyTake(pdTRUE, 0);
    xTaskCreate(pump_task, "pump", 8192, &p, 5, NULL);
    TEST_ASSERT_TRUE(wait_connect(UAC2_HOST_DRIVER_EVENT_TX_CONNECTED, NULL, NULL, 1000));
    // The pump is now blocked in handle_events(portMAX_DELAY)
    vTaskDelay(pdMS_TO_TICKS(50));
    int64_t t0 = esp_timer_get_time();
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
    TEST_ASSERT_LESS_THAN(1000000, esp_timer_get_time() - t0);
    TEST_ASSERT_NOT_EQUAL(0, ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)));
    TEST_ASSERT_EQUAL(ESP_FAIL, p.last_ret);
}

// ── Playback ─────────────────────────────────────────────────────

static void test_playback_48k_sends_whole_288_byte_packets(void)
{
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &k_tx_48k));
    TEST_ASSERT_NOT_EQUAL(0, uac2_host_device_get_start_time(dev));
    writer_t *w = &s_writer;
    writer_start(w, dev);
    vTaskDelay(pdMS_TO_TICKS(200));
    fake_usb_clear_out_stats();
    vTaskDelay(pdMS_TO_TICKS(1000));
    fake_usb_stats_t st = fake_usb_stats();
    writer_join(w);
    TEST_ASSERT_EQUAL(ESP_OK, w->end_err);
    TEST_ASSERT_GREATER_THAN(50, w->writes);
    TEST_ASSERT_UINT64_WITHIN(150, 1000, st.out_packets);
    TEST_ASSERT_EQUAL(288, st.out_min_packet);
    TEST_ASSERT_EQUAL(288, st.out_max_packet);
    // Every written byte arrives once and in order, across ring-buffer wraps
    TEST_ASSERT_GREATER_THAN(288 * 500, st.out_nonzero_bytes);
    TEST_ASSERT_EQUAL(0, st.out_pattern_breaks);
    TEST_ASSERT_EQUAL(0, st.out_silence_after_data);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(dev));
    TEST_ASSERT_EQUAL(0, uac2_host_device_get_start_time(dev));
    TEST_ASSERT_EQUAL(0, uac2_host_device_get_feedback(dev));
    TEST_ASSERT_EQUAL(0, fake_usb_stats().interfaces_claimed);
    TEST_ASSERT_EQUAL(0, dev_events(UAC2_HOST_DEVICE_EVENT_STREAM_ERROR));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void test_playback_follows_feedback_drift(void)
{
    uint8_t addr = attach_and_install();
    fake_usb_set_feedback_q16(q16_from_millihertz(48000900));   // 48.0009 samples/frame
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &k_tx_48k));
    writer_t *w = &s_writer;
    writer_start(w, dev);
    vTaskDelay(pdMS_TO_TICKS(300));
    fake_usb_clear_out_stats();
    vTaskDelay(pdMS_TO_TICKS(4000));
    fake_usb_stats_t st = fake_usb_stats();
    writer_join(w);
    double avg = (double)st.out_bytes / 6.0 / (double)st.out_packets;
    printf("    average %.5f samples/packet over %llu packets (device asks 48.0009)\n",
           avg, (unsigned long long)st.out_packets);
    TEST_ASSERT_TRUE(avg > 48.0004 && avg < 48.0014);
    TEST_ASSERT_EQUAL(288, st.out_min_packet);
    TEST_ASSERT_EQUAL(294, st.out_max_packet);
    TEST_ASSERT_GREATER_THAN(0, st.out_nonzero_bytes);
    TEST_ASSERT_EQUAL(0, st.out_pattern_breaks);
    TEST_ASSERT_EQUAL(0, st.out_silence_after_data);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(dev));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void test_suspend_and_resume(void)
{
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &k_tx_48k));
    writer_t *w = &s_writer;
    writer_start(w, dev);
    vTaskDelay(pdMS_TO_TICKS(200));
    writer_join(w);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_suspend(dev));
    fake_usb_clear_out_stats();
    vTaskDelay(pdMS_TO_TICKS(100));
    TEST_ASSERT_EQUAL(0, fake_usb_stats().out_packets);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_resume(dev));
    writer_start(w, dev);
    vTaskDelay(pdMS_TO_TICKS(300));
    writer_join(w);
    TEST_ASSERT_EQUAL(ESP_OK, w->end_err);
    TEST_ASSERT_GREATER_THAN(200, fake_usb_stats().out_packets);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(dev));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void test_teardown_after_channel_error(void)
{
    // A channel error halts the pipe with URBs still queued behind it
    // (hcd_dwc.c). They never complete on their own, so suspend() falls back
    // to halting, which returns at once on a halted pipe. The feedback
    // endpoint has one URB, so an error there leaves nothing queued for stop().
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &k_tx_48k));
    vTaskDelay(pdMS_TO_TICKS(50));
    fake_usb_error_halt_endpoint(0x01);
    vTaskDelay(pdMS_TO_TICKS(20));
    int64_t t0 = esp_timer_get_time();
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_suspend(dev));
    TEST_ASSERT_LESS_THAN(500000, esp_timer_get_time() - t0);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_resume(dev));
    fake_usb_clear_out_stats();
    vTaskDelay(pdMS_TO_TICKS(100));
    TEST_ASSERT_GREATER_THAN(50, fake_usb_stats().out_packets);   // the pipe was cleared

    fake_usb_error_halt_endpoint(0x81);   // feedback, every 8 ms
    vTaskDelay(pdMS_TO_TICKS(30));
    t0 = esp_timer_get_time();
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(dev));
    TEST_ASSERT_LESS_THAN(500000, esp_timer_get_time() - t0);
    TEST_ASSERT_EQUAL(0, dev_events(UAC2_HOST_DEVICE_EVENT_STREAM_ERROR));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

#if !CONFIG_UAC2_STOP_STREAMS_ON_RESTART
#error "test_restart_stops_running_streams needs CONFIG_UAC2_STOP_STREAMS_ON_RESTART (sdkconfig.defaults)"
#endif

typedef struct {
    uac2_host_device_handle_t dev;
    atomic_bool started;
    atomic_bool done;
    esp_err_t err;
} stop_job_t;

static void stop_task(void *arg)
{
    stop_job_t *job = arg;
    atomic_store(&job->started, true);
    job->err = uac2_host_device_stop(job->dev);
    atomic_store(&job->done, true);
    vTaskDelete(NULL);
}

typedef struct {
    atomic_bool done;
    uint32_t set_interface_calls;   // when its esp_restart() handlers returned
} restart_job_t;

static void restart_task(void *arg)
{
    restart_job_t *job = arg;
    restart_and_return();
    job->set_interface_calls = fake_usb_stats().set_interface_calls;
    atomic_store(&job->done, true);
    vTaskDelete(NULL);
}

static void test_restart_stops_running_streams(void)
{
    // A self-powered device keeps VBUS across an ESP32 reset. The driver's
    // shutdown handler stops the stream first, so the device sees alt 0 (a
    // miniDSP 2x4 HD reset mid-stream stays silent until it reboots).
    TEST_ASSERT_EQUAL(ESP_OK, esp_register_shutdown_handler(restart_jump_back));
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &k_tx_48k));
    vTaskDelay(pdMS_TO_TICKS(100));
    uint32_t set_interface_calls = fake_usb_stats().set_interface_calls;
    restart_and_return();
    TEST_ASSERT_EQUAL(set_interface_calls + 1, fake_usb_stats().set_interface_calls);
    uac2_host_stream_info_t info;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, uac2_host_device_get_stream_info(dev, &info));
    fake_usb_clear_out_stats();
    vTaskDelay(pdMS_TO_TICKS(50));
    TEST_ASSERT_EQUAL(0, fake_usb_stats().out_packets);

    // From the event task (a driver callback) the handler can't wait for
    // transfers, so it leaves the stream alone instead of timing out
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &k_tx_48k));
    vTaskDelay(pdMS_TO_TICKS(50));
    set_interface_calls = fake_usb_stats().set_interface_calls;
    atomic_store(&s_restart_in_tx_done, true);
    uint8_t chunk[288] = {0};
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_write(dev, chunk, sizeof(chunk), 100));   // re-arms TX_DONE
    for (int i = 0; i < 100 && atomic_load(&s_restart_in_tx_done_us) < 0; i++) vTaskDelay(pdMS_TO_TICKS(5));
    long long cb_us = atomic_load(&s_restart_in_tx_done_us);
    TEST_ASSERT_GREATER_OR_EQUAL(0, cb_us);
    TEST_ASSERT_LESS_THAN(50000, cb_us);
    TEST_ASSERT_EQUAL(set_interface_calls, fake_usb_stats().set_interface_calls);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_stream_info(dev, &info));
    TEST_ASSERT_TRUE(info.active);

    // A stop() already under way (waiting on a URB callback) isn't at alt 0
    // yet: the handler waits for it rather than skipping the stream
    vTaskDelay(pdMS_TO_TICKS(20));
    atomic_store(&s_block_tx_done_ms, 300);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_write(dev, chunk, sizeof(chunk), 100));
    for (int i = 0; i < 100 && !atomic_load(&s_tx_done_blocking); i++) vTaskDelay(pdMS_TO_TICKS(5));
    TEST_ASSERT_TRUE(atomic_load(&s_tx_done_blocking));
    static stop_job_t job;
    job = (stop_job_t){.dev = dev};
    // Above this task (one core): it runs into stop() until stop() blocks on
    // the held callback, before xTaskCreate() returns here
    TEST_ASSERT_EQUAL(pdPASS, xTaskCreate(stop_task, "stop", 4096, &job, 11, NULL));
    TEST_ASSERT_TRUE(atomic_load(&job.started));
    TEST_ASSERT_FALSE(atomic_load(&job.done));
    TEST_ASSERT_TRUE(atomic_load(&s_tx_done_blocking));
    set_interface_calls = fake_usb_stats().set_interface_calls;
    restart_and_return();
    TEST_ASSERT_EQUAL(set_interface_calls + 1, fake_usb_stats().set_interface_calls);
    for (int i = 0; i < 100 && !atomic_load(&job.done); i++) vTaskDelay(pdMS_TO_TICKS(5));
    TEST_ASSERT_TRUE(atomic_load(&job.done));
    TEST_ASSERT_EQUAL(ESP_OK, job.err);

    // Started suspended: READY, but its alt is already selected
    uac2_host_stream_config_t suspended_cfg = k_tx_48k;
    suspended_cfg.flags = UAC2_FLAG_STREAM_SUSPEND_AFTER_START;
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &suspended_cfg));
    set_interface_calls = fake_usb_stats().set_interface_calls;
    restart_and_return();
    TEST_ASSERT_EQUAL(set_interface_calls + 1, fake_usb_stats().set_interface_calls);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, uac2_host_device_get_stream_info(dev, &info));

    // Two tasks restarting at once: the second, which runs once this task
    // blocks in the handler, returns only after the stream reached alt 0
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &k_tx_48k));
    vTaskDelay(pdMS_TO_TICKS(20));
    atomic_store(&s_block_tx_done_ms, 200);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_write(dev, chunk, sizeof(chunk), 100));
    for (int i = 0; i < 100 && !atomic_load(&s_tx_done_blocking); i++) vTaskDelay(pdMS_TO_TICKS(5));
    TEST_ASSERT_TRUE(atomic_load(&s_tx_done_blocking));
    static restart_job_t second;
    second = (restart_job_t){0};
    TEST_ASSERT_EQUAL(pdPASS, xTaskCreate(restart_task, "restart2", 4096, &second, 9, NULL));
    set_interface_calls = fake_usb_stats().set_interface_calls;
    restart_and_return();
    for (int i = 0; i < 200 && !atomic_load(&second.done); i++) vTaskDelay(pdMS_TO_TICKS(5));
    TEST_ASSERT_TRUE(atomic_load(&second.done));
    TEST_ASSERT_EQUAL(set_interface_calls + 1, second.set_interface_calls);
    TEST_ASSERT_EQUAL(set_interface_calls + 1, fake_usb_stats().set_interface_calls);

    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
    TEST_ASSERT_EQUAL(ESP_OK, esp_unregister_shutdown_handler(restart_jump_back));
}

static void test_stop_waits_for_a_callback_mid_resubmit(void)
{
    // A URB callback that checked for ACTIVE just before stop() changed the
    // state still resubmits. stop() waits for it instead of timing out its
    // drain and selecting alt 0 (or halting) under the late transfer.
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &k_tx_48k));
    vTaskDelay(pdMS_TO_TICKS(50));
    atomic_store(&s_block_tx_done_ms, 300);
    uint8_t chunk[288] = {0};
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_write(dev, chunk, sizeof(chunk), 100));   // re-arms TX_DONE
    for (int i = 0; i < 100 && !atomic_load(&s_tx_done_blocking); i++) vTaskDelay(pdMS_TO_TICKS(5));
    TEST_ASSERT_TRUE(atomic_load(&s_tx_done_blocking));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(dev));
    TEST_ASSERT_FALSE(atomic_load(&s_tx_done_blocking));
    TEST_ASSERT_EQUAL(0, fake_usb_stats().submit_on_inactive_alt);

    // Held past stop()'s 2 s wait: stop() gives up without touching the
    // endpoints or the alt, and a later stop() finishes the job
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &k_tx_48k));
    vTaskDelay(pdMS_TO_TICKS(50));
    atomic_store(&s_block_tx_done_ms, 2500);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_write(dev, chunk, sizeof(chunk), 100));
    for (int i = 0; i < 100 && !atomic_load(&s_tx_done_blocking); i++) vTaskDelay(pdMS_TO_TICKS(5));
    TEST_ASSERT_TRUE(atomic_load(&s_tx_done_blocking));
    uint32_t set_interface_calls = fake_usb_stats().set_interface_calls;
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, uac2_host_device_stop(dev));
    TEST_ASSERT_EQUAL(set_interface_calls, fake_usb_stats().set_interface_calls);
    for (int i = 0; i < 200 && atomic_load(&s_tx_done_blocking); i++) vTaskDelay(pdMS_TO_TICKS(5));
    TEST_ASSERT_FALSE(atomic_load(&s_tx_done_blocking));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(dev));
    TEST_ASSERT_EQUAL(0, fake_usb_stats().submit_on_inactive_alt);
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void test_io_after_close_is_invalid_state(void)
{
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    uint8_t chunk[288] = {0};
    uint32_t got = 1;
    // The handle is freed; the driver only compares it against its list
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, uac2_host_device_write(dev, chunk, sizeof(chunk), 0));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, uac2_host_device_read(dev, chunk, sizeof(chunk), &got, 0));
    TEST_ASSERT_EQUAL(0, got);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

// ── Faults ───────────────────────────────────────────────────────

static void test_unplug_during_playback_then_replug(void)
{
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &k_tx_48k));
    writer_t *w = &s_writer;
    writer_start(w, dev);
    vTaskDelay(pdMS_TO_TICKS(300));
    fake_usb_detach();
    writer_wait_end(w, 2000);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, w->end_err);
    TEST_ASSERT_EQUAL(1, dev_events(UAC2_HOST_DEVICE_EVENT_DISCONNECTED));
    TEST_ASSERT_EQUAL(0, dev_events(UAC2_HOST_DEVICE_EVENT_STREAM_ERROR));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_TRUE(wait_until(device_freed, 1000));

    fake_usb_attach(NULL, 0);
    TEST_ASSERT_TRUE(wait_connect(UAC2_HOST_DRIVER_EVENT_TX_CONNECTED, &addr, NULL, 1000));
    dev = open_iface(addr, TX_IFACE);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &k_tx_48k));
    vTaskDelay(pdMS_TO_TICKS(100));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(dev));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void test_unplug_while_idle_open(void)
{
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    fake_usb_detach();
    vTaskDelay(pdMS_TO_TICKS(100));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, uac2_host_device_start(dev, &k_tx_48k));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_TRUE(wait_until(device_freed, 1000));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void test_failed_urb_allocation_rolls_back(void)
{
    // The second URB allocation fails: start reports it and leaves nothing
    // behind, so the next start works (tearDown checks for leaked transfers)
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    fake_usb_fail_transfer_alloc_after(1);
    TEST_ASSERT_EQUAL(ESP_ERR_NO_MEM, uac2_host_device_start(dev, &k_tx_48k));
    TEST_ASSERT_EQUAL(0, fake_usb_stats().interfaces_claimed);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_set_sample_rate(dev, 44100));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &k_tx_48k));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(dev));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void test_start_after_unplug_before_close_refused(void)
{
    // The unplugged stream keeps its ring buffer and URBs until close();
    // starting again must not allocate over them (tearDown checks for
    // leaked transfers)
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &k_tx_48k));
    vTaskDelay(pdMS_TO_TICKS(100));
    fake_usb_detach();
    vTaskDelay(pdMS_TO_TICKS(100));
    TEST_ASSERT_EQUAL(1, dev_events(UAC2_HOST_DEVICE_EVENT_DISCONNECTED));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, uac2_host_device_start(dev, &k_tx_48k));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_TRUE(wait_until(device_freed, 1000));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void test_control_timeout_orphan_then_late_reply(void)
{
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    fake_usb_hang_ctrl(CLOCK_SOURCE, CS_CLOCK_VALID, true);
    bool valid = false;
    int64_t t0 = esp_timer_get_time();
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, uac2_host_device_get_clock_valid(dev, &valid));
    TEST_ASSERT_GREATER_OR_EQUAL(900000, esp_timer_get_time() - t0);
    uint32_t rate = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, uac2_host_device_get_sample_rate(dev, &rate));

    // Close succeeds; the device stays open in the driver with the URB
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(1, fake_usb_stats().device_open_count);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, try_open(addr, TX_IFACE, &dev));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, uac2_host_uninstall());

    TEST_ASSERT_TRUE(fake_usb_release_hung_ctrl());
    TEST_ASSERT_TRUE(wait_until(device_closed, 1000));
    dev = open_iface(addr, TX_IFACE);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_sample_rate(dev, &rate));
    TEST_ASSERT_EQUAL(48000, rate);
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

/**
 * A timed-out request during start()'s clock setup is reported as the
 * timeout. The next request would otherwise fail on the URB the timeout left
 * in flight and replace it with ESP_ERR_INVALID_STATE.
 */
static void test_start_reports_clock_setup_timeout(void)
{
    static const struct {
        uint8_t request;
        bool get;
        bool stall_set;
        const char *what;
    } cases[] = {
        {0x02, true, false, "RANGE before the SET"},
        {0x01, false, false, "the SET"},
        {0x01, true, false, "the readback after the SET"},
        {0x01, true, true, "the readback after a STALLed SET"},
    };
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        printf("    hang %s\n", cases[i].what);
        if (cases[i].stall_set) fake_usb_stall_ctrl(CLOCK_SOURCE, 0x01, 0, false);
        fake_usb_hang_ctrl_request(CLOCK_SOURCE, 0x01, cases[i].request, cases[i].get);
        TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, uac2_host_device_start(dev, &k_tx_48k));
        TEST_ASSERT_EQUAL(0, fake_usb_stats().interfaces_claimed);
        fake_usb_stall_ctrl_clear();
        TEST_ASSERT_TRUE(fake_usb_release_hung_ctrl());
        vTaskDelay(pdMS_TO_TICKS(50));      // the late reply reaches the driver
    }
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &k_tx_48k));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(dev));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

/** A timed-out request whose late reply is a STALL leaves EP0 recovering for the next one. */
static void test_late_stall_then_request_waits_for_ep0(void)
{
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    fake_usb_set_ep0_recovery_frames(30);
    fake_usb_hang_ctrl(CLOCK_SOURCE, CS_CLOCK_VALID, true);
    bool valid = false;
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, uac2_host_device_get_clock_valid(dev, &valid));
    fake_usb_stall_ctrl(CLOCK_SOURCE, CS_CLOCK_VALID, 0, true);     // the late reply STALLs
    TEST_ASSERT_TRUE(fake_usb_release_hung_ctrl());
    vTaskDelay(pdMS_TO_TICKS(10));      // delivered; EP0 refuses submits for ~20 ms more
    uint32_t rate = 0;
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_sample_rate(dev, &rate));
    TEST_ASSERT_EQUAL(48000, rate);
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void test_control_timeout_orphan_reclaimed_on_unplug(void)
{
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    fake_usb_hang_ctrl(CLOCK_SOURCE, CS_CLOCK_VALID, true);
    bool valid = false;
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, uac2_host_device_get_clock_valid(dev, &valid));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    fake_usb_detach();
    TEST_ASSERT_TRUE(wait_until(device_freed, 1000));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void test_late_reply_consumed_by_next_request(void)
{
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    fake_usb_hang_ctrl(CLOCK_SOURCE, CS_CLOCK_VALID, true);
    bool valid = false;
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, uac2_host_device_get_clock_valid(dev, &valid));
    TEST_ASSERT_TRUE(fake_usb_release_hung_ctrl());
    vTaskDelay(pdMS_TO_TICKS(50));
    // The abandoned completion is drained first, then the new request runs
    uint32_t rate = 0;
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_sample_rate(dev, &rate));
    TEST_ASSERT_EQUAL(48000, rate);
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void test_interface_release_retried_while_busy(void)
{
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &k_tx_48k));
    vTaskDelay(pdMS_TO_TICKS(100));
    fake_usb_set_release_busy(3);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(dev));
    TEST_ASSERT_EQUAL(0, fake_usb_stats().interfaces_claimed);
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void test_dead_data_endpoint_reaches_stream_error(void)
{
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &k_tx_48k));
    writer_t *w = &s_writer;
    writer_start(w, dev);
    vTaskDelay(pdMS_TO_TICKS(100));
    // Feedback keeps working while every data packet fails
    fake_usb_fail_out_urbs(USB_TRANSFER_STATUS_ERROR, 100000);
    int64_t deadline = esp_timer_get_time() + 2000000;
    while (dev_events(UAC2_HOST_DEVICE_EVENT_STREAM_ERROR) == 0 && esp_timer_get_time() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    TEST_ASSERT_EQUAL(1, dev_events(UAC2_HOST_DEVICE_EVENT_STREAM_ERROR));
    writer_wait_end(w, 2000);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, w->end_err);
    fake_usb_fail_out_urbs(USB_TRANSFER_STATUS_COMPLETED, 0);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(dev));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void test_skipped_feedback_is_not_fatal(void)
{
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &k_tx_48k));
    writer_t *w = &s_writer;
    writer_start(w, dev);
    fake_usb_set_feedback_packet_status(USB_TRANSFER_STATUS_SKIPPED, 50);   // 400 ms of feedback slots
    vTaskDelay(pdMS_TO_TICKS(700));
    writer_join(w);
    TEST_ASSERT_EQUAL(ESP_OK, w->end_err);
    TEST_ASSERT_EQUAL(0, dev_events(UAC2_HOST_DEVICE_EVENT_STREAM_ERROR));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(dev));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void test_close_active_stream_without_stop(void)
{
    uint8_t addr = attach_and_install();
    for (int i = 0; i < 20; i++) {
        uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
        TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &k_tx_48k));
        writer_t *w = &s_writer;
        writer_start(w, dev);
        vTaskDelay(pdMS_TO_TICKS(30 + (i % 5) * 7));
        writer_join(w);
        TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
        TEST_ASSERT_EQUAL(0, fake_usb_stats().interfaces_claimed);
    }
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void test_close_while_writer_running(void)
{
    uint8_t addr = attach_and_install();
    for (int i = 0; i < 20; i++) {
        uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
        TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &k_tx_48k));
        writer_t *w = &s_writer;
        writer_start(w, dev);
        vTaskDelay(pdMS_TO_TICKS(20 + (i % 5) * 7));
        // The writer is blocked in or about to call write() during teardown
        TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
        TEST_ASSERT_NOT_EQUAL(0, ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000)));
        s_active_writer = NULL;
        TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, w->end_err);
    }
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

// ── Shared clock ─────────────────────────────────────────────────

static void test_set_sample_rate_refused_while_streaming(void)
{
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t tx = open_iface(addr, TX_IFACE);
    uac2_host_device_handle_t rx = open_iface(addr, RX_IFACE);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(tx, &k_tx_48k));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, uac2_host_device_set_sample_rate(rx, 44100));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, uac2_host_device_set_sample_rate(tx, 44100));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(tx));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_set_sample_rate(rx, 44100));
    TEST_ASSERT_EQUAL(44100, fake_usb_stats().last_set_rate);
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(rx));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(tx));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void test_sibling_stream_at_other_rate_refused(void)
{
    // The Linux target is not ESP32-S3, so TX and RX may stream together
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t tx = open_iface(addr, TX_IFACE);
    uac2_host_device_handle_t rx = open_iface(addr, RX_IFACE);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(tx, &k_tx_48k));
    uac2_host_stream_config_t rx_44k = k_rx_48k;
    rx_44k.sample_freq = 44100;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, uac2_host_device_start(rx, &rx_44k));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(rx, &k_rx_48k));
    vTaskDelay(pdMS_TO_TICKS(100));
    uint8_t buf[288];
    uint32_t got = 0;
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_read(rx, buf, sizeof(buf), &got, 100));
    TEST_ASSERT_GREATER_THAN(0, got);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(rx));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(tx));
    TEST_ASSERT_EQUAL(0, fake_usb_stats().set_rate_while_streaming);
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(rx));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(tx));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}


static void hammer_task(void *arg)
{
    hammer_t *h = (hammer_t *)arg;
    for (uint32_t i = 0; !h->stop; i++) {
        esp_err_t err = uac2_host_device_set_sample_rate(h->dev, (i & 1) ? 48000 : 44100);
        if (err == ESP_OK) h->ok++;
        else if (err == ESP_ERR_INVALID_STATE) h->rejected++;
        else h->other++;
        vTaskDelay(1);
    }
    h->done = true;
    xTaskNotifyGive(h->waiter);
    vTaskDelete(NULL);
}

static void test_clock_never_changes_under_a_stream(void)
{
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t tx = open_iface(addr, TX_IFACE);
    uac2_host_device_handle_t rx = open_iface(addr, RX_IFACE);
    static hammer_t h;
    h = (hammer_t){.dev = rx, .waiter = xTaskGetCurrentTaskHandle()};
    (void)ulTaskNotifyTake(pdTRUE, 0);
    s_active_hammer = &h;
    xTaskCreate(hammer_task, "hammer", 8192, &h, 6, NULL);
    for (int i = 0; i < 15; i++) {
        TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(tx, &k_tx_48k));
        vTaskDelay(pdMS_TO_TICKS(40));
        TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(tx));
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    h.stop = true;
    TEST_ASSERT_NOT_EQUAL(0, ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000)));
    s_active_hammer = NULL;
    printf("    hammer: %u set, %u refused, %u other\n", (unsigned)h.ok, (unsigned)h.rejected, (unsigned)h.other);
    TEST_ASSERT_EQUAL(0, fake_usb_stats().set_rate_while_streaming);
    TEST_ASSERT_GREATER_THAN(0, h.rejected);
    TEST_ASSERT_EQUAL(0, h.other);
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(rx));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(tx));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

// ── Runner ───────────────────────────────────────────────────────

// ── Device compatibility ─────────────────────────────────────────

#define ENT_FU_PLAYBACK 10
#define CS_FU_MUTE      0x01
#define CS_FU_VOLUME    0x02

static uint8_t s_cfg_buf[512];

static void test_adaptive_endpoint_plays_44k1_without_feedback(void)
{
    uint16_t len = fake_usb_no_feedback_config(UAC2_EP_SYNC_ADAPTIVE, s_cfg_buf, sizeof(s_cfg_buf));
    TEST_ASSERT_NOT_EQUAL(0, len);
    uint8_t addr = attach_config_and_install(s_cfg_buf, len);
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    const uac2_host_stream_config_t cfg = {.channels = 2, .bit_resolution = 24, .sample_freq = 44100};
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &cfg));
    uac2_host_stream_info_t info;
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_stream_info(dev, &info));
    TEST_ASSERT_EQUAL(0, info.fb_ep_addr);
    TEST_ASSERT_EQUAL(UAC2_EP_SYNC_ADAPTIVE, info.sync_type);
    writer_t *w = &s_writer;
    writer_start(w, dev);
    vTaskDelay(pdMS_TO_TICKS(200));
    // From the first (silent) packet on: nine 44-frame packets, then one of 45.
    // 16.16 rounding of 44.1 would send its first 45-frame packet one later.
    fake_usb_stats_t first = fake_usb_stats();
    TEST_ASSERT_EQUAL(32, first.first_out_count);
    for (int i = 0; i < 32; i++) {
        TEST_ASSERT_EQUAL_MESSAGE((i % 10 == 9) ? 45 * 6 : 44 * 6, first.first_out_packets[i],
                                  "packet size out of the 44/45 pattern");
    }
    fake_usb_clear_out_stats();
    vTaskDelay(pdMS_TO_TICKS(2000));
    fake_usb_stats_t st = fake_usb_stats();
    writer_join(w);
    TEST_ASSERT_EQUAL(ESP_OK, w->end_err);
    // Still 44.1 frames per packet, within one frame for a window that starts
    // mid-pattern
    int64_t frames = (int64_t)(st.out_bytes / 6);
    int64_t diff = frames * 10 - (int64_t)st.out_packets * 441;
    printf("    %lld frames in %llu packets\n", (long long)frames, (unsigned long long)st.out_packets);
    TEST_ASSERT_TRUE(diff >= -10 && diff <= 10);
    TEST_ASSERT_EQUAL(44 * 6, st.out_min_packet);
    TEST_ASSERT_EQUAL(45 * 6, st.out_max_packet);
    TEST_ASSERT_GREATER_THAN(0, st.out_nonzero_bytes);
    TEST_ASSERT_EQUAL(0, st.out_pattern_breaks);
    TEST_ASSERT_EQUAL(0, dev_events(UAC2_HOST_DEVICE_EVENT_STREAM_ERROR));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(dev));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

/**
 * Play against a feedback endpoint of this wMaxPacketSize. ESP-IDF rejects an
 * isochronous IN request that is not a multiple of the endpoint size, so the
 * driver has to request exactly what the descriptor advertises.
 */
static void play_following_feedback_endpoint(uint16_t fb_mps)
{
    uint16_t len = fake_usb_feedback_mps_config(fb_mps, s_cfg_buf, sizeof(s_cfg_buf));
    TEST_ASSERT_NOT_EQUAL(0, len);
    uint8_t addr = attach_config_and_install(s_cfg_buf, len);
    fake_usb_set_feedback_q16(q16_from_millihertz(48000900));   // 48.0009 samples/frame
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &k_tx_48k));
    writer_t *w = &s_writer;
    writer_start(w, dev);
    vTaskDelay(pdMS_TO_TICKS(300));
    fake_usb_clear_out_stats();
    vTaskDelay(pdMS_TO_TICKS(4000));
    fake_usb_stats_t st = fake_usb_stats();
    writer_join(w);
    TEST_ASSERT_EQUAL(ESP_OK, w->end_err);
    TEST_ASSERT_EQUAL(0, st.invalid_transfers);
    double avg = (double)st.out_bytes / 6.0 / (double)st.out_packets;
    printf("    average %.5f samples/packet (device asks 48.0009)\n", avg);
    TEST_ASSERT_TRUE(avg > 48.0004 && avg < 48.0014);
    TEST_ASSERT_EQUAL(0, st.out_pattern_breaks);
    TEST_ASSERT_EQUAL(0, dev_events(UAC2_HOST_DEVICE_EVENT_STREAM_ERROR));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(dev));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

/** The USB spec's full-speed feedback: 10.14 in a 3-byte endpoint. */
static void test_three_byte_feedback_endpoint_followed(void)
{
    play_following_feedback_endpoint(3);
}

/** An endpoint larger than the 4-byte 16.16 value it sends. */
static void test_oversized_feedback_endpoint_followed(void)
{
    play_following_feedback_endpoint(8);
}

/** A 2-byte endpoint cannot carry a feedback value: refused before claiming. */
static void test_undersized_feedback_endpoint_refused(void)
{
    uint16_t len = fake_usb_feedback_mps_config(2, s_cfg_buf, sizeof(s_cfg_buf));
    TEST_ASSERT_NOT_EQUAL(0, len);
    uint8_t addr = attach_config_and_install(s_cfg_buf, len);
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, uac2_host_device_start(dev, &k_tx_48k));
    TEST_ASSERT_EQUAL(0, fake_usb_stats().interfaces_claimed);
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void test_async_endpoint_without_feedback_refuses_44k1(void)
{
    uint16_t len = fake_usb_no_feedback_config(UAC2_EP_SYNC_ASYNC, s_cfg_buf, sizeof(s_cfg_buf));
    TEST_ASSERT_NOT_EQUAL(0, len);
    uint8_t addr = attach_config_and_install(s_cfg_buf, len);
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    const uac2_host_stream_config_t cfg = {.channels = 2, .bit_resolution = 24, .sample_freq = 44100};
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, uac2_host_device_start(dev, &cfg));
    TEST_ASSERT_EQUAL(0, fake_usb_stats().interfaces_claimed);
    // An integer rate sends whole frames, no feedback needed
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &k_tx_48k));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(dev));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void test_unadvertised_sample_rate_refused(void)
{
    // The fake miniDSP advertises 44.1 and 48 kHz only
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    uint32_t sets = fake_usb_stats().set_rate_calls;
    const uac2_host_stream_config_t cfg = {.channels = 2, .bit_resolution = 24, .sample_freq = 32000};
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, uac2_host_device_start(dev, &cfg));
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, uac2_host_device_set_sample_rate(dev, 32000));
    TEST_ASSERT_EQUAL(sets, fake_usb_stats().set_rate_calls);
    TEST_ASSERT_EQUAL(0, fake_usb_stats().interfaces_claimed);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_set_sample_rate(dev, 44100));
    TEST_ASSERT_EQUAL(44100, fake_usb_stats().last_set_rate);
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void test_rate_beyond_the_ranges_read_not_refused(void)
{
    // The driver reads 16 ranges; with 17 advertised, a rate it did not find
    // may be in the 17th, so it is sent. With 16 the list is complete.
    uint32_t rates[17];
    for (int i = 0; i < 16; i++) rates[i] = 8000 + 1000 * i;
    rates[16] = 32000;
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    fake_usb_set_clock_rates(rates, 17);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_set_sample_rate(dev, 32000));
    TEST_ASSERT_EQUAL(32000, fake_usb_stats().last_set_rate);
    fake_usb_set_clock_rates(rates, 16);
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, uac2_host_device_set_sample_rate(dev, 32000));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_set_sample_rate(dev, 23000));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void test_clock_set_before_alt_enabled(void)
{
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &k_tx_48k));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_suspend(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_resume(dev));
    fake_usb_stats_t st = fake_usb_stats();
    TEST_ASSERT_GREATER_OR_EQUAL(2, st.set_rate_calls);
    TEST_ASSERT_EQUAL(0, st.alt_enabled_without_rate);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(dev));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void test_all_channel_controls_fall_back_when_master_stalls(void)
{
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    int16_t vol = 0;
    bool muted = false;

    fake_usb_stall_ctrl(ENT_FU_PLAYBACK, CS_FU_VOLUME, 0, false);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_set_volume_all_channels(dev, -10 * 256));
    for (uint8_t ch = 1; ch <= 2; ch++) {
        TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_volume(dev, ch, &vol));
        TEST_ASSERT_EQUAL(-10 * 256, vol);
    }

    fake_usb_stall_ctrl(ENT_FU_PLAYBACK, CS_FU_MUTE, 0, false);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_set_mute_all_channels(dev, true));
    for (uint8_t ch = 1; ch <= 2; ch++) {
        TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_mute(dev, ch, &muted));
        TEST_ASSERT_TRUE(muted);
    }
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_set_mute_all_channels(dev, false));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_mute(dev, 1, &muted));
    TEST_ASSERT_FALSE(muted);

    // A failing per-channel control is still reported
    fake_usb_stall_ctrl(ENT_FU_PLAYBACK, CS_FU_MUTE, 2, false);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, uac2_host_device_set_mute_all_channels(dev, true));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void test_read_only_controls_left_alone(void)
{
    // Master mute and volume read-only (01 each); channels 1 and 2 stay host programmable
    uint16_t len = fake_usb_minidsp_config(s_cfg_buf, sizeof(s_cfg_buf));
    TEST_ASSERT_TRUE(fake_usb_config_set_fu_controls(s_cfg_buf, len, ENT_FU_PLAYBACK, 0, 0x05));
    uint8_t addr = attach_config_and_install(s_cfg_buf, len);
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    int16_t master_vol = 0;
    bool master_muted = true;
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_volume(dev, 0, &master_vol));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_mute(dev, 0, &master_muted));

    // Refused without a request on the bus
    uint32_t requests = fake_usb_stats().ctrl_requests;
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, uac2_host_device_set_volume(dev, 0, -10 * 256));
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, uac2_host_device_set_volume_percent(dev, 0, 50));
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, uac2_host_device_set_mute(dev, 0, !master_muted));
    TEST_ASSERT_EQUAL(requests, fake_usb_stats().ctrl_requests);

    // Every-channel writes skip the read-only master
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_set_volume_all_channels(dev, -10 * 256));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_set_mute_all_channels(dev, !master_muted));
    int16_t vol = 0;
    bool muted = master_muted;
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_volume(dev, 0, &vol));
    TEST_ASSERT_EQUAL(master_vol, vol);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_mute(dev, 0, &muted));
    TEST_ASSERT_EQUAL(master_muted, muted);
    for (uint8_t ch = 1; ch <= 2; ch++) {
        TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_volume(dev, ch, &vol));
        TEST_ASSERT_EQUAL(-10 * 256, vol);
        TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_mute(dev, ch, &muted));
        TEST_ASSERT_EQUAL(!master_muted, muted);
    }
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void test_all_read_only_controls_refused(void)
{
    uint16_t len = fake_usb_minidsp_config(s_cfg_buf, sizeof(s_cfg_buf));
    for (uint8_t ch = 0; ch <= 2; ch++) {
        TEST_ASSERT_TRUE(fake_usb_config_set_fu_controls(s_cfg_buf, len, ENT_FU_PLAYBACK, ch, 0x05));
    }
    uint8_t addr = attach_config_and_install(s_cfg_buf, len);
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    uint32_t requests = fake_usb_stats().ctrl_requests;
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, uac2_host_device_set_volume_all_channels(dev, -10 * 256));
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, uac2_host_device_set_mute_all_channels(dev, true));
    TEST_ASSERT_EQUAL(requests, fake_usb_stats().ctrl_requests);
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

/** An 8-byte EP0: every IN request still asks for whole packets (checked in tearDown). */
static void test_control_requests_rounded_to_small_ep0(void)
{
    fake_usb_set_ep0_mps(8);
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    uac2_sample_rate_range_t rates[UAC2_MAX_SAMPLE_RATE_RANGES];
    uint8_t n = 0;
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_sample_rate_range(dev, rates, &n));
    TEST_ASSERT_EQUAL(2, n);
    int16_t vol = 0;
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_set_volume(dev, 1, -6 * 256));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_volume(dev, 1, &vol));
    TEST_ASSERT_EQUAL(-6 * 256, vol);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &k_tx_48k));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(dev));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void test_reversed_volume_range_not_used(void)
{
    fake_usb_set_volume_range(0, -90 * 256, 256);   // max below min
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    uint8_t pct = 0;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_RESPONSE, uac2_host_device_set_volume_percent(dev, 0, 50));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_RESPONSE, uac2_host_device_get_volume_percent(dev, 0, &pct));
    int16_t vol = 1;
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_volume(dev, 0, &vol));
    TEST_ASSERT_EQUAL(0, vol);   // the percent setter sent nothing
    // Nor does a raw dB value, which cannot be checked against the range
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_RESPONSE, uac2_host_device_set_volume(dev, 1, -20 * 256));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_volume(dev, 1, &vol));
    TEST_ASSERT_EQUAL(0, vol);
    // The raw ranges are reported as the device sent them, without being cached
    uac2_volume_range_t ranges[UAC2_MAX_VOLUME_RANGES];
    uint8_t n = 0;
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_volume_range(dev, 0, ranges, &n));
    TEST_ASSERT_EQUAL(1, n);
    TEST_ASSERT_EQUAL(-90 * 256, ranges[0].max);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_RESPONSE, uac2_host_device_set_volume_percent(dev, 0, 50));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void test_ring_buffer_sizes_checked(void)
{
    uint8_t addr = attach_and_install();
    // Smaller than one 294-byte packet: refused before anything is claimed
    uac2_host_device_handle_t dev = open_iface_buffered(addr, TX_IFACE, 100, 0);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, uac2_host_device_start(dev, &k_tx_48k));
    TEST_ASSERT_EQUAL(0, fake_usb_stats().interfaces_claimed);
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    dev = open_iface_buffered(addr, RX_IFACE, 100, 0);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, uac2_host_device_start(dev, &k_rx_48k));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    // A threshold the ring can never reach
    dev = open_iface_buffered(addr, TX_IFACE, 1000, 2000);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, uac2_host_device_start(dev, &k_tx_48k));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    // A write larger than the ring fails at once instead of timing out
    dev = open_iface_buffered(addr, TX_IFACE, 1000, 0);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &k_tx_48k));
    static uint8_t pcm[1001];
    int64_t t0 = esp_timer_get_time();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, uac2_host_device_write(dev, pcm, sizeof(pcm), 1000));
    TEST_ASSERT_LESS_THAN(100000, esp_timer_get_time() - t0);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_write(dev, pcm, 900, 1000));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(dev));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

/** A USB-MIDI function's UAC1 AC interface after the audio function is not used for requests. */
static void test_trailing_midi_function_left_alone(void)
{
    uint16_t len = fake_usb_midi_function_config(s_cfg_buf, sizeof(s_cfg_buf));
    TEST_ASSERT_NOT_EQUAL(0, len);
    uint8_t addr = attach_config_and_install(s_cfg_buf, len);
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    uac2_device_info_t info;
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_info(dev, &info));
    TEST_ASSERT_EQUAL(0, info.ac_iface_num);
    TEST_ASSERT_EQUAL(3, info.num_as_ifaces);
    int16_t vol = 0;
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_set_volume(dev, 1, -3 * 256));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_volume(dev, 1, &vol));
    TEST_ASSERT_EQUAL(-3 * 256, vol);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &k_tx_48k));
    TEST_ASSERT_EQUAL(48000, fake_usb_stats().last_set_rate);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(dev));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void test_start_falls_back_to_an_alt_the_host_can_fit(void)
{
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    uac2_host_stream_info_t info;

    // The host cannot fit the 24-bit alt's 294-byte endpoint; the 16-bit alt's 196 fits
    fake_usb_set_max_iso_out_mps(200);
    const uac2_host_stream_config_t best = {.channels = 2, .bit_resolution = 0, .sample_freq = 48000};
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &best));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_stream_info(dev, &info));
    TEST_ASSERT_EQUAL(2, info.alt_setting);
    TEST_ASSERT_EQUAL(16, info.bit_resolution);
    TEST_ASSERT_EQUAL(2, info.sub_slot_size);
    TEST_ASSERT_EQUAL(196, info.ep_max_packet_size);
    TEST_ASSERT_TRUE(info.active);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(dev));

    // An explicit 24-bit request has no other alt to fall back to
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_SUPPORTED, uac2_host_device_start(dev, &k_tx_48k));
    TEST_ASSERT_EQUAL(0, fake_usb_stats().interfaces_claimed);

    fake_usb_set_max_iso_out_mps(0);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &k_tx_48k));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_stream_info(dev, &info));
    TEST_ASSERT_EQUAL(1, info.alt_setting);
    TEST_ASSERT_EQUAL(24, info.bit_resolution);
    TEST_ASSERT_EQUAL(3, info.sub_slot_size);
    TEST_ASSERT_EQUAL(48000, info.sample_freq);
    TEST_ASSERT_EQUAL(0x81, info.fb_ep_addr);
    TEST_ASSERT_EQUAL(UAC2_EP_SYNC_ASYNC, info.sync_type);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_suspend(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_stream_info(dev, &info));
    TEST_ASSERT_FALSE(info.active);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(dev));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, uac2_host_device_get_stream_info(dev, &info));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static bool s_ring_empty_dev_ok;
static uac2_host_device_handle_t s_status_dev;
static bool ring_drained(void)
{
    uac2_host_buffer_status_t bs;
    s_ring_empty_dev_ok = uac2_host_device_get_buffer_status(s_status_dev, &bs) == ESP_OK;
    return s_ring_empty_dev_ok && bs.ring_bytes == 0;
}

static void test_buffer_status_tracks_queued_audio(void)
{
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &k_tx_48k));
    uac2_host_buffer_status_t bs;
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_buffer_status(dev, &bs));
    TEST_ASSERT_EQUAL(28800, bs.ring_capacity);     // default: 100 packets

    static uint8_t chunk[14400];                    // 50 ms at 48 kHz 24-bit stereo
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_write(dev, chunk, sizeof(chunk), 100));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_buffer_status(dev, &bs));
    printf("    queued %u bytes (%u us), %u us in flight\n",
           (unsigned)bs.ring_bytes, (unsigned)bs.ring_us, (unsigned)bs.in_flight_us);
    TEST_ASSERT_TRUE(bs.ring_bytes > sizeof(chunk) / 2 && bs.ring_bytes <= sizeof(chunk));
    TEST_ASSERT_UINT32_WITHIN(1, (uint32_t)((uint64_t)bs.ring_bytes * 1000000 / 288000), bs.ring_us);
    TEST_ASSERT_TRUE(bs.in_flight_us > 0 && bs.in_flight_us <= 9000);

    // Drains at 288 bytes per 1 ms frame
    s_status_dev = dev;
    TEST_ASSERT_TRUE(wait_until(ring_drained, 500));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(dev));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, uac2_host_device_get_buffer_status(dev, &bs));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

// ── Playback and capture together (CONFIG_UAC2_ALLOW_DUPLEX on ESP32-S3) ──
// The S3-only opposite-direction guard is not compiled for the Linux target,
// so these cover the driver's own handling of two streams at once.

/**
 * Read the capture stream for duration_ms; returns the bytes captured. The
 * fake's capture data is a running byte counter, so *breaks counts places
 * where a byte is not the previous one + 1 (data lost, repeated, misplaced).
 */
static uint64_t drain_capture(uac2_host_device_handle_t rx, int duration_ms, int *breaks)
{
    static uint8_t buf[1024];
    uint64_t total = 0;
    bool have_prev = false;
    uint8_t prev = 0;
    if (breaks) *breaks = 0;
    // Timed in ticks, as the fake's bus is (one frame per tick): a loaded host
    // drops POSIX-port ticks, and a wall-clock window would then see fewer frames
    TickType_t start = xTaskGetTickCount();
    while (xTaskGetTickCount() - start < pdMS_TO_TICKS(duration_ms)) {
        uint32_t n = 0;
        esp_err_t err = uac2_host_device_read(rx, buf, sizeof(buf), &n, 10);
        if (err == ESP_OK) {
            for (uint32_t i = 0; i < n; i++) {
                if (have_prev && buf[i] != (uint8_t)(prev + 1) && breaks) (*breaks)++;
                prev = buf[i];
                have_prev = true;
            }
            total += n;
        } else if (err != ESP_ERR_TIMEOUT) {
            break;
        }
    }
    return total;
}

static void test_buffer_status_counts_underruns_and_drops(void)
{
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t dev = open_iface(addr, TX_IFACE);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &k_tx_48k));
    uac2_host_buffer_status_t bs;
    vTaskDelay(pdMS_TO_TICKS(50));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_buffer_status(dev, &bs));
    TEST_ASSERT_EQUAL(0, bs.underrun_bytes);    // start-up silence, nothing written yet
    writer_t *w = &s_writer;
    writer_start(w, dev);
    vTaskDelay(pdMS_TO_TICKS(500));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_buffer_status(dev, &bs));
    TEST_ASSERT_EQUAL(0, bs.underrun_bytes);    // the writer keeps up
    writer_join(w);
    TEST_ASSERT_EQUAL(ESP_OK, w->end_err);
    vTaskDelay(pdMS_TO_TICKS(300));             // the ~100 ms ring drains, then silence
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_buffer_status(dev, &bs));
    printf("    %u underrun bytes after the writer stopped\n", (unsigned)bs.underrun_bytes);
    TEST_ASSERT_GREATER_THAN(288 * 100, bs.underrun_bytes);
    TEST_ASSERT_EQUAL(0, bs.dropped_bytes);
    // The counters restart with the stream, and a resumed or restarted
    // stream's start-up silence is not an underrun
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_suspend(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_resume(dev));
    vTaskDelay(pdMS_TO_TICKS(50));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_buffer_status(dev, &bs));
    TEST_ASSERT_EQUAL(0, bs.underrun_bytes);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &k_tx_48k));
    vTaskDelay(pdMS_TO_TICKS(50));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_buffer_status(dev, &bs));
    TEST_ASSERT_EQUAL(0, bs.underrun_bytes);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(dev));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));

    // Capture nobody reads: the ring fills, then packets are dropped
    dev = open_iface(addr, RX_IFACE);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &k_rx_48k));
    vTaskDelay(pdMS_TO_TICKS(300));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_buffer_status(dev, &bs));
    printf("    %u capture bytes dropped\n", (unsigned)bs.dropped_bytes);
    TEST_ASSERT_GREATER_THAN(288 * 100, bs.dropped_bytes);
    TEST_ASSERT_EQUAL(0, bs.underrun_bytes);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(dev, &k_rx_48k));
    vTaskDelay(pdMS_TO_TICKS(30));     // well inside the ~100 ms ring
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_get_buffer_status(dev, &bs));
    TEST_ASSERT_EQUAL(0, bs.dropped_bytes);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(dev));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(dev));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void test_playback_and_capture_together(void)
{
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t tx = open_iface(addr, TX_IFACE);
    uac2_host_device_handle_t rx = open_iface(addr, RX_IFACE);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(tx, &k_tx_48k));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(rx, &k_rx_48k));
    writer_t *w = &s_writer;
    writer_start(w, tx);
    fake_usb_clear_out_stats();
    int breaks = -1;
    uint64_t captured = drain_capture(rx, 1000, &breaks);
    fake_usb_stats_t st = fake_usb_stats();
    printf("    1 s: %llu bytes captured (%d breaks in the counter pattern), %llu playback packets\n",
           (unsigned long long)captured, breaks, (unsigned long long)st.out_packets);
    // 288 bytes per 1 ms frame each way; every captured byte in order
    TEST_ASSERT_UINT64_WITHIN(288 * 150, 288 * 1000, captured);
    TEST_ASSERT_EQUAL(0, breaks);
    TEST_ASSERT_UINT64_WITHIN(150, 1000, st.out_packets);
    TEST_ASSERT_EQUAL(288, st.out_min_packet);
    TEST_ASSERT_GREATER_THAN(0, st.out_nonzero_bytes);
    TEST_ASSERT_EQUAL(0, st.out_pattern_breaks);

    // Stopping capture leaves playback running, and capture can start again
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(rx));
    fake_usb_clear_out_stats();
    vTaskDelay(pdMS_TO_TICKS(300));
    TEST_ASSERT_GREATER_THAN(200, fake_usb_stats().out_packets);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(rx, &k_rx_48k));
    TEST_ASSERT_GREATER_THAN(288 * 100, drain_capture(rx, 300, &breaks));
    TEST_ASSERT_EQUAL(0, breaks);

    writer_join(w);
    TEST_ASSERT_EQUAL(ESP_OK, w->end_err);
    TEST_ASSERT_EQUAL(0, dev_events(UAC2_HOST_DEVICE_EVENT_STREAM_ERROR));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(rx));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(tx));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(rx));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(tx));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void test_close_capture_while_playback_runs(void)
{
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t tx = open_iface(addr, TX_IFACE);
    uac2_host_device_handle_t rx = open_iface(addr, RX_IFACE);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(tx, &k_tx_48k));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(rx, &k_rx_48k));
    writer_t *w = &s_writer;
    writer_start(w, tx);
    vTaskDelay(pdMS_TO_TICKS(200));
    // Close without stop: the capture stream is torn down, the device stays open
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(rx));
    TEST_ASSERT_EQUAL(1, fake_usb_stats().device_open_count);
    fake_usb_clear_out_stats();
    vTaskDelay(pdMS_TO_TICKS(300));
    TEST_ASSERT_GREATER_THAN(200, fake_usb_stats().out_packets);
    writer_join(w);
    TEST_ASSERT_EQUAL(ESP_OK, w->end_err);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_stop(tx));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(tx));
    TEST_ASSERT_TRUE(wait_until(device_closed, 1000));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void test_unplug_during_playback_and_capture(void)
{
    uint8_t addr = attach_and_install();
    uac2_host_device_handle_t tx = open_iface(addr, TX_IFACE);
    uac2_host_device_handle_t rx = open_iface(addr, RX_IFACE);
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(tx, &k_tx_48k));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_device_start(rx, &k_rx_48k));
    writer_t *w = &s_writer;
    writer_start(w, tx);
    TEST_ASSERT_GREATER_THAN(0, drain_capture(rx, 200, NULL));
    fake_usb_detach();
    writer_wait_end(w, 2000);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, w->end_err);
    uint32_t n = 0;
    static uint8_t buf[64];
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, uac2_host_device_read(rx, buf, sizeof(buf), &n, 100));
    // One DISCONNECTED per open interface
    TEST_ASSERT_EQUAL(2, dev_events(UAC2_HOST_DEVICE_EVENT_DISCONNECTED));
    TEST_ASSERT_EQUAL(0, dev_events(UAC2_HOST_DEVICE_EVENT_STREAM_ERROR));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(rx));
    TEST_ASSERT_EQUAL(ESP_OK, close_iface(tx));
    TEST_ASSERT_TRUE(wait_until(device_freed, 1000));
    TEST_ASSERT_EQUAL(ESP_OK, uac2_host_uninstall());
}

static void run_all_tests(void *arg)
{
    (void)arg;
    UNITY_BEGIN();
    RUN_TEST(test_attached_device_reported_by_install_scan);
    RUN_TEST(test_device_attached_after_install_reported);
    RUN_TEST(test_reinstall_with_device_attached);
    RUN_TEST(test_uninstall_refused_while_interface_open);
    RUN_TEST(test_malformed_configuration_not_reported);
    RUN_TEST(test_concurrent_uninstalls);
    RUN_TEST(test_manual_pump_uninstall_from_pump_task);
    RUN_TEST(test_manual_pump_uninstall_from_other_task);
    RUN_TEST(test_playback_48k_sends_whole_288_byte_packets);
    RUN_TEST(test_playback_follows_feedback_drift);
    RUN_TEST(test_suspend_and_resume);
    RUN_TEST(test_teardown_after_channel_error);
    RUN_TEST(test_stop_waits_for_a_callback_mid_resubmit);
    RUN_TEST(test_restart_stops_running_streams);
    RUN_TEST(test_io_after_close_is_invalid_state);
    RUN_TEST(test_unplug_during_playback_then_replug);
    RUN_TEST(test_unplug_while_idle_open);
    RUN_TEST(test_start_after_unplug_before_close_refused);
    RUN_TEST(test_failed_urb_allocation_rolls_back);
    RUN_TEST(test_control_timeout_orphan_then_late_reply);
    RUN_TEST(test_control_timeout_orphan_reclaimed_on_unplug);
    RUN_TEST(test_start_reports_clock_setup_timeout);
    RUN_TEST(test_late_stall_then_request_waits_for_ep0);
    RUN_TEST(test_late_reply_consumed_by_next_request);
    RUN_TEST(test_interface_release_retried_while_busy);
    RUN_TEST(test_dead_data_endpoint_reaches_stream_error);
    RUN_TEST(test_skipped_feedback_is_not_fatal);
    RUN_TEST(test_close_active_stream_without_stop);
    RUN_TEST(test_close_while_writer_running);
    RUN_TEST(test_set_sample_rate_refused_while_streaming);
    RUN_TEST(test_sibling_stream_at_other_rate_refused);
    RUN_TEST(test_clock_never_changes_under_a_stream);
    RUN_TEST(test_adaptive_endpoint_plays_44k1_without_feedback);
    RUN_TEST(test_three_byte_feedback_endpoint_followed);
    RUN_TEST(test_oversized_feedback_endpoint_followed);
    RUN_TEST(test_undersized_feedback_endpoint_refused);
    RUN_TEST(test_async_endpoint_without_feedback_refuses_44k1);
    RUN_TEST(test_unadvertised_sample_rate_refused);
    RUN_TEST(test_rate_beyond_the_ranges_read_not_refused);
    RUN_TEST(test_clock_set_before_alt_enabled);
    RUN_TEST(test_all_channel_controls_fall_back_when_master_stalls);
    RUN_TEST(test_read_only_controls_left_alone);
    RUN_TEST(test_all_read_only_controls_refused);
    RUN_TEST(test_control_requests_rounded_to_small_ep0);
    RUN_TEST(test_trailing_midi_function_left_alone);
    RUN_TEST(test_reversed_volume_range_not_used);
    RUN_TEST(test_ring_buffer_sizes_checked);
    RUN_TEST(test_start_falls_back_to_an_alt_the_host_can_fit);
    RUN_TEST(test_buffer_status_tracks_queued_audio);
    RUN_TEST(test_buffer_status_counts_underruns_and_drops);
    RUN_TEST(test_playback_and_capture_together);
    RUN_TEST(test_close_capture_while_playback_runs);
    RUN_TEST(test_unplug_during_playback_and_capture);
    int failures = UNITY_END();
    exit(failures ? 1 : 0);
}

void app_main(void)
{
    if (esp_register_shutdown_handler(unexpected_restart) != ESP_OK) exit(1);
    xTaskCreate(watchdog_task, "watchdog", 4096, NULL, configMAX_PRIORITIES - 1, NULL);
    // Above the writer/hammer tasks (6) and the driver's event task (5), so a
    // test can always run its teardown; below the fake bus (23)
    xTaskCreate(run_all_tests, "tests", 32768, NULL, 10, NULL);
}
