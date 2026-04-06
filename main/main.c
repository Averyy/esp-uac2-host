/*
 * SPDX-FileCopyrightText: 2026 Avery Levitt
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * esp-uac2-host — Comprehensive UAC2 driver test suite
 *
 * Uses the install/uninstall lifecycle — driver discovers devices internally,
 * fires callbacks, app opens interfaces and runs tests.
 *
 * Tests:
 *   1.  Basic 48kHz/24-bit streaming (10s)
 *   2.  Volume/mute control during streaming
 *   3.  Stop/restart cycle
 *   4.  44.1kHz sample rate switch
 *   5.  Start time precision (multiple iterations)
 *   6.  Feedback convergence & clock validity monitoring
 *   7.  16-bit alt setting
 *   8.  Rapid measurement cycles (9x stop/start)
 *   9.  Volume range & channel exploration
 *  10.  Sample rate switch stress
 *  11.  Ring buffer starvation/recovery
 *  12.  Long-running stability
 */

#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "usb/usb_host.h"
#include "usb/uac2_desc.h"
#include "usb/uac2_host.h"
#include "esp_heap_caps.h"
#include "tone_gen.h"

// Static miniDSP descriptor dump for self-test
#include "minidsp_2x4hd_descriptors.h"

#define HOST_LIB_TASK_PRIORITY  2
#define UAC2_TASK_PRIORITY      3
#define UAC2_TASK_STACK_SIZE    (6 * 1024)
#define DEV_TASK_STACK_SIZE     (12 * 1024)

// Tone test config
#define TONE_SAMPLE_RATE    48000
#define TONE_CHANNELS       2
#define TONE_BIT_DEPTH      24
#define TONE_FREQ_HZ        1000.0f
#define TONE_AMPLITUDE      0.5f
#define TONE_BUF_MS         10
#define TONE_BUF_SIZE       ((TONE_SAMPLE_RATE / 1000) * TONE_BUF_MS * TONE_CHANNELS * (TONE_BIT_DEPTH / 8))

static const char *TAG = "uac2-host";

// ── App state ────────────────────────────────────────────────────

typedef struct {
    uac2_host_device_handle_t uac2_dev;
    uint8_t dev_addr;
    uint8_t iface_num;
    volatile bool dev_connected;
    volatile TaskHandle_t dev_task_hdl;
    uint32_t connect_cycle;          // disconnect/reconnect cycle counter
    size_t heap_baseline;            // free heap at first connect (for leak detection)
} app_state_t;

static app_state_t s_app;

// ── Event tracking ───────────────────────────────────────────────

static volatile uint32_t evt_tx_done = 0;
static volatile uint32_t evt_errors = 0;
static volatile uint32_t evt_disconnects = 0;

static void reset_event_counters(void)
{
    evt_tx_done = 0;
    evt_errors = 0;
    evt_disconnects = 0;
}

static void log_event_counters(const char *label)
{
    ESP_LOGI(TAG, "Events [%s]: TX_DONE=%" PRIu32 " ERRORS=%" PRIu32
             " DISCONNECTS=%" PRIu32, label, evt_tx_done, evt_errors, evt_disconnects);
}

// ── Self-test: parse static miniDSP descriptors ──────────────────

static void run_descriptor_self_test(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " Self-test: miniDSP 2x4 HD descriptors");
    ESP_LOGI(TAG, "========================================");

    ESP_LOGI(TAG, "Device: VID=0x%02X%02X PID=0x%02X%02X",
             minidsp_2x4hd_device_desc[9], minidsp_2x4hd_device_desc[8],
             minidsp_2x4hd_device_desc[11], minidsp_2x4hd_device_desc[10]);

    uac2_device_info_t info;
    bool is_uac2 = uac2_parse_config_descriptor(
        minidsp_2x4hd_config1_desc,
        sizeof(minidsp_2x4hd_config1_desc),
        &info);

    if (is_uac2) {
        ESP_LOGI(TAG, "PASS: UAC2 detected (bcdADC=0x%04X)", info.bcdADC);
        uac2_log_device_info(&info);

        bool pass = true;
        if (info.num_clock_sources != 1 || info.clock_sources[0].clock_id != 41) {
            ESP_LOGE(TAG, "FAIL: Expected clock source ID=41, got %d sources",
                     info.num_clock_sources);
            pass = false;
        }
        if (info.num_clock_selectors != 1 || info.clock_selectors[0].clock_id != 40) {
            ESP_LOGE(TAG, "FAIL: Expected clock selector ID=40");
            pass = false;
        }
        if (info.num_terminals < 4) {
            ESP_LOGE(TAG, "FAIL: Expected at least 4 terminals, got %d",
                     info.num_terminals);
            pass = false;
        }
        if (info.num_as_ifaces < 1) {
            ESP_LOGE(TAG, "FAIL: Expected at least 1 AS interface, got %d",
                     info.num_as_ifaces);
            pass = false;
        }
        if (info.num_as_ifaces >= 1) {
            const uac2_as_iface_t *as = &info.as_ifaces[0];
            if (as->bit_resolution != 24 || as->sub_slot_size != 3) {
                ESP_LOGE(TAG, "FAIL: Alt 1 expected 24-bit/3-byte, got %d-bit/%d-byte",
                         as->bit_resolution, as->sub_slot_size);
                pass = false;
            }
            if (as->ep_max_packet_size != 294) {
                ESP_LOGE(TAG, "FAIL: Alt 1 expected MaxPkt=294, got %d",
                         as->ep_max_packet_size);
                pass = false;
            }
            if (as->fb_ep_addr != 0x81) {
                ESP_LOGE(TAG, "FAIL: Expected feedback EP 0x81, got 0x%02X",
                         as->fb_ep_addr);
                pass = false;
            }
        }

        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, " Self-test %s", pass ? "PASSED" : "FAILED");
        ESP_LOGI(TAG, "========================================");
    } else {
        ESP_LOGE(TAG, "FAIL: UAC2 not detected in miniDSP descriptors!");
    }
    ESP_LOGI(TAG, "");
}

// ── Callbacks ────────────────────────────────────────────────────

static void device_event_cb(uac2_host_device_handle_t dev,
                            const uac2_host_device_event_t event, void *arg)
{
    switch (event) {
    case UAC2_HOST_DEVICE_EVENT_TX_DONE:
        evt_tx_done++;
        break;
    case UAC2_HOST_DEVICE_EVENT_RX_DONE:
        break;
    case UAC2_HOST_DEVICE_EVENT_TRANSFER_ERROR:
        evt_errors++;
        ESP_LOGW(TAG, "UAC2: Transfer error (#%" PRIu32 ")", evt_errors);
        break;
    case UAC2_HOST_DEVICE_EVENT_DISCONNECTED:
        evt_disconnects++;
        s_app.dev_connected = false;
        ESP_LOGW(TAG, "UAC2: Disconnected");
        break;
    case UAC2_HOST_DEVICE_EVENT_STREAM_ERROR:
        ESP_LOGE(TAG, "UAC2: Stream dead (max consecutive errors)");
        break;
    }
}

// ── Parsed UAC2 device info logging ──────────────────────────────

static void log_all_device_info(uac2_host_device_handle_t uac2_dev)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " PARSED UAC2 DEVICE INFO");
    ESP_LOGI(TAG, "========================================");

    uac2_device_info_t info;
    esp_err_t err = uac2_host_device_get_info(uac2_dev, &info);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get device info: %s", esp_err_to_name(err));
        return;
    }

    uac2_log_device_info(&info);

    ESP_LOGI(TAG, "--- Clock topology detail ---");
    for (int i = 0; i < info.num_clock_sources; i++) {
        ESP_LOGI(TAG, "  ClockSrc[%d]: id=%d attr=0x%02X ctrl=0x%02X assoc=%d",
                 i, info.clock_sources[i].clock_id,
                 info.clock_sources[i].attributes,
                 info.clock_sources[i].controls,
                 info.clock_sources[i].assoc_terminal);
    }
    for (int i = 0; i < info.num_clock_selectors; i++) {
        ESP_LOGI(TAG, "  ClockSel[%d]: id=%d nr_pins=%d src=[%d,%d,%d,%d]",
                 i, info.clock_selectors[i].clock_id,
                 info.clock_selectors[i].nr_pins,
                 info.clock_selectors[i].source_ids[0],
                 info.clock_selectors[i].source_ids[1],
                 info.clock_selectors[i].source_ids[2],
                 info.clock_selectors[i].source_ids[3]);
    }

    ESP_LOGI(TAG, "--- Terminals ---");
    for (int i = 0; i < info.num_terminals; i++) {
        ESP_LOGI(TAG, "  T[%d]: id=%d type=0x%04X(%s) clk=%d ch=%d %s src=%d",
                 i, info.terminals[i].terminal_id,
                 info.terminals[i].terminal_type,
                 uac2_terminal_type_str(info.terminals[i].terminal_type),
                 info.terminals[i].clock_source_id,
                 info.terminals[i].nr_channels,
                 info.terminals[i].is_input ? "IN" : "OUT",
                 info.terminals[i].source_id);
    }

    ESP_LOGI(TAG, "--- Feature units ---");
    for (int i = 0; i < info.num_feature_units; i++) {
        ESP_LOGI(TAG, "  FU[%d]: id=%d src=%d ch=%d",
                 i, info.feature_units[i].unit_id,
                 info.feature_units[i].source_id,
                 info.feature_units[i].nr_channels);
    }

    ESP_LOGI(TAG, "--- AS interfaces ---");
    for (int i = 0; i < info.num_as_ifaces; i++) {
        const uac2_as_iface_t *as = &info.as_ifaces[i];
        ESP_LOGI(TAG, "  AS[%d]: iface=%d alt=%d link=%d fmt=%d ch=%d sub=%d bit=%d",
                 i, as->interface_num, as->alt_setting, as->terminal_link,
                 as->format_type, as->nr_channels, as->sub_slot_size, as->bit_resolution);
        ESP_LOGI(TAG, "         ep=0x%02X attr=0x%02X mps=%d interval=%d",
                 as->ep_addr, as->ep_attributes, as->ep_max_packet_size, as->ep_interval);
        if (as->fb_ep_addr) {
            ESP_LOGI(TAG, "         fb=0x%02X fb_mps=%d fb_interval=%d",
                     as->fb_ep_addr, as->fb_ep_max_packet_size, as->fb_ep_interval);
        }
    }
}

static void log_clock_info(uac2_host_device_handle_t uac2_dev)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "--- Clock info ---");

    uint32_t sample_rate = 0;
    esp_err_t err = uac2_host_device_get_sample_rate(uac2_dev, &sample_rate);
    ESP_LOGI(TAG, "Current sample rate: %" PRIu32 " Hz (%s)",
             sample_rate, esp_err_to_name(err));

    uac2_sample_rate_range_t ranges[UAC2_MAX_SAMPLE_RATE_RANGES];
    uint8_t num_ranges = 0;
    err = uac2_host_device_get_sample_rate_range(uac2_dev, ranges, &num_ranges);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Sample rate ranges: %d", num_ranges);
        for (int i = 0; i < num_ranges; i++) {
            ESP_LOGI(TAG, "  [%d] min=%" PRIu32 " max=%" PRIu32 " res=%" PRIu32,
                     i, ranges[i].min, ranges[i].max, ranges[i].res);
        }
    }

    bool clock_valid = false;
    err = uac2_host_device_get_clock_valid(uac2_dev, &clock_valid);
    ESP_LOGI(TAG, "Clock valid: %s (%s)", clock_valid ? "yes" : "no", esp_err_to_name(err));
}

static void log_volume_info(uac2_host_device_handle_t uac2_dev)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "--- Volume/Mute info ---");

    uac2_volume_range_t vranges[UAC2_MAX_VOLUME_RANGES];
    uint8_t num_vranges = 0;
    esp_err_t err = uac2_host_device_get_volume_range(uac2_dev, 0, vranges, &num_vranges);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Volume ranges (master ch0): %d", num_vranges);
        for (int i = 0; i < num_vranges; i++) {
            ESP_LOGI(TAG, "  [%d] min=%.2f dB  max=%.2f dB  res=%.4f dB",
                     i, vranges[i].min / 256.0, vranges[i].max / 256.0,
                     vranges[i].res / 256.0);
        }
    }

    for (int ch = 1; ch <= 2; ch++) {
        err = uac2_host_device_get_volume_range(uac2_dev, ch, vranges, &num_vranges);
        if (err == ESP_OK && num_vranges > 0) {
            ESP_LOGI(TAG, "Volume range (ch%d): min=%.2f max=%.2f res=%.4f dB",
                     ch, vranges[0].min / 256.0, vranges[0].max / 256.0,
                     vranges[0].res / 256.0);
        }
    }

    for (int ch = 0; ch <= 2; ch++) {
        int16_t vol = 0;
        err = uac2_host_device_get_volume(uac2_dev, ch, &vol);
        ESP_LOGI(TAG, "Volume ch%d: %d raw (%.2f dB) [%s]",
                 ch, vol, vol / 256.0, esp_err_to_name(err));
    }

    for (int ch = 0; ch <= 2; ch++) {
        bool muted = false;
        err = uac2_host_device_get_mute(uac2_dev, ch, &muted);
        ESP_LOGI(TAG, "Mute ch%d: %s [%s]",
                 ch, muted ? "MUTED" : "unmuted", esp_err_to_name(err));
    }
}

// ── Streaming helper ─────────────────────────────────────────────

static int stream_tone(app_state_t *app, uint32_t sample_rate,
                       uint8_t bit_depth, float freq_hz, int duration_sec)
{
    uac2_host_stream_config_t stream_cfg = {
        .sample_freq = sample_rate,
        .channels = TONE_CHANNELS,
        .bit_resolution = bit_depth,
    };

    reset_event_counters();
    int64_t t0 = esp_timer_get_time();
    esp_err_t err = uac2_host_device_start(app->uac2_dev, &stream_cfg);
    int64_t t1 = esp_timer_get_time();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Stream start failed: %s (%" PRId64 " us)",
                 esp_err_to_name(err), t1 - t0);
        return 0;
    }

    int64_t start_time = uac2_host_device_get_start_time(app->uac2_dev);
    ESP_LOGI(TAG, "stream_start: %" PRId64 " us, first_frame=%" PRId64 " us",
             t1 - t0, start_time);

    uint32_t bytes_per_sample = bit_depth / 8;
    uint32_t bytes_per_ms = (sample_rate / 1000) * TONE_CHANNELS * bytes_per_sample;
    uint32_t buf_size = bytes_per_ms * TONE_BUF_MS;

    ESP_LOGI(TAG, "Streaming %d Hz @ %" PRIu32 " Hz/%d-bit for %d sec (%" PRIu32 " B/ms)",
             (int)freq_hz, sample_rate, bit_depth, duration_sec, bytes_per_ms);

    tone_gen_t gen;
    tone_gen_config_t tone_cfg = {
        .sample_rate = sample_rate,
        .channels = TONE_CHANNELS,
        .bit_depth = bit_depth,
        .frequency = freq_hz,
        .amplitude = TONE_AMPLITUDE,
    };
    tone_gen_init(&gen, &tone_cfg);

    uint8_t tone_buf[TONE_BUF_SIZE];
    if (buf_size > TONE_BUF_SIZE) {
        ESP_LOGE(TAG, "Sample rate too high for tone buffer");
        uac2_host_device_stop(app->uac2_dev);
        return 0;
    }

    uint32_t writes = 0;
    uint32_t writes_per_sec = 1000 / TONE_BUF_MS;
    uint32_t target_writes = (uint32_t)duration_sec * writes_per_sec;

    // Pre-fill ring buffer (~50ms)
    for (int i = 0; i < 50 / TONE_BUF_MS && app->dev_connected; i++) {
        tone_gen_fill(&gen, tone_buf, buf_size);
        uac2_host_device_write(app->uac2_dev, tone_buf, buf_size, 100);
    }

    while (app->dev_connected && writes < target_writes) {
        tone_gen_fill(&gen, tone_buf, buf_size);
        esp_err_t wr = uac2_host_device_write(app->uac2_dev, tone_buf, buf_size, 100);
        if (wr == ESP_ERR_INVALID_STATE) break;
        if (wr == ESP_OK) {
            writes++;
            if (writes % writes_per_sec == 0) {
                ESP_LOGI(TAG, "  %" PRIu32 " sec", writes / writes_per_sec);
            }
        }
    }

    int seconds = (int)(writes / writes_per_sec);

    t0 = esp_timer_get_time();
    uac2_host_device_stop(app->uac2_dev);
    t1 = esp_timer_get_time();
    ESP_LOGI(TAG, "Stream stopped after %d sec (stop took %" PRId64 " us)", seconds, t1 - t0);
    log_event_counters("stream_tone");
    return seconds;
}

// ── Test suite ───────────────────────────────────────────────────

static void run_stream_tests(app_state_t *app)
{
    esp_err_t err;
    int sec;

    // ── Test 1: Basic 48kHz/24-bit streaming (10s) ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 1: Basic 48kHz/24-bit streaming (10 sec) ===");
    sec = stream_tone(app, 48000, 24, 1000.0f, 10);
    if (!app->dev_connected) return;
    ESP_LOGI(TAG, "=== TEST 1: %s (%d sec) ===", sec >= 10 ? "PASS" : "FAIL", sec);

    // ── Test 2: Volume/mute control during streaming ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 2: Volume/mute control during streaming ===");
    {
        uac2_host_stream_config_t scfg = {
            .sample_freq = TONE_SAMPLE_RATE,
            .channels = TONE_CHANNELS,
            .bit_resolution = TONE_BIT_DEPTH,
        };
        err = uac2_host_device_start(app->uac2_dev, &scfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "=== TEST 2: FAIL (stream start: %s) ===", esp_err_to_name(err));
        } else {
            tone_gen_t gen;
            tone_gen_config_t tcfg = {
                .sample_rate = TONE_SAMPLE_RATE, .channels = TONE_CHANNELS,
                .bit_depth = TONE_BIT_DEPTH, .frequency = TONE_FREQ_HZ,
                .amplitude = TONE_AMPLITUDE,
            };
            tone_gen_init(&gen, &tcfg);

            uint8_t tbuf[TONE_BUF_SIZE];
            uint32_t bsz = (TONE_SAMPLE_RATE / 1000) * TONE_BUF_MS * TONE_CHANNELS * (TONE_BIT_DEPTH / 8);
            bool t2_pass = true;

            for (int i = 0; i < 50 / TONE_BUF_MS && app->dev_connected; i++) {
                tone_gen_fill(&gen, tbuf, bsz);
                uac2_host_device_write(app->uac2_dev, tbuf, bsz, 100);
            }

            uint32_t writes = 0;
            uint32_t wps = 1000 / TONE_BUF_MS;
            while (app->dev_connected && writes < 3 * wps) {
                tone_gen_fill(&gen, tbuf, bsz);
                esp_err_t wr = uac2_host_device_write(app->uac2_dev, tbuf, bsz, 100);
                if (wr == ESP_ERR_INVALID_STATE) break;
                if (wr == ESP_OK) writes++;

                if (writes == wps / 2) {
                    err = uac2_host_device_set_mute(app->uac2_dev, 0, true);
                    ESP_LOGI(TAG, "  Set mute=true: %s", esp_err_to_name(err));
                    if (err != ESP_OK) t2_pass = false;
                }
                if (writes == wps) {
                    err = uac2_host_device_set_mute(app->uac2_dev, 0, false);
                    ESP_LOGI(TAG, "  Set mute=false: %s", esp_err_to_name(err));
                    if (err != ESP_OK) t2_pass = false;
                    err = uac2_host_device_set_volume(app->uac2_dev, 0, -12 * 256);
                    ESP_LOGI(TAG, "  Set volume=-12dB: %s", esp_err_to_name(err));
                    if (err != ESP_OK) t2_pass = false;
                }
                if (writes == wps + wps / 2) {
                    int16_t vol = 0;
                    err = uac2_host_device_get_volume(app->uac2_dev, 0, &vol);
                    ESP_LOGI(TAG, "  Get volume: %d (%.2f dB) %s",
                             vol, vol / 256.0, esp_err_to_name(err));
                    if (err != ESP_OK) t2_pass = false;
                    bool muted = false;
                    err = uac2_host_device_get_mute(app->uac2_dev, 0, &muted);
                    ESP_LOGI(TAG, "  Get mute: %s %s",
                             muted ? "MUTED" : "unmuted", esp_err_to_name(err));
                    if (err != ESP_OK) t2_pass = false;
                }
                if (writes == 2 * wps) {
                    err = uac2_host_device_set_volume(app->uac2_dev, 0, 0);
                    ESP_LOGI(TAG, "  Set volume=0dB: %s", esp_err_to_name(err));
                    if (err != ESP_OK) t2_pass = false;
                }
            }

            uac2_host_device_stop(app->uac2_dev);
            if (!app->dev_connected) return;
            ESP_LOGI(TAG, "=== TEST 2: %s ===", t2_pass ? "PASS" : "FAIL");
        }
    }

    // ── Test 3: Stop/restart cycle ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 3: Stop/restart cycle (5s -> 2s pause -> 5s) ===");
    sec = stream_tone(app, 48000, 24, 440.0f, 5);
    if (!app->dev_connected) return;
    bool t3_pass = (sec >= 5);
    ESP_LOGI(TAG, "  Pausing 2 seconds...");
    for (int i = 0; i < 20 && app->dev_connected; i++)
        vTaskDelay(pdMS_TO_TICKS(100));
    if (!app->dev_connected) return;
    sec = stream_tone(app, 48000, 24, 880.0f, 5);
    if (!app->dev_connected) return;
    t3_pass = t3_pass && (sec >= 5);
    ESP_LOGI(TAG, "=== TEST 3: %s ===", t3_pass ? "PASS" : "FAIL");

    // ── Test 4: 44.1kHz sample rate ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 4: 44.1kHz streaming (5 sec) ===");
    {
        err = uac2_host_device_set_sample_rate(app->uac2_dev, 44100);
        ESP_LOGI(TAG, "  Set 44100 Hz: %s", esp_err_to_name(err));
        uint32_t readback = 0;
        err = uac2_host_device_get_sample_rate(app->uac2_dev, &readback);
        ESP_LOGI(TAG, "  Readback: %" PRIu32 " Hz (%s)", readback, esp_err_to_name(err));

        sec = stream_tone(app, 44100, 24, 1000.0f, 5);
        if (!app->dev_connected) return;
        uac2_host_device_set_sample_rate(app->uac2_dev, 48000);
        ESP_LOGI(TAG, "=== TEST 4: %s (%d sec) ===", sec >= 5 ? "PASS" : "FAIL", sec);
    }

    // ── Test 5: Start time precision ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 5: Start time precision (5 iterations) ===");
    {
        bool t5_pass = true;
        for (int iter = 0; iter < 5 && app->dev_connected; iter++) {
            uac2_host_stream_config_t scfg = {
                .sample_freq = 48000, .channels = TONE_CHANNELS,
                .bit_resolution = TONE_BIT_DEPTH,
            };
            int64_t before = esp_timer_get_time();
            err = uac2_host_device_start(app->uac2_dev, &scfg);
            int64_t after = esp_timer_get_time();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "  [%d] start failed: %s", iter, esp_err_to_name(err));
                t5_pass = false;
                break;
            }
            int64_t start_time = uac2_host_device_get_start_time(app->uac2_dev);
            ESP_LOGI(TAG, "  [%d] before=%" PRId64 " start=%" PRId64 " after=%" PRId64
                     " delta=%" PRId64 " us",
                     iter, before, start_time, after, start_time - before);
            if (start_time < before || start_time > after) {
                ESP_LOGW(TAG, "  [%d] start_time outside expected range!", iter);
                t5_pass = false;
            }
            tone_gen_t gen;
            tone_gen_config_t tcfg = {
                .sample_rate = 48000, .channels = TONE_CHANNELS,
                .bit_depth = TONE_BIT_DEPTH, .frequency = 1000.0f,
                .amplitude = TONE_AMPLITUDE,
            };
            tone_gen_init(&gen, &tcfg);
            uint8_t tbuf[TONE_BUF_SIZE];
            uint32_t bsz = (48000 / 1000) * TONE_BUF_MS * TONE_CHANNELS * 3;
            for (int i = 0; i < 100 && app->dev_connected; i++) {
                tone_gen_fill(&gen, tbuf, bsz);
                uac2_host_device_write(app->uac2_dev, tbuf, bsz, 100);
            }
            uac2_host_device_stop(app->uac2_dev);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        if (!app->dev_connected) return;
        ESP_LOGI(TAG, "=== TEST 5: %s ===", t5_pass ? "PASS" : "FAIL");
    }

    // ── Test 6: Feedback convergence & clock validity ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 6: Feedback convergence & clock validity (5 sec) ===");
    {
        bool clk_before = false;
        uac2_host_device_get_clock_valid(app->uac2_dev, &clk_before);
        ESP_LOGI(TAG, "  Clock valid BEFORE stream: %s", clk_before ? "yes" : "no");

        uac2_host_stream_config_t scfg = {
            .sample_freq = 48000, .channels = TONE_CHANNELS,
            .bit_resolution = TONE_BIT_DEPTH,
        };
        err = uac2_host_device_start(app->uac2_dev, &scfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "=== TEST 6: FAIL (start: %s) ===", esp_err_to_name(err));
        } else {
            tone_gen_t gen;
            tone_gen_config_t tcfg = {
                .sample_rate = 48000, .channels = TONE_CHANNELS,
                .bit_depth = TONE_BIT_DEPTH, .frequency = 1000.0f,
                .amplitude = TONE_AMPLITUDE,
            };
            tone_gen_init(&gen, &tcfg);
            uint8_t tbuf[TONE_BUF_SIZE];
            uint32_t bsz = (48000 / 1000) * TONE_BUF_MS * TONE_CHANNELS * 3;
            for (int i = 0; i < 50 / TONE_BUF_MS && app->dev_connected; i++) {
                tone_gen_fill(&gen, tbuf, bsz);
                uac2_host_device_write(app->uac2_dev, tbuf, bsz, 100);
            }
            uint32_t writes = 0;
            uint32_t wps = 1000 / TONE_BUF_MS;
            bool got_feedback = false;
            while (app->dev_connected && writes < 5 * wps) {
                tone_gen_fill(&gen, tbuf, bsz);
                esp_err_t wr = uac2_host_device_write(app->uac2_dev, tbuf, bsz, 100);
                if (wr == ESP_ERR_INVALID_STATE) break;
                if (wr == ESP_OK) writes++;
                if (writes % wps == 0) {
                    uint32_t fb = uac2_host_device_get_feedback(app->uac2_dev);
                    bool clk = false;
                    uac2_host_device_get_clock_valid(app->uac2_dev, &clk);
                    ESP_LOGI(TAG, "  [%" PRIu32 "s] feedback=%" PRIu32 ".%04" PRIu32
                             " (raw=0x%08" PRIX32 "), clock=%s",
                             writes / wps, (uint32_t)(fb >> 16),
                             (uint32_t)((fb & 0xFFFF) * 10000 / 65536),
                             fb, clk ? "valid" : "INVALID");
                    if (fb != 0) got_feedback = true;
                }
            }
            uac2_host_device_stop(app->uac2_dev);
            bool clk_after = false;
            uac2_host_device_get_clock_valid(app->uac2_dev, &clk_after);
            ESP_LOGI(TAG, "  Feedback received: %s", got_feedback ? "YES" : "NO");
            if (!app->dev_connected) return;
            ESP_LOGI(TAG, "=== TEST 6: %s ===", got_feedback ? "PASS" : "PASS (no feedback)");
        }
    }

    // ── Test 7: 16-bit alt setting ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 7: 16-bit streaming (5 sec) ===");
    sec = stream_tone(app, 48000, 16, 1000.0f, 5);
    if (!app->dev_connected) return;
    ESP_LOGI(TAG, "=== TEST 7: %s (%d sec) ===", sec >= 5 ? "PASS" : "FAIL", sec);

    // ── Test 8: Rapid measurement cycles (9x) ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 8: Rapid measurement cycles (9x) ===");
    {
        bool t8_pass = true;
        float freqs[] = {100, 200, 300, 400, 500, 600, 700, 800, 900};

        for (int i = 0; i < 9 && app->dev_connected; i++) {
            int64_t cycle_start = esp_timer_get_time();
            uac2_host_stream_config_t scfg = {
                .sample_freq = 48000, .channels = TONE_CHANNELS,
                .bit_resolution = TONE_BIT_DEPTH,
            };
            err = uac2_host_device_start(app->uac2_dev, &scfg);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "  Cycle %d: start failed: %s", i, esp_err_to_name(err));
                t8_pass = false;
                break;
            }
            int64_t stime = uac2_host_device_get_start_time(app->uac2_dev);
            tone_gen_t gen;
            tone_gen_config_t tcfg = {
                .sample_rate = 48000, .channels = TONE_CHANNELS,
                .bit_depth = TONE_BIT_DEPTH, .frequency = freqs[i],
                .amplitude = TONE_AMPLITUDE,
            };
            tone_gen_init(&gen, &tcfg);
            uint8_t tbuf[TONE_BUF_SIZE];
            uint32_t bsz = (48000 / 1000) * TONE_BUF_MS * TONE_CHANNELS * 3;
            for (int j = 0; j < 50 / TONE_BUF_MS && app->dev_connected; j++) {
                tone_gen_fill(&gen, tbuf, bsz);
                uac2_host_device_write(app->uac2_dev, tbuf, bsz, 100);
            }
            uint32_t writes = 0;
            uint32_t wps = 1000 / TONE_BUF_MS;
            while (app->dev_connected && writes < 3 * wps) {
                tone_gen_fill(&gen, tbuf, bsz);
                esp_err_t wr = uac2_host_device_write(app->uac2_dev, tbuf, bsz, 100);
                if (wr == ESP_ERR_INVALID_STATE) break;
                if (wr == ESP_OK) writes++;
            }
            uac2_host_device_stop(app->uac2_dev);
            int64_t cycle_end = esp_timer_get_time();
            int cycle_sec = (int)(writes / wps);
            ESP_LOGI(TAG, "  Cycle %d: %.0f Hz, %d sec, start=%" PRId64 ", total=%" PRId64 " ms",
                     i, freqs[i], cycle_sec, stime, (cycle_end - cycle_start) / 1000);
            if (cycle_sec < 3) t8_pass = false;
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (!app->dev_connected) return;
        ESP_LOGI(TAG, "=== TEST 8: %s ===", t8_pass ? "PASS" : "FAIL");
    }

    // ── Test 9: Volume range & channel exploration ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 9: Volume range & channel exploration ===");
    {
        bool t9_pass = true;
        int16_t test_volumes[] = {0, -256, -3072, -6144, 256};
        for (int i = 0; i < 5; i++) {
            err = uac2_host_device_set_volume(app->uac2_dev, 0, test_volumes[i]);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "  Set %.1f dB: %s", test_volumes[i] / 256.0, esp_err_to_name(err));
                if (err != ESP_ERR_NOT_SUPPORTED && err != ESP_ERR_INVALID_ARG) t9_pass = false;
                continue;
            }
            int16_t readback = 0;
            err = uac2_host_device_get_volume(app->uac2_dev, 0, &readback);
            ESP_LOGI(TAG, "  Set %.1f dB -> readback %.2f dB [%s]",
                     test_volumes[i] / 256.0, readback / 256.0, esp_err_to_name(err));
        }
        for (int ch = 0; ch <= 2; ch++) {
            err = uac2_host_device_set_mute(app->uac2_dev, ch, true);
            ESP_LOGI(TAG, "  Mute ch%d=true: %s", ch, esp_err_to_name(err));
            bool muted = false;
            uac2_host_device_get_mute(app->uac2_dev, ch, &muted);
            ESP_LOGI(TAG, "  Readback ch%d mute: %s", ch, muted ? "yes" : "no");
            uac2_host_device_set_mute(app->uac2_dev, ch, false);
        }
        uac2_host_device_set_volume(app->uac2_dev, 0, 0);
        ESP_LOGI(TAG, "=== TEST 9: %s ===", t9_pass ? "PASS" : "FAIL");
    }

    // ── Test 10: Sample rate switch stress ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 10: Sample rate switch stress ===");
    {
        bool t10_pass = true;
        uint32_t rates[] = {48000, 44100, 48000, 44100, 48000};
        for (int i = 0; i < 5 && app->dev_connected; i++) {
            int64_t t0 = esp_timer_get_time();
            err = uac2_host_device_set_sample_rate(app->uac2_dev, rates[i]);
            int64_t t1 = esp_timer_get_time();
            uint32_t readback = 0;
            uac2_host_device_get_sample_rate(app->uac2_dev, &readback);
            ESP_LOGI(TAG, "  [%d] Set %" PRIu32 " -> read %" PRIu32 " (%" PRId64 " us) %s",
                     i, rates[i], readback, t1 - t0, esp_err_to_name(err));
            if (err == ESP_OK && readback != rates[i]) t10_pass = false;
            sec = stream_tone(app, rates[i], 24, 1000.0f, 2);
            if (!app->dev_connected) return;
            if (sec < 2) t10_pass = false;
        }
        uac2_host_device_set_sample_rate(app->uac2_dev, 48000);
        ESP_LOGI(TAG, "=== TEST 10: %s ===", t10_pass ? "PASS" : "FAIL");
    }

    // ── Test 11: Ring buffer starvation/recovery ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 11: Ring buffer starvation/recovery ===");
    {
        uac2_host_stream_config_t scfg = {
            .sample_freq = 48000, .channels = TONE_CHANNELS,
            .bit_resolution = TONE_BIT_DEPTH,
        };
        reset_event_counters();
        err = uac2_host_device_start(app->uac2_dev, &scfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "=== TEST 11: FAIL (start: %s) ===", esp_err_to_name(err));
        } else {
            tone_gen_t gen;
            tone_gen_config_t tcfg = {
                .sample_rate = 48000, .channels = TONE_CHANNELS,
                .bit_depth = TONE_BIT_DEPTH, .frequency = 1000.0f,
                .amplitude = TONE_AMPLITUDE,
            };
            tone_gen_init(&gen, &tcfg);
            uint8_t tbuf[TONE_BUF_SIZE];
            uint32_t bsz = (48000 / 1000) * TONE_BUF_MS * TONE_CHANNELS * 3;
            ESP_LOGI(TAG, "  Phase 1: Normal feed (2s)");
            uint32_t writes = 0;
            uint32_t wps = 1000 / TONE_BUF_MS;
            for (int i = 0; i < 50 / TONE_BUF_MS && app->dev_connected; i++) {
                tone_gen_fill(&gen, tbuf, bsz);
                uac2_host_device_write(app->uac2_dev, tbuf, bsz, 100);
            }
            while (app->dev_connected && writes < 2 * wps) {
                tone_gen_fill(&gen, tbuf, bsz);
                esp_err_t wr = uac2_host_device_write(app->uac2_dev, tbuf, bsz, 100);
                if (wr == ESP_ERR_INVALID_STATE) break;
                if (wr == ESP_OK) writes++;
            }
            log_event_counters("after normal feed");
            ESP_LOGI(TAG, "  Phase 2: Starvation (2s)");
            uint32_t errors_before = evt_errors;
            for (int i = 0; i < 20 && app->dev_connected; i++)
                vTaskDelay(pdMS_TO_TICKS(100));
            log_event_counters("after starvation");
            ESP_LOGI(TAG, "  New errors during starvation: %" PRIu32, evt_errors - errors_before);
            ESP_LOGI(TAG, "  Phase 3: Resume feed (2s)");
            writes = 0;
            while (app->dev_connected && writes < 2 * wps) {
                tone_gen_fill(&gen, tbuf, bsz);
                esp_err_t wr = uac2_host_device_write(app->uac2_dev, tbuf, bsz, 100);
                if (wr == ESP_ERR_INVALID_STATE) break;
                if (wr == ESP_OK) writes++;
            }
            log_event_counters("after recovery");
            uac2_host_device_stop(app->uac2_dev);
            if (!app->dev_connected) return;
            bool t11_pass = (writes > 0);
            ESP_LOGI(TAG, "=== TEST 11: %s (recovered %" PRIu32 " writes) ===",
                     t11_pass ? "PASS" : "FAIL", writes);
        }
    }

    // ── Test 12: Long-running stability ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 12: Long-running stability (until disconnect, max 1h) ===");
    stream_tone(app, 48000, 24, 1000.0f, 3600);
    ESP_LOGI(TAG, "=== TEST 12: ended (disconnect or timeout) ===");
}

// ── Device task ──────────────────────────────────────────────────

static void device_task(void *arg)
{
    app_state_t *app = (app_state_t *)arg;
    app->connect_cycle++;

    // ── Heap tracking ──
    size_t heap_before = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    if (app->heap_baseline == 0) {
        app->heap_baseline = heap_before;
    }
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " CONNECT CYCLE %" PRIu32 "  (heap: %u free, baseline: %u, delta: %d)",
             app->connect_cycle, (unsigned)heap_before, (unsigned)app->heap_baseline,
             (int)app->heap_baseline - (int)heap_before);
    ESP_LOGI(TAG, "========================================");

    // Open the playback interface
    uac2_host_device_config_t dev_cfg = {
        .addr = app->dev_addr,
        .iface_num = app->iface_num,
        .buffer_size = 0,       // auto
        .buffer_threshold = 0,  // auto
        .callback = device_event_cb,
        .callback_arg = app,
    };
    esp_err_t err = uac2_host_device_open(&dev_cfg, &app->uac2_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UAC2 device open failed: %s", esp_err_to_name(err));
        goto done;
    }
    ESP_LOGI(TAG, "*** UAC2 device opened (cycle %" PRIu32 ") ***", app->connect_cycle);

    // Only dump full device info on first cycle
    if (app->connect_cycle == 1) {
        log_all_device_info(app->uac2_dev);
        log_clock_info(app->uac2_dev);
        log_volume_info(app->uac2_dev);
        uac2_host_device_print_info(app->uac2_dev);
    }

    // Run full test suite
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " RUNNING TEST SUITE (cycle %" PRIu32 ")", app->connect_cycle);
    ESP_LOGI(TAG, "========================================");
    run_stream_tests(app);

    // Close device
    if (app->uac2_dev) {
        uac2_host_device_close(app->uac2_dev);
        app->uac2_dev = NULL;
    }

done:
    // ── Stack watermark ──
    UBaseType_t stack_hwm = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGI(TAG, "Device task stack high watermark: %u bytes free (of %d)",
             (unsigned)(stack_hwm * sizeof(StackType_t)), DEV_TASK_STACK_SIZE);

    // ── Heap after close ──
    size_t heap_after = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    int leak = (int)heap_before - (int)heap_after;
    ESP_LOGI(TAG, "Heap after close: %u free (leak this cycle: %d bytes, total from baseline: %d)",
             (unsigned)heap_after, leak, (int)app->heap_baseline - (int)heap_after);
    if (leak > 64) {
        ESP_LOGW(TAG, "POSSIBLE MEMORY LEAK: %d bytes not freed this cycle", leak);
    }

    ESP_LOGI(TAG, "--- Device task exiting (cycle %" PRIu32 ") ---", app->connect_cycle);
    ESP_LOGI(TAG, "Disconnect and reconnect USB to run again (cycle %" PRIu32 " next)",
             app->connect_cycle + 1);
    app->dev_connected = false;
    app->dev_task_hdl = NULL;
    vTaskDelete(NULL);
}

// ── Driver-level callback ────────────────────────────────────────

static void driver_event_cb(uint8_t addr, uint8_t iface_num,
                            const uac2_host_driver_event_t event, void *arg)
{
    app_state_t *app = (app_state_t *)arg;

    if (event == UAC2_HOST_DRIVER_EVENT_TX_CONNECTED && !app->dev_connected) {
        ESP_LOGI(TAG, "TX interface connected: addr=%d iface=%d", addr, iface_num);
        app->dev_addr = addr;
        app->iface_num = iface_num;
        app->dev_connected = true;

        // Create device task
        TaskHandle_t task_hdl = NULL;
        BaseType_t ret = xTaskCreatePinnedToCore(device_task, "uac2_dev",
                                                  DEV_TASK_STACK_SIZE, app,
                                                  UAC2_TASK_PRIORITY + 1, &task_hdl, 0);
        if (ret == pdPASS) {
            app->dev_task_hdl = task_hdl;
        } else {
            ESP_LOGE(TAG, "Failed to create device task");
        }
    } else if (event == UAC2_HOST_DRIVER_EVENT_RX_CONNECTED) {
        ESP_LOGI(TAG, "RX interface available: addr=%d iface=%d (not opening)", addr, iface_num);
    }
}

// ── USB Host Library task ────────────────────────────────────────

static void usb_host_lib_task(void *arg)
{
    ESP_LOGI(TAG, "Installing USB Host Library");
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    xTaskNotifyGive((TaskHandle_t)arg);

    while (1) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);

        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_LOGW(TAG, "No more clients");
            usb_host_device_free_all();
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "All USB devices freed, ready for reconnect");
        }
    }
}

// ── Entry point ──────────────────────────────────────────────────

void app_main(void)
{
    ESP_LOGI(TAG, "esp-uac2-host — Comprehensive UAC2 test suite v0.1");
    ESP_LOGI(TAG, "Driver v%d.%d.%d",
             UAC2_HOST_VER_MAJOR, UAC2_HOST_VER_MINOR, UAC2_HOST_VER_PATCH);

    memset(&s_app, 0, sizeof(s_app));

    // Run self-test against static miniDSP descriptors
    run_descriptor_self_test();

    // Start USB Host Library task
    TaskHandle_t host_task_hdl;
    xTaskCreatePinnedToCore(usb_host_lib_task, "usb_host", 4096,
                            xTaskGetCurrentTaskHandle(),
                            HOST_LIB_TASK_PRIORITY, &host_task_hdl, 0);

    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));

    // Install UAC2 Host driver (creates background event task)
    uac2_host_driver_config_t drv_cfg = {
        .create_background_task = true,
        .task_priority = UAC2_TASK_PRIORITY,
        .stack_size = UAC2_TASK_STACK_SIZE,
        .core_id = 0,
        .callback = driver_event_cb,
        .callback_arg = &s_app,
    };
    ESP_ERROR_CHECK(uac2_host_install(&drv_cfg));

    ESP_LOGI(TAG, "USB Host + UAC2 driver initialized, waiting for device...");
}
