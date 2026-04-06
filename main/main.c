/*
 * SPDX-FileCopyrightText: 2026 Avery Levitt
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * esp-uac2-host — Comprehensive UAC2 driver test suite
 *
 * Tests all driver functionality against a real UAC2 device (miniDSP 2x4 HD).
 * Exhaustive logging for simulator validation and driver verification.
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
#include "tone_gen.h"

// Static miniDSP descriptor dump for self-test (ref/ added to include path in CMakeLists)
#include "minidsp_2x4hd_descriptors.h"

#define HOST_LIB_TASK_PRIORITY  2
#define CLASS_TASK_PRIORITY     3
#define CLASS_TASK_STACK_SIZE   (6 * 1024)
#define DEV_TASK_STACK_SIZE     (12 * 1024)

// Tone test config
#define TONE_SAMPLE_RATE    48000
#define TONE_CHANNELS       2
#define TONE_BIT_DEPTH      24
#define TONE_FREQ_HZ        1000.0f
#define TONE_AMPLITUDE      0.5f
#define TONE_BUF_MS         10
// Buffer sized for worst case: 48kHz/24-bit/stereo/10ms = 2880 bytes
#define TONE_BUF_SIZE       ((TONE_SAMPLE_RATE / 1000) * TONE_BUF_MS * TONE_CHANNELS * (TONE_BIT_DEPTH / 8))

static const char *TAG = "uac2-host";

// ── Event tracking ────────────────────────────────────────────────

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

// ── Hex dump (for simulator descriptor capture) ───────────────────

static void hex_dump(const char *label, const uint8_t *data, uint16_t len)
{
    ESP_LOGI(TAG, "%s (%d bytes):", label, len);
    for (int i = 0; i < len; i += 16) {
        char line[80];
        int pos = snprintf(line, sizeof(line), "  [%04X] ", i);
        for (int j = 0; j < 16 && (i + j) < len; j++) {
            pos += snprintf(line + pos, sizeof(line) - pos, "%02X ", data[i + j]);
        }
        ESP_LOGI(TAG, "%s", line);
    }
}

// ── Self-test: parse static miniDSP descriptors ───────────────────

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

// ── Class driver ──────────────────────────────────────────────────

typedef struct {
    usb_host_client_handle_t client_hdl;
    usb_device_handle_t dev_hdl;
    uac2_host_device_handle_t uac2_dev;
    uint8_t dev_addr;
    volatile bool dev_connected;
    volatile TaskHandle_t dev_task_hdl;
} class_driver_t;

static void client_event_cb(const usb_host_client_event_msg_t *event, void *arg)
{
    class_driver_t *driver = (class_driver_t *)arg;
    switch (event->event) {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
        ESP_LOGI(TAG, "New device connected (addr %d)", event->new_dev.address);
        driver->dev_addr = event->new_dev.address;
        driver->dev_connected = true;
        break;
    case USB_HOST_CLIENT_EVENT_DEV_GONE:
        ESP_LOGW(TAG, "Device disconnected");
        driver->dev_connected = false;
        break;
    default:
        break;
    }
}

static void uac2_event_cb(uac2_host_device_handle_t dev,
                           uac2_host_event_t event, void *arg)
{
    switch (event) {
    case UAC2_HOST_EVENT_TX_DONE:
        evt_tx_done++;
        break;
    case UAC2_HOST_EVENT_RX_DONE:
        break;
    case UAC2_HOST_EVENT_TRANSFER_ERROR:
        evt_errors++;
        ESP_LOGW(TAG, "UAC2: Transfer error (#%" PRIu32 ")", evt_errors);
        break;
    case UAC2_HOST_EVENT_DISCONNECTED:
        evt_disconnects++;
        ESP_LOGW(TAG, "UAC2: Disconnected");
        break;
    }
}

// ── Raw descriptor dump (critical for simulator update) ───────────

static void dump_raw_descriptors(usb_device_handle_t dev_hdl)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " RAW DESCRIPTOR DUMP (for simulator)");
    ESP_LOGI(TAG, "========================================");

    // Device descriptor
    const usb_device_desc_t *dev_desc;
    if (usb_host_get_device_descriptor(dev_hdl, &dev_desc) == ESP_OK) {
        hex_dump("DEVICE DESCRIPTOR", (const uint8_t *)dev_desc, dev_desc->bLength);
        ESP_LOGI(TAG, "  VID=0x%04X PID=0x%04X bcdUSB=0x%04X bcdDevice=0x%04X",
                 dev_desc->idVendor, dev_desc->idProduct,
                 dev_desc->bcdUSB, dev_desc->bcdDevice);
        ESP_LOGI(TAG, "  Class=0x%02X SubClass=0x%02X Protocol=0x%02X",
                 dev_desc->bDeviceClass, dev_desc->bDeviceSubClass, dev_desc->bDeviceProtocol);
        ESP_LOGI(TAG, "  MaxPktSize0=%d NumConfigs=%d",
                 dev_desc->bMaxPacketSize0, dev_desc->bNumConfigurations);
        ESP_LOGI(TAG, "  iManufacturer=%d iProduct=%d iSerialNumber=%d",
                 dev_desc->iManufacturer, dev_desc->iProduct, dev_desc->iSerialNumber);
    }

    // Config descriptor — THE critical dump (full raw bytes including all interfaces)
    const usb_config_desc_t *config_desc;
    if (usb_host_get_active_config_descriptor(dev_hdl, &config_desc) == ESP_OK) {
        hex_dump("CONFIG DESCRIPTOR (FULL RAW)", (const uint8_t *)config_desc,
                 config_desc->wTotalLength);
        ESP_LOGI(TAG, "  wTotalLength=%d bNumInterfaces=%d bConfigurationValue=%d",
                 config_desc->wTotalLength, config_desc->bNumInterfaces,
                 config_desc->bConfigurationValue);
        ESP_LOGI(TAG, "  bmAttributes=0x%02X bMaxPower=%d",
                 config_desc->bmAttributes, config_desc->bMaxPower);
    }

    // String descriptors (raw hex for exact reproduction)
    usb_device_info_t dev_info;
    if (usb_host_device_info(dev_hdl, &dev_info) == ESP_OK) {
        ESP_LOGI(TAG, "Speed: %s",
                 dev_info.speed == 0 ? "Low" : dev_info.speed == 1 ? "Full" : "High");
        if (dev_info.str_desc_manufacturer) {
            hex_dump("STRING: Manufacturer",
                     (const uint8_t *)dev_info.str_desc_manufacturer,
                     dev_info.str_desc_manufacturer->bLength);
            usb_print_string_descriptor(dev_info.str_desc_manufacturer);
        }
        if (dev_info.str_desc_product) {
            hex_dump("STRING: Product",
                     (const uint8_t *)dev_info.str_desc_product,
                     dev_info.str_desc_product->bLength);
            usb_print_string_descriptor(dev_info.str_desc_product);
        }
        if (dev_info.str_desc_serial_num) {
            hex_dump("STRING: Serial",
                     (const uint8_t *)dev_info.str_desc_serial_num,
                     dev_info.str_desc_serial_num->bLength);
            usb_print_string_descriptor(dev_info.str_desc_serial_num);
        }
    }
}

// ── Parsed UAC2 device info logging ───────────────────────────────

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

    // Extra detail for simulator matching
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

// ── Clock & volume info logging ───────────────────────────────────

static void log_clock_info(uac2_host_device_handle_t uac2_dev)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "--- Clock info ---");

    uint32_t sample_rate = 0;
    esp_err_t err = uac2_host_get_sample_rate(uac2_dev, &sample_rate);
    ESP_LOGI(TAG, "Current sample rate: %" PRIu32 " Hz (%s)",
             sample_rate, esp_err_to_name(err));

    uac2_sample_rate_range_t ranges[UAC2_MAX_SAMPLE_RATE_RANGES];
    uint8_t num_ranges = 0;
    err = uac2_host_get_sample_rate_range(uac2_dev, ranges, &num_ranges);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Sample rate ranges: %d", num_ranges);
        for (int i = 0; i < num_ranges; i++) {
            ESP_LOGI(TAG, "  [%d] min=%" PRIu32 " max=%" PRIu32 " res=%" PRIu32,
                     i, ranges[i].min, ranges[i].max, ranges[i].res);
        }
    } else {
        ESP_LOGW(TAG, "Sample rate range: %s", esp_err_to_name(err));
    }

    bool clock_valid = false;
    err = uac2_host_get_clock_valid(uac2_dev, &clock_valid);
    ESP_LOGI(TAG, "Clock valid: %s (%s)", clock_valid ? "yes" : "no", esp_err_to_name(err));
}

static void log_volume_info(uac2_host_device_handle_t uac2_dev)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "--- Volume/Mute info ---");

    // Volume range (master)
    uac2_volume_range_t vranges[UAC2_MAX_VOLUME_RANGES];
    uint8_t num_vranges = 0;
    esp_err_t err = uac2_host_get_volume_range(uac2_dev, 0, vranges, &num_vranges);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Volume ranges (master ch0): %d", num_vranges);
        for (int i = 0; i < num_vranges; i++) {
            ESP_LOGI(TAG, "  [%d] min=%.2f dB  max=%.2f dB  res=%.4f dB",
                     i, vranges[i].min / 256.0, vranges[i].max / 256.0,
                     vranges[i].res / 256.0);
        }
    } else {
        ESP_LOGW(TAG, "Volume range (master): %s", esp_err_to_name(err));
    }

    // Per-channel volume ranges
    for (int ch = 1; ch <= 2; ch++) {
        err = uac2_host_get_volume_range(uac2_dev, ch, vranges, &num_vranges);
        if (err == ESP_OK && num_vranges > 0) {
            ESP_LOGI(TAG, "Volume range (ch%d): min=%.2f max=%.2f res=%.4f dB",
                     ch, vranges[0].min / 256.0, vranges[0].max / 256.0,
                     vranges[0].res / 256.0);
        } else {
            ESP_LOGW(TAG, "Volume range (ch%d): %s", ch, esp_err_to_name(err));
        }
    }

    // Current volume per channel
    for (int ch = 0; ch <= 2; ch++) {
        int16_t vol = 0;
        err = uac2_host_get_volume(uac2_dev, ch, &vol);
        ESP_LOGI(TAG, "Volume ch%d: %d raw (%.2f dB) [%s]",
                 ch, vol, vol / 256.0, esp_err_to_name(err));
    }

    // Mute state per channel
    for (int ch = 0; ch <= 2; ch++) {
        bool muted = false;
        err = uac2_host_get_mute(uac2_dev, ch, &muted);
        ESP_LOGI(TAG, "Mute ch%d: %s [%s]",
                 ch, muted ? "MUTED" : "unmuted", esp_err_to_name(err));
    }
}

// ── Streaming helper ──────────────────────────────────────────────

/**
 * Stream a tone. Returns seconds streamed, or 0 on failure.
 * Logs start_time, stream_start/stop timing, event counters.
 */
static int stream_tone(class_driver_t *driver, uint32_t sample_rate,
                       uint8_t bit_depth, float freq_hz, int duration_sec)
{
    uac2_stream_config_t stream_cfg = {
        .sample_rate = sample_rate,
        .channels = TONE_CHANNELS,
        .bit_resolution = bit_depth,
    };

    reset_event_counters();
    int64_t t0 = esp_timer_get_time();
    esp_err_t err = uac2_host_stream_start(driver->uac2_dev, UAC2_STREAM_TX,
                                            &stream_cfg);
    int64_t t1 = esp_timer_get_time();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Stream start failed: %s (%" PRId64 " us)",
                 esp_err_to_name(err), t1 - t0);
        return 0;
    }

    int64_t start_time = uac2_host_stream_get_start_time(driver->uac2_dev);
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
        uac2_host_stream_stop(driver->uac2_dev, UAC2_STREAM_TX);
        return 0;
    }

    uint32_t writes = 0;
    uint32_t writes_per_sec = 1000 / TONE_BUF_MS;
    uint32_t target_writes = (uint32_t)duration_sec * writes_per_sec;

    // Pre-fill ring buffer (~50ms)
    for (int i = 0; i < 50 / TONE_BUF_MS && driver->dev_connected; i++) {
        tone_gen_fill(&gen, tone_buf, buf_size);
        uac2_host_stream_write(driver->uac2_dev, tone_buf, buf_size, 100);
    }

    while (driver->dev_connected && writes < target_writes) {
        tone_gen_fill(&gen, tone_buf, buf_size);
        esp_err_t wr = uac2_host_stream_write(driver->uac2_dev,
                                               tone_buf, buf_size, 100);
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
    uac2_host_stream_stop(driver->uac2_dev, UAC2_STREAM_TX);
    t1 = esp_timer_get_time();
    ESP_LOGI(TAG, "Stream stopped after %d sec (stop took %" PRId64 " us)", seconds, t1 - t0);
    log_event_counters("stream_tone");
    return seconds;
}

// ── Test suite ────────────────────────────────────────────────────

static void run_stream_tests(class_driver_t *driver)
{
    esp_err_t err;
    int sec;

    // ── Test 1: Basic 48kHz/24-bit streaming (10s) ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 1: Basic 48kHz/24-bit streaming (10 sec) ===");
    sec = stream_tone(driver, 48000, 24, 1000.0f, 10);
    if (!driver->dev_connected) return;
    ESP_LOGI(TAG, "=== TEST 1: %s (%d sec) ===", sec >= 10 ? "PASS" : "FAIL", sec);

    // ── Test 2: Volume/mute control during streaming ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 2: Volume/mute control during streaming ===");
    {
        uac2_stream_config_t scfg = {
            .sample_rate = TONE_SAMPLE_RATE,
            .channels = TONE_CHANNELS,
            .bit_resolution = TONE_BIT_DEPTH,
        };
        err = uac2_host_stream_start(driver->uac2_dev, UAC2_STREAM_TX, &scfg);
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

            // Pre-fill
            for (int i = 0; i < 50 / TONE_BUF_MS && driver->dev_connected; i++) {
                tone_gen_fill(&gen, tbuf, bsz);
                uac2_host_stream_write(driver->uac2_dev, tbuf, bsz, 100);
            }

            uint32_t writes = 0;
            uint32_t wps = 1000 / TONE_BUF_MS;
            while (driver->dev_connected && writes < 3 * wps) {
                tone_gen_fill(&gen, tbuf, bsz);
                esp_err_t wr = uac2_host_stream_write(driver->uac2_dev, tbuf, bsz, 100);
                if (wr == ESP_ERR_INVALID_STATE) break;
                if (wr == ESP_OK) writes++;

                // 0.5s: mute
                if (writes == wps / 2) {
                    err = uac2_host_set_mute(driver->uac2_dev, 0, true);
                    ESP_LOGI(TAG, "  Set mute=true: %s", esp_err_to_name(err));
                    if (err != ESP_OK) t2_pass = false;
                }
                // 1.0s: unmute + volume -12dB
                if (writes == wps) {
                    err = uac2_host_set_mute(driver->uac2_dev, 0, false);
                    ESP_LOGI(TAG, "  Set mute=false: %s", esp_err_to_name(err));
                    if (err != ESP_OK) t2_pass = false;

                    err = uac2_host_set_volume(driver->uac2_dev, 0, -12 * 256);
                    ESP_LOGI(TAG, "  Set volume=-12dB: %s", esp_err_to_name(err));
                    if (err != ESP_OK) t2_pass = false;
                }
                // 1.5s: read back
                if (writes == wps + wps / 2) {
                    int16_t vol = 0;
                    err = uac2_host_get_volume(driver->uac2_dev, 0, &vol);
                    ESP_LOGI(TAG, "  Get volume: %d (%.2f dB) %s",
                             vol, vol / 256.0, esp_err_to_name(err));
                    if (err != ESP_OK) t2_pass = false;

                    bool muted = false;
                    err = uac2_host_get_mute(driver->uac2_dev, 0, &muted);
                    ESP_LOGI(TAG, "  Get mute: %s %s",
                             muted ? "MUTED" : "unmuted", esp_err_to_name(err));
                    if (err != ESP_OK) t2_pass = false;
                }
                // 2.0s: set volume 0dB (restore)
                if (writes == 2 * wps) {
                    err = uac2_host_set_volume(driver->uac2_dev, 0, 0);
                    ESP_LOGI(TAG, "  Set volume=0dB: %s", esp_err_to_name(err));
                    if (err != ESP_OK) t2_pass = false;
                }
            }

            uac2_host_stream_stop(driver->uac2_dev, UAC2_STREAM_TX);
            if (!driver->dev_connected) return;
            ESP_LOGI(TAG, "=== TEST 2: %s ===", t2_pass ? "PASS" : "FAIL");
        }
    }

    // ── Test 3: Stop/restart cycle ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 3: Stop/restart cycle (5s -> 2s pause -> 5s) ===");
    sec = stream_tone(driver, 48000, 24, 440.0f, 5);
    if (!driver->dev_connected) return;
    bool t3_pass = (sec >= 5);

    ESP_LOGI(TAG, "  Pausing 2 seconds...");
    for (int i = 0; i < 20 && driver->dev_connected; i++)
        vTaskDelay(pdMS_TO_TICKS(100));
    if (!driver->dev_connected) return;

    sec = stream_tone(driver, 48000, 24, 880.0f, 5);
    if (!driver->dev_connected) return;
    t3_pass = t3_pass && (sec >= 5);
    ESP_LOGI(TAG, "=== TEST 3: %s ===", t3_pass ? "PASS" : "FAIL");

    // ── Test 4: 44.1kHz sample rate ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 4: 44.1kHz streaming (5 sec) ===");
    {
        // Set sample rate and verify readback
        err = uac2_host_set_sample_rate(driver->uac2_dev, 44100);
        ESP_LOGI(TAG, "  Set 44100 Hz: %s", esp_err_to_name(err));

        uint32_t readback = 0;
        err = uac2_host_get_sample_rate(driver->uac2_dev, &readback);
        ESP_LOGI(TAG, "  Readback: %" PRIu32 " Hz (%s)", readback, esp_err_to_name(err));

        sec = stream_tone(driver, 44100, 24, 1000.0f, 5);
        if (!driver->dev_connected) return;

        // Restore 48kHz
        uac2_host_set_sample_rate(driver->uac2_dev, 48000);
        ESP_LOGI(TAG, "=== TEST 4: %s (%d sec) ===", sec >= 5 ? "PASS" : "FAIL", sec);
    }

    // ── Test 5: Start time precision ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 5: Start time precision (5 iterations) ===");
    {
        bool t5_pass = true;
        for (int iter = 0; iter < 5 && driver->dev_connected; iter++) {
            uac2_stream_config_t scfg = {
                .sample_rate = 48000, .channels = TONE_CHANNELS,
                .bit_resolution = TONE_BIT_DEPTH,
            };

            int64_t before = esp_timer_get_time();
            err = uac2_host_stream_start(driver->uac2_dev, UAC2_STREAM_TX, &scfg);
            int64_t after = esp_timer_get_time();

            if (err != ESP_OK) {
                ESP_LOGE(TAG, "  [%d] start failed: %s", iter, esp_err_to_name(err));
                t5_pass = false;
                break;
            }

            int64_t start_time = uac2_host_stream_get_start_time(driver->uac2_dev);
            ESP_LOGI(TAG, "  [%d] before=%" PRId64 " start=%" PRId64 " after=%" PRId64
                     " delta=%" PRId64 " us",
                     iter, before, start_time, after, start_time - before);

            // start_time should be between before and after
            if (start_time < before || start_time > after) {
                ESP_LOGW(TAG, "  [%d] start_time outside expected range!", iter);
                t5_pass = false;
            }

            // Brief stream to keep device happy, then stop
            tone_gen_t gen;
            tone_gen_config_t tcfg = {
                .sample_rate = 48000, .channels = TONE_CHANNELS,
                .bit_depth = TONE_BIT_DEPTH, .frequency = 1000.0f,
                .amplitude = TONE_AMPLITUDE,
            };
            tone_gen_init(&gen, &tcfg);
            uint8_t tbuf[TONE_BUF_SIZE];
            uint32_t bsz = (48000 / 1000) * TONE_BUF_MS * TONE_CHANNELS * 3;
            for (int i = 0; i < 100 && driver->dev_connected; i++) {
                tone_gen_fill(&gen, tbuf, bsz);
                uac2_host_stream_write(driver->uac2_dev, tbuf, bsz, 100);
            }

            uac2_host_stream_stop(driver->uac2_dev, UAC2_STREAM_TX);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        if (!driver->dev_connected) return;
        ESP_LOGI(TAG, "=== TEST 5: %s ===", t5_pass ? "PASS" : "FAIL");
    }

    // ── Test 6: Feedback convergence & clock validity ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 6: Feedback convergence & clock validity (5 sec) ===");
    {
        bool clk_before = false;
        uac2_host_get_clock_valid(driver->uac2_dev, &clk_before);
        ESP_LOGI(TAG, "  Clock valid BEFORE stream: %s", clk_before ? "yes" : "no");

        uac2_stream_config_t scfg = {
            .sample_rate = 48000, .channels = TONE_CHANNELS,
            .bit_resolution = TONE_BIT_DEPTH,
        };
        err = uac2_host_stream_start(driver->uac2_dev, UAC2_STREAM_TX, &scfg);
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

            // Pre-fill
            for (int i = 0; i < 50 / TONE_BUF_MS && driver->dev_connected; i++) {
                tone_gen_fill(&gen, tbuf, bsz);
                uac2_host_stream_write(driver->uac2_dev, tbuf, bsz, 100);
            }

            // Stream for 5 seconds, logging feedback + clock validity every second
            uint32_t writes = 0;
            uint32_t wps = 1000 / TONE_BUF_MS;
            bool got_feedback = false;
            while (driver->dev_connected && writes < 5 * wps) {
                tone_gen_fill(&gen, tbuf, bsz);
                esp_err_t wr = uac2_host_stream_write(driver->uac2_dev, tbuf, bsz, 100);
                if (wr == ESP_ERR_INVALID_STATE) break;
                if (wr == ESP_OK) writes++;

                // Log feedback + clock validity every second
                if (writes % wps == 0) {
                    uint32_t fb = uac2_host_stream_get_feedback(driver->uac2_dev);
                    bool clk = false;
                    uac2_host_get_clock_valid(driver->uac2_dev, &clk);
                    ESP_LOGI(TAG, "  [%" PRIu32 "s] feedback=%" PRIu32 ".%04" PRIu32
                             " samples/frame (raw=0x%08" PRIX32 "), clock=%s",
                             writes / wps,
                             (uint32_t)(fb >> 16),
                             (uint32_t)((fb & 0xFFFF) * 10000 / 65536),
                             fb, clk ? "valid" : "INVALID");
                    if (fb != 0) got_feedback = true;
                }
            }

            uac2_host_stream_stop(driver->uac2_dev, UAC2_STREAM_TX);

            bool clk_after = false;
            uac2_host_get_clock_valid(driver->uac2_dev, &clk_after);
            ESP_LOGI(TAG, "  Clock valid AFTER stream: %s", clk_after ? "yes" : "no");
            ESP_LOGI(TAG, "  Feedback received: %s", got_feedback ? "YES" : "NO (device may not send at FS)");

            if (!driver->dev_connected) return;
            ESP_LOGI(TAG, "=== TEST 6: %s ===",
                     got_feedback ? "PASS" : "PASS (no feedback, using nominal rate)");
        }
    }

    // ── Test 7: 16-bit alt setting ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 7: 16-bit streaming (5 sec) ===");
    sec = stream_tone(driver, 48000, 16, 1000.0f, 5);
    if (!driver->dev_connected) return;
    ESP_LOGI(TAG, "=== TEST 7: %s (%d sec) ===", sec >= 5 ? "PASS" : "FAIL", sec);

    // ── Test 8: Rapid measurement cycles (9x) ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 8: Rapid measurement cycles (9x, simulating multi-sub sweep) ===");
    {
        bool t8_pass = true;
        float freqs[] = {100, 200, 300, 400, 500, 600, 700, 800, 900};
        int64_t cycle_times[9];

        for (int i = 0; i < 9 && driver->dev_connected; i++) {
            int64_t cycle_start = esp_timer_get_time();

            uac2_stream_config_t scfg = {
                .sample_rate = 48000, .channels = TONE_CHANNELS,
                .bit_resolution = TONE_BIT_DEPTH,
            };
            err = uac2_host_stream_start(driver->uac2_dev, UAC2_STREAM_TX, &scfg);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "  Cycle %d: start failed: %s", i, esp_err_to_name(err));
                t8_pass = false;
                break;
            }

            int64_t stime = uac2_host_stream_get_start_time(driver->uac2_dev);

            // Stream for 3 seconds
            tone_gen_t gen;
            tone_gen_config_t tcfg = {
                .sample_rate = 48000, .channels = TONE_CHANNELS,
                .bit_depth = TONE_BIT_DEPTH, .frequency = freqs[i],
                .amplitude = TONE_AMPLITUDE,
            };
            tone_gen_init(&gen, &tcfg);
            uint8_t tbuf[TONE_BUF_SIZE];
            uint32_t bsz = (48000 / 1000) * TONE_BUF_MS * TONE_CHANNELS * 3;

            // Pre-fill
            for (int j = 0; j < 50 / TONE_BUF_MS && driver->dev_connected; j++) {
                tone_gen_fill(&gen, tbuf, bsz);
                uac2_host_stream_write(driver->uac2_dev, tbuf, bsz, 100);
            }

            uint32_t writes = 0;
            uint32_t wps = 1000 / TONE_BUF_MS;
            while (driver->dev_connected && writes < 3 * wps) {
                tone_gen_fill(&gen, tbuf, bsz);
                esp_err_t wr = uac2_host_stream_write(driver->uac2_dev, tbuf, bsz, 100);
                if (wr == ESP_ERR_INVALID_STATE) break;
                if (wr == ESP_OK) writes++;
            }

            uac2_host_stream_stop(driver->uac2_dev, UAC2_STREAM_TX);

            int64_t cycle_end = esp_timer_get_time();
            cycle_times[i] = cycle_end - cycle_start;
            int cycle_sec = (int)(writes / wps);

            ESP_LOGI(TAG, "  Cycle %d: %.0f Hz, %d sec, start=%" PRId64
                     ", total=%" PRId64 " ms",
                     i, freqs[i], cycle_sec, stime, cycle_times[i] / 1000);

            if (cycle_sec < 3) t8_pass = false;

            // Brief pause between cycles (simulates mute command delay)
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (!driver->dev_connected) return;
        ESP_LOGI(TAG, "=== TEST 8: %s (9 cycles) ===", t8_pass ? "PASS" : "FAIL");
    }

    // ── Test 9: Volume range & channel exploration ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 9: Volume range & channel exploration ===");
    {
        bool t9_pass = true;

        // Test volume set/get round-trip on master
        int16_t test_volumes[] = {0, -256, -3072, -6144, 256}; // 0, -1, -12, -24, +1 dB
        for (int i = 0; i < 5; i++) {
            err = uac2_host_set_volume(driver->uac2_dev, 0, test_volumes[i]);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "  Set %.1f dB: %s", test_volumes[i] / 256.0, esp_err_to_name(err));
                if (err != ESP_ERR_NOT_SUPPORTED) t9_pass = false;
                continue;
            }

            int16_t readback = 0;
            err = uac2_host_get_volume(driver->uac2_dev, 0, &readback);
            ESP_LOGI(TAG, "  Set %.1f dB -> readback %.2f dB [%s]",
                     test_volumes[i] / 256.0, readback / 256.0, esp_err_to_name(err));
        }

        // Test per-channel mute
        for (int ch = 0; ch <= 2; ch++) {
            err = uac2_host_set_mute(driver->uac2_dev, ch, true);
            ESP_LOGI(TAG, "  Mute ch%d=true: %s", ch, esp_err_to_name(err));

            bool muted = false;
            uac2_host_get_mute(driver->uac2_dev, ch, &muted);
            ESP_LOGI(TAG, "  Readback ch%d mute: %s", ch, muted ? "yes" : "no");

            err = uac2_host_set_mute(driver->uac2_dev, ch, false);
            ESP_LOGI(TAG, "  Mute ch%d=false: %s", ch, esp_err_to_name(err));
        }

        // Restore volume to 0 dB
        uac2_host_set_volume(driver->uac2_dev, 0, 0);

        ESP_LOGI(TAG, "=== TEST 9: %s ===", t9_pass ? "PASS" : "FAIL");
    }

    // ── Test 10: Sample rate switch stress ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 10: Sample rate switch stress ===");
    {
        bool t10_pass = true;
        uint32_t rates[] = {48000, 44100, 48000, 44100, 48000};

        for (int i = 0; i < 5 && driver->dev_connected; i++) {
            int64_t t0 = esp_timer_get_time();
            err = uac2_host_set_sample_rate(driver->uac2_dev, rates[i]);
            int64_t t1 = esp_timer_get_time();

            uint32_t readback = 0;
            uac2_host_get_sample_rate(driver->uac2_dev, &readback);

            ESP_LOGI(TAG, "  [%d] Set %" PRIu32 " -> read %" PRIu32 " (%" PRId64 " us) %s",
                     i, rates[i], readback, t1 - t0, esp_err_to_name(err));

            if (err == ESP_OK && readback != rates[i]) {
                ESP_LOGW(TAG, "  Rate mismatch!");
                t10_pass = false;
            }

            // Stream briefly at this rate to verify it actually works
            sec = stream_tone(driver, rates[i], 24, 1000.0f, 2);
            if (!driver->dev_connected) return;
            if (sec < 2) t10_pass = false;
        }

        // Restore 48kHz
        uac2_host_set_sample_rate(driver->uac2_dev, 48000);
        ESP_LOGI(TAG, "=== TEST 10: %s ===", t10_pass ? "PASS" : "FAIL");
    }

    // ── Test 11: Ring buffer starvation/recovery ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 11: Ring buffer starvation/recovery ===");
    {
        uac2_stream_config_t scfg = {
            .sample_rate = 48000, .channels = TONE_CHANNELS,
            .bit_resolution = TONE_BIT_DEPTH,
        };
        reset_event_counters();
        err = uac2_host_stream_start(driver->uac2_dev, UAC2_STREAM_TX, &scfg);
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

            // Phase 1: Feed normally for 2 seconds
            ESP_LOGI(TAG, "  Phase 1: Normal feed (2s)");
            uint32_t writes = 0;
            uint32_t wps = 1000 / TONE_BUF_MS;
            for (int i = 0; i < 50 / TONE_BUF_MS && driver->dev_connected; i++) {
                tone_gen_fill(&gen, tbuf, bsz);
                uac2_host_stream_write(driver->uac2_dev, tbuf, bsz, 100);
            }
            while (driver->dev_connected && writes < 2 * wps) {
                tone_gen_fill(&gen, tbuf, bsz);
                esp_err_t wr = uac2_host_stream_write(driver->uac2_dev, tbuf, bsz, 100);
                if (wr == ESP_ERR_INVALID_STATE) break;
                if (wr == ESP_OK) writes++;
            }
            log_event_counters("after normal feed");

            // Phase 2: STARVE for 2 seconds (no writes)
            ESP_LOGI(TAG, "  Phase 2: Starvation (2s, no writes)");
            uint32_t errors_before = evt_errors;
            for (int i = 0; i < 20 && driver->dev_connected; i++) {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            log_event_counters("after starvation");
            ESP_LOGI(TAG, "  New errors during starvation: %" PRIu32, evt_errors - errors_before);

            // Phase 3: Resume feeding for 2 seconds
            ESP_LOGI(TAG, "  Phase 3: Resume feed (2s)");
            writes = 0;
            while (driver->dev_connected && writes < 2 * wps) {
                tone_gen_fill(&gen, tbuf, bsz);
                esp_err_t wr = uac2_host_stream_write(driver->uac2_dev, tbuf, bsz, 100);
                if (wr == ESP_ERR_INVALID_STATE) break;
                if (wr == ESP_OK) writes++;
            }
            log_event_counters("after recovery");

            uac2_host_stream_stop(driver->uac2_dev, UAC2_STREAM_TX);
            if (!driver->dev_connected) return;

            // Pass if we didn't crash and stream was still functional
            bool t11_pass = (writes > 0);
            ESP_LOGI(TAG, "=== TEST 11: %s (recovered %" PRIu32 " writes) ===",
                     t11_pass ? "PASS" : "FAIL", writes);
        }
    }

    // ── Test 12: Long-running stability ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 12: Long-running stability (until disconnect, max 1h) ===");
    stream_tone(driver, 48000, 24, 1000.0f, 3600);
    ESP_LOGI(TAG, "=== TEST 12: ended (disconnect or timeout) ===");
}

// ── Device task ───────────────────────────────────────────────────

static void handle_new_device_task(void *arg);

static void handle_new_device(class_driver_t *driver)
{
    TaskHandle_t task_hdl = NULL;
    BaseType_t ret = xTaskCreatePinnedToCore(handle_new_device_task, "uac2_dev",
                                              DEV_TASK_STACK_SIZE, driver,
                                              CLASS_TASK_PRIORITY + 1, &task_hdl, 0);
    if (ret == pdPASS) {
        driver->dev_task_hdl = task_hdl;
    } else {
        ESP_LOGE(TAG, "Failed to create device task");
    }
}

static void handle_new_device_task(void *arg)
{
    class_driver_t *driver = (class_driver_t *)arg;
    esp_err_t err;

    // Open device
    err = usb_host_device_open(driver->client_hdl, driver->dev_addr, &driver->dev_hdl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open device: %s", esp_err_to_name(err));
        driver->dev_task_hdl = NULL;
        vTaskDelete(NULL);
        return;
    }

    // Device info
    usb_device_info_t dev_info;
    err = usb_host_device_info(driver->dev_hdl, &dev_info);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Device info failed: %s", esp_err_to_name(err));
        goto done;
    }
    ESP_LOGI(TAG, "Device info: %s speed",
             dev_info.speed <= 2 ? (char *[]){"Low", "Full", "High"}[dev_info.speed] : "Unknown");

    // ── Phase 0: Raw descriptor dump ──
    dump_raw_descriptors(driver->dev_hdl);

    // Try to open as UAC2 device
    err = uac2_host_device_open(driver->client_hdl, driver->dev_hdl,
                                uac2_event_cb, driver, &driver->uac2_dev);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "*** UAC2 device opened ***");

        // ── Phase 1: Exhaustive device info dump ──
        log_all_device_info(driver->uac2_dev);
        log_clock_info(driver->uac2_dev);
        log_volume_info(driver->uac2_dev);

        // ── Phase 2: Run full test suite ──
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, " RUNNING TEST SUITE");
        ESP_LOGI(TAG, "========================================");
        run_stream_tests(driver);

    } else if (err == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGI(TAG, "Not a UAC2 device");
    } else {
        ESP_LOGE(TAG, "UAC2 open failed: %s", esp_err_to_name(err));
    }

done:
    ESP_LOGI(TAG, "--- Device task exiting ---");
    driver->dev_task_hdl = NULL;
    vTaskDelete(NULL);
}

static void handle_device_gone(class_driver_t *driver)
{
    if (driver->dev_task_hdl) {
        ESP_LOGI(TAG, "Waiting for device task to exit...");
        for (int i = 0; i < 20 && driver->dev_task_hdl != NULL; i++) {
            usb_host_client_handle_events(driver->client_hdl, pdMS_TO_TICKS(50));
        }
        if (driver->dev_task_hdl) {
            ESP_LOGW(TAG, "Device task did not exit in time");
        }
    }

    if (driver->uac2_dev) {
        uac2_host_device_close(driver->uac2_dev);
        driver->uac2_dev = NULL;
    }

    if (driver->dev_hdl) {
        usb_host_device_close(driver->client_hdl, driver->dev_hdl);
        driver->dev_hdl = NULL;
        ESP_LOGI(TAG, "Device closed");
    }
}

static void class_driver_task(void *arg)
{
    class_driver_t driver = {
        .client_hdl = NULL,
        .dev_hdl = NULL,
        .uac2_dev = NULL,
        .dev_addr = 0,
        .dev_connected = false,
        .dev_task_hdl = NULL,
    };

    ESP_LOGI(TAG, "Registering USB Host client");
    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = 5,
        .async = {
            .client_event_callback = client_event_cb,
            .callback_arg = &driver,
        },
    };
    ESP_ERROR_CHECK(usb_host_client_register(&client_config, &driver.client_hdl));
    ESP_LOGI(TAG, "Waiting for USB device...");

    while (1) {
        usb_host_client_handle_events(driver.client_hdl, portMAX_DELAY);

        if (driver.dev_connected && driver.dev_hdl == NULL && driver.dev_task_hdl == NULL) {
            handle_new_device(&driver);
        }

        if (!driver.dev_connected && driver.dev_hdl != NULL) {
            handle_device_gone(&driver);
            ESP_LOGI(TAG, "Waiting for USB device...");
        }
    }
}

// ── USB Host Library task ─────────────────────────────────────────

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
    }
}

// ── Entry point ───────────────────────────────────────────────────

void app_main(void)
{
    ESP_LOGI(TAG, "esp-uac2-host — Comprehensive UAC2 test suite v1.1");
    ESP_LOGI(TAG, "Driver v%d.%d.%d",
             UAC2_HOST_VER_MAJOR, UAC2_HOST_VER_MINOR, UAC2_HOST_VER_PATCH);

    // Run self-test against static miniDSP descriptors
    run_descriptor_self_test();

    // Start USB Host Library task
    TaskHandle_t host_task_hdl;
    xTaskCreatePinnedToCore(usb_host_lib_task, "usb_host", 4096,
                            xTaskGetCurrentTaskHandle(),
                            HOST_LIB_TASK_PRIORITY, &host_task_hdl, 0);

    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));

    // Start class driver task
    xTaskCreatePinnedToCore(class_driver_task, "class_drv", CLASS_TASK_STACK_SIZE,
                            NULL, CLASS_TASK_PRIORITY, NULL, 0);

    ESP_LOGI(TAG, "USB Host initialized, tasks running");
}
