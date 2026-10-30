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
 *   6.  Feedback presence & clock validity monitoring
 *   7.  16-bit alt setting
 *   8.  Rapid measurement cycles (9x stop/start)
 *   9.  Volume range & channel exploration
 *  10.  Sample rate switch stress
 *  11.  Ring buffer starvation/recovery
 *  12.  Long-running stability
 *
 * Additional live checks:
 *   - Suspend/resume on an active playback stream
 *   - ESP32-S3 duplex guard (reject opposite-direction stream activation)
 *   - Restart mid-stream (CONFIG_UAC2_TEST_RESTART_SEC)
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
#include "esp_idf_version.h"
#include "esp_system.h"
#if CONFIG_HEAP_TRACING_STANDALONE
#include "esp_heap_trace.h"
#define HEAP_TRACE_RECORDS  300
static heap_trace_record_t s_heap_trace_records[HEAP_TRACE_RECORDS];
#endif
#include "tone_gen.h"

// Static miniDSP descriptor dump for self-test
#include "minidsp_2x4hd_descriptors.h"

#define HOST_LIB_TASK_PRIORITY  2
#define UAC2_TASK_PRIORITY      3
#define UAC2_TASK_STACK_SIZE    (6 * 1024)
#define DEV_TASK_STACK_SIZE     (16 * 1024)

// Tone test config
#define TONE_SAMPLE_RATE    48000
#define TONE_CHANNELS       2
#define TONE_BIT_DEPTH      24
#if CONFIG_UAC2_TEST_TONES_SUBWOOFER
// Every tone inside a subwoofer's pass band, at -12 dBFS
#define TONE_FREQ_HZ        50.0f
#define TONE_LOW_HZ         40.0f
#define TONE_HIGH_HZ        80.0f
#define TONE_ALT_HZ         60.0f
#define TONE_CYCLE_HZ(i)    (30.0f + 10.0f * (i))
#define TONE_AMPLITUDE      0.25f
#else
#define TONE_FREQ_HZ        1000.0f
#define TONE_LOW_HZ         440.0f
#define TONE_HIGH_HZ        880.0f
#define TONE_ALT_HZ         750.0f
#define TONE_CYCLE_HZ(i)    (100.0f * ((i) + 1))
#define TONE_AMPLITUDE      0.5f
#endif
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

    // Device task lifecycle (protected by s_app_lock). A reconnect that arrives
    // while the previous cycle's task is still tearing down is queued here and
    // started by that task on exit, so two tasks never share this state.
    bool task_active;
    bool connect_pending;
    uint8_t pending_addr;
    uint8_t pending_iface;

    // Driver reinstall live check (protected by s_app_lock): while set, the
    // USB Host Library task keeps devices on NO_CLIENTS and the driver
    // callback records the re-reported device instead of queueing a task.
    bool reinstall_check_active;
    bool reinstall_reported;
    uint8_t reinstall_addr;
    uint8_t reinstall_iface;

    bool ep0_hang_check_done;        // the ep0-hang simulator's fault is one-shot
} app_state_t;

static app_state_t s_app;
static portMUX_TYPE s_app_lock = portMUX_INITIALIZER_UNLOCKED;
static uac2_host_driver_config_t s_driver_cfg;

static void start_device_task(app_state_t *app, uint8_t addr, uint8_t iface_num);

#define HEAP_SETTLE_TIMEOUT_MS 3000

// ── Event tracking ───────────────────────────────────────────────

static volatile uint32_t evt_tx_done = 0;
static volatile uint32_t evt_errors = 0;
static volatile uint32_t evt_disconnects = 0;
static volatile uint32_t evt_stream_errors = 0;

static void reset_event_counters(void)
{
    evt_tx_done = 0;
    evt_errors = 0;
    evt_disconnects = 0;
    evt_stream_errors = 0;
}

static void log_event_counters(const char *label)
{
    ESP_LOGI(TAG, "Events [%s]: TX_DONE=%" PRIu32 " ERRORS=%" PRIu32
             " STREAM_ERRORS=%" PRIu32 " DISCONNECTS=%" PRIu32,
             label, evt_tx_done, evt_errors, evt_stream_errors, evt_disconnects);
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
        evt_stream_errors++;
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

/** The playback alt start() tries first for cfg, copied to *out. */
static bool find_selected_playback_alt(uac2_host_device_handle_t uac2_dev,
                                       uint8_t iface_num,
                                       const uac2_host_stream_config_t *cfg,
                                       uac2_as_iface_t *out)
{
    uac2_device_info_t info;
    if (uac2_host_device_get_info(uac2_dev, &info) != ESP_OK) {
        return false;
    }

    const uac2_as_iface_t *best = NULL;
    for (int i = 0; i < info.num_as_ifaces; i++) {
        const uac2_as_iface_t *as = &info.as_ifaces[i];

        if (as->interface_num != iface_num) continue;
        if ((as->ep_addr & 0x80u) != 0) continue;  // playback uses iso OUT
        if (cfg->bit_resolution && as->bit_resolution != cfg->bit_resolution) continue;
        if (cfg->channels && as->nr_channels != cfg->channels) continue;

        if (!best || as->bit_resolution > best->bit_resolution) {
            best = as;
        }
    }

    if (best) *out = *best;
    return best != NULL;
}

static bool selected_stream_has_feedback(uac2_host_device_handle_t uac2_dev,
                                         uint8_t iface_num,
                                         const uac2_host_stream_config_t *cfg)
{
    uac2_as_iface_t as;
    return find_selected_playback_alt(uac2_dev, iface_num, cfg, &as) && as.fb_ep_addr != 0;
}

/**
 * Whether start() should manage a fractional rate (44.1 kHz) for cfg: it tries
 * every matching playback alt, and an alt needs a feedback endpoint unless its
 * data endpoint is adaptive or synchronous. Other reasons start() skips an alt
 * (endpoint interval, max packet size, the host refusing the claim) are not
 * modelled: they do not depend on the rate, so such a device already fails
 * the 48 kHz tests.
 */
static bool selected_stream_plays_fractional(uac2_host_device_handle_t uac2_dev,
                                             uint8_t iface_num,
                                             const uac2_host_stream_config_t *cfg)
{
    uac2_device_info_t info;
    if (uac2_host_device_get_info(uac2_dev, &info) != ESP_OK) {
        return false;
    }
    for (int i = 0; i < info.num_as_ifaces; i++) {
        const uac2_as_iface_t *as = &info.as_ifaces[i];
        if (as->interface_num != iface_num) continue;
        if ((as->ep_addr & 0x80u) != 0) continue;
        if (cfg->bit_resolution && as->bit_resolution != cfg->bit_resolution) continue;
        if (cfg->channels && as->nr_channels != cfg->channels) continue;
        uint8_t sync_type = (as->ep_attributes >> 2) & 0x03;
        if (as->fb_ep_addr != 0 || sync_type == UAC2_EP_SYNC_ADAPTIVE || sync_type == UAC2_EP_SYNC_SYNC) {
            return true;
        }
    }
    return false;
}

static const uac2_terminal_t *find_terminal_by_id(const uac2_device_info_t *info,
                                                  uint8_t terminal_id)
{
    for (int i = 0; i < info->num_terminals; i++) {
        if (info->terminals[i].terminal_id == terminal_id) {
            return &info->terminals[i];
        }
    }
    return NULL;
}

static const uac2_feature_unit_t *find_feature_unit_by_id(const uac2_device_info_t *info,
                                                          uint8_t unit_id)
{
    for (int i = 0; i < info->num_feature_units; i++) {
        if (info->feature_units[i].unit_id == unit_id) {
            return &info->feature_units[i];
        }
    }
    return NULL;
}

static const uac2_feature_unit_t *resolve_feature_unit_for_iface_info(const uac2_device_info_t *info,
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

static bool channel_map_supports(uint32_t channel_map, uint8_t channel)
{
    return channel < 32 && (channel_map & (1u << channel)) != 0;
}

static int first_supported_channel(uint32_t channel_map)
{
    for (uint8_t channel = 0; channel < 32; channel++) {
        if (channel_map_supports(channel_map, channel)) {
            return channel;
        }
    }
    return -1;
}

typedef enum {
    SIM_PROFILE_NONE = 0,
    SIM_PROFILE_NO_FEEDBACK,
    SIM_PROFILE_CHANNEL_ONLY_FU,
    SIM_PROFILE_EP0_HANG,
    SIM_PROFILE_FEEDBACK_DRIFT,
} simulator_profile_t;

static simulator_profile_t detect_simulator_profile(const uac2_device_info_t *info)
{
    if (strstr(info->product, "[sim no-fb]")) {
        return SIM_PROFILE_NO_FEEDBACK;
    }
    if (strstr(info->product, "[sim ch-only]")) {
        return SIM_PROFILE_CHANNEL_ONLY_FU;
    }
    if (strstr(info->product, "[sim ep0-hang]")) {
        return SIM_PROFILE_EP0_HANG;
    }
    if (strstr(info->product, "[sim fb-drift]")) {
        return SIM_PROFILE_FEEDBACK_DRIFT;
    }
    return SIM_PROFILE_NONE;
}

static bool playback_alts_match_feedback_expectation(const uac2_device_info_t *info,
                                                     uint8_t iface_num, bool expect_feedback)
{
    bool saw_playback_alt = false;

    for (int i = 0; i < info->num_as_ifaces; i++) {
        const uac2_as_iface_t *as = &info->as_ifaces[i];

        if (as->interface_num != iface_num) continue;
        if ((as->ep_addr & 0x80u) != 0) continue;

        saw_playback_alt = true;
        if ((as->fb_ep_addr != 0) != expect_feedback) {
            ESP_LOGE(TAG, "Simulator profile mismatch: iface %d alt %d feedback=%s, expected %s",
                     iface_num, as->alt_setting,
                     as->fb_ep_addr ? "present" : "absent",
                     expect_feedback ? "present" : "absent");
            return false;
        }
    }

    if (!saw_playback_alt) {
        ESP_LOGE(TAG, "Simulator profile mismatch: no playback alts found on iface %d", iface_num);
    }
    return saw_playback_alt;
}

static bool assert_simulator_profile_expectations(const uac2_device_info_t *info, uint8_t iface_num)
{
    simulator_profile_t profile = detect_simulator_profile(info);

    switch (profile) {
    case SIM_PROFILE_NO_FEEDBACK:
        ESP_LOGI(TAG, "Simulator profile detected: no-feedback");
        return playback_alts_match_feedback_expectation(info, iface_num, false);

    case SIM_PROFILE_CHANNEL_ONLY_FU: {
        ESP_LOGI(TAG, "Simulator profile detected: channel-only feature unit");
        const uac2_feature_unit_t *fu = resolve_feature_unit_for_iface_info(info, iface_num);
        if (!fu) {
            ESP_LOGE(TAG, "Simulator profile mismatch: no feature unit resolved for iface %d", iface_num);
            return false;
        }
        if (fu->volume_ch_map != 0x00000006u || fu->mute_ch_map != 0x00000006u) {
            ESP_LOGE(TAG, "Simulator profile mismatch: expected mute/volume maps 0x00000006, got volume=0x%08" PRIX32 " mute=0x%08" PRIX32,
                     fu->volume_ch_map, fu->mute_ch_map);
            return false;
        }
        return true;
    }

    case SIM_PROFILE_EP0_HANG:
        // Its one-shot fault already ran in the control-timeout live check
        ESP_LOGI(TAG, "Simulator profile detected: ep0-hang (reconnected after the fault)");
        return true;

    case SIM_PROFILE_FEEDBACK_DRIFT:
        ESP_LOGI(TAG, "Simulator profile detected: feedback drift (+2 Hz); "
                 "check the simulator log's PACKET SIZING CHECK lines");
        return true;

    case SIM_PROFILE_NONE:
    default:
        return true;
    }
}

static uint32_t tone_chunk_bytes(uint32_t sample_rate, uint8_t bit_depth,
                                 uint32_t interval_ms, uint32_t *frame_remainder)
{
    uint32_t bytes_per_frame = TONE_CHANNELS * (bit_depth / 8);
    uint64_t frame_total = ((uint64_t)sample_rate * interval_ms) + *frame_remainder;
    uint32_t frames = (uint32_t)(frame_total / 1000);
    *frame_remainder = (uint32_t)(frame_total % 1000);
    return frames * bytes_per_frame;
}

static uint32_t max_tone_chunk_bytes(uint32_t sample_rate, uint8_t bit_depth,
                                     uint32_t interval_ms)
{
    uint32_t bytes_per_frame = TONE_CHANNELS * (bit_depth / 8);
    uint32_t max_frames = (uint32_t)((((uint64_t)sample_rate * interval_ms) + 999) / 1000);
    return max_frames * bytes_per_frame;
}

static uint32_t sample_rate_to_feedback_q16(uint32_t sample_rate_hz)
{
    return (uint32_t)(((uint64_t)sample_rate_hz << 16) / 1000);
}

static uint32_t q16_abs_diff(uint32_t a, uint32_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

static bool find_capture_iface_config(const uac2_device_info_t *info,
                                      uint8_t *iface_num_out,
                                      uac2_host_stream_config_t *cfg_out)
{
    if (!info || !iface_num_out || !cfg_out) {
        return false;
    }

    const uac2_as_iface_t *best = NULL;
    for (int i = 0; i < info->num_as_ifaces; i++) {
        const uac2_as_iface_t *as = &info->as_ifaces[i];
        if ((as->ep_addr & 0x80u) == 0 || as->alt_setting == 0) {
            continue;
        }
        if (!best ||
            as->bit_resolution > best->bit_resolution ||
            (as->bit_resolution == best->bit_resolution && as->nr_channels > best->nr_channels)) {
            best = as;
        }
    }

    if (!best) {
        return false;
    }

    *iface_num_out = best->interface_num;
    cfg_out->sample_freq = 48000;
    cfg_out->channels = best->nr_channels;
    cfg_out->bit_resolution = best->bit_resolution;
    cfg_out->flags = 0;
    return true;
}

/**
 * uac2_host_device_start() for a playback stream fed from the tone generator,
 * which writes packed samples (bit_depth / 8 bytes). A device may carry 24-bit
 * samples in 4-byte slots; packed data would misframe its audio while write
 * counts still look fine, so such a stream is stopped again and reported as
 * ESP_FAIL. Every harness start that writes tone goes through here.
 */
static esp_err_t start_playback(uac2_host_device_handle_t dev, const uac2_host_stream_config_t *cfg)
{
    esp_err_t err = uac2_host_device_start(dev, cfg);
    if (err != ESP_OK) {
        return err;
    }
    uac2_host_stream_info_t info;
    err = uac2_host_device_get_stream_info(dev, &info);
    if (err == ESP_OK && info.sub_slot_size * 8 != info.bit_resolution) {
        ESP_LOGE(TAG, "Unsupported sample layout for this harness: %d-bit samples in %d-byte slots",
                 info.bit_resolution, info.sub_slot_size);
        err = ESP_FAIL;
    }
    if (err != ESP_OK) {
        uac2_host_device_stop(dev);
    }
    return err;
}

static void tone_init(tone_gen_t *gen, uint32_t sample_rate, uint8_t bit_depth, float freq_hz)
{
    tone_gen_config_t tone_cfg = {
        .sample_rate = sample_rate,
        .channels = TONE_CHANNELS,
        .bit_depth = bit_depth,
        .frequency = freq_hz,
        .amplitude = TONE_AMPLITUDE,
    };
    tone_gen_init(gen, &tone_cfg);
}

// The caller's generator carries on across prefill, feeding and the fade-out,
// so the tone has no seams
static uint32_t prefill_tone_buffer(uac2_host_device_handle_t dev, tone_gen_t *gen,
                                    uint32_t prefill_ms)
{
    uint32_t sample_rate = gen->config.sample_rate;
    uint8_t bit_depth = gen->config.bit_depth;
    uint32_t frame_remainder = 0;
    uint32_t writes = 0;
    uint8_t tone_buf[TONE_BUF_SIZE];
    uint32_t prefill_writes = prefill_ms / TONE_BUF_MS;
    for (uint32_t i = 0; i < prefill_writes; i++) {
        uint32_t buf_size = tone_chunk_bytes(sample_rate, bit_depth, TONE_BUF_MS, &frame_remainder);
        tone_gen_fill(gen, tone_buf, buf_size);
        if (uac2_host_device_write(dev, tone_buf, buf_size, 100) == ESP_OK) {
            writes++;
        }
    }
    return writes;
}

// Writes only complete while URBs drain the ring buffer, so a stream that
// stays ACTIVE but stalls would otherwise be retried forever. Allow twice the
// nominal feed time plus slack, then let the caller's write count fail it.
static int64_t write_deadline_us(uint32_t target_writes)
{
    return esp_timer_get_time() + ((int64_t)target_writes * TONE_BUF_MS * 2 + 2000) * 1000;
}

// close() keeps the interface open when stream teardown has not finished
// (e.g. URBs still draining after an unplug). Retry so the handle is not
// leaked, which would also block reopening the interface on reconnect.
static void close_device_with_retry(uac2_host_device_handle_t dev, const char *what)
{
    for (int attempt = 1; attempt <= 20; attempt++) {
        esp_err_t err = uac2_host_device_close(dev);
        if (err == ESP_OK) return;
        ESP_LOGE(TAG, "%s close failed (attempt %d): %s", what, attempt, esp_err_to_name(err));
        if (err == ESP_ERR_INVALID_ARG) return;  // no longer open
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGE(TAG, "%s interface LEAKED: close kept failing", what);
}

static uint32_t feed_tone_for_writes(app_state_t *app, uac2_host_device_handle_t dev,
                                     tone_gen_t *gen, uint32_t target_writes,
                                     const char *label)
{
    uint32_t sample_rate = gen->config.sample_rate;
    uint8_t bit_depth = gen->config.bit_depth;
    uint32_t frame_remainder = 0;
    uint32_t writes = 0;
    uint32_t writes_per_sec = 1000 / TONE_BUF_MS;
    uint8_t tone_buf[TONE_BUF_SIZE];
    int64_t deadline = write_deadline_us(target_writes);
    while (app->dev_connected && writes < target_writes && esp_timer_get_time() < deadline) {
        uint32_t buf_size = tone_chunk_bytes(sample_rate, bit_depth, TONE_BUF_MS, &frame_remainder);
        tone_gen_fill(gen, tone_buf, buf_size);
        esp_err_t err = uac2_host_device_write(dev, tone_buf, buf_size, 100);
        if (err == ESP_ERR_INVALID_STATE) {
            break;
        }
        if (err == ESP_OK) {
            writes++;
            if (label && writes % writes_per_sec == 0) {
                ESP_LOGI(TAG, "  %s %" PRIu32 " sec", label, writes / writes_per_sec);
            }
        }
    }
    return writes;
}

/**
 * End a tone without a click: fade the generator out over the next writes,
 * then let the queued audio play before the caller stops the stream, since
 * stop() discards what is still in the ring buffer.
 */
static void tone_fade_out_and_drain(app_state_t *app, tone_gen_t *gen, uint8_t *buf,
                                    uint32_t chunk_bytes)
{
    tone_gen_fade_out(gen);
    // The fade (up to 133 ms at 30 Hz), then one chunk of silence
    for (int i = 0; i < 30 && app->dev_connected; i++) {
        bool last = tone_gen_silent(gen);
        tone_gen_fill(gen, buf, chunk_bytes);
        if (uac2_host_device_write(app->uac2_dev, buf, chunk_bytes, 100) != ESP_OK) {
            return;
        }
        if (last) break;
    }
    int64_t deadline = esp_timer_get_time() + 2000000;
    uac2_host_buffer_status_t status = {0};
    while (app->dev_connected && esp_timer_get_time() < deadline) {
        if (uac2_host_device_get_buffer_status(app->uac2_dev, &status) != ESP_OK) {
            return;     // stream no longer active
        }
        if (status.ring_bytes == 0) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    // Then the audio already in submitted URBs: rounded up to whole ticks, plus
    // one because vTaskDelay() can end up to a tick early
    uint32_t wait_ms = status.in_flight_us / 1000 + 5;
    TickType_t ticks = (wait_ms * configTICK_RATE_HZ + 999) / 1000;
    vTaskDelay(ticks + 1);
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
    esp_err_t err = start_playback(app->uac2_dev, &stream_cfg);
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
    uint32_t max_chunk_size = max_tone_chunk_bytes(sample_rate, bit_depth, TONE_BUF_MS);
    uint32_t frame_remainder = 0;

    ESP_LOGI(TAG, "Streaming %d Hz @ %" PRIu32 " Hz/%d-bit for %d sec (%" PRIu32 " B/ms)",
             (int)freq_hz, sample_rate, bit_depth, duration_sec,
             (uint32_t)(((uint64_t)sample_rate * TONE_CHANNELS * bytes_per_sample) / 1000));

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
    if (max_chunk_size > TONE_BUF_SIZE) {
        ESP_LOGE(TAG, "Sample rate too high for tone buffer");
        uac2_host_device_stop(app->uac2_dev);
        return 0;
    }

    uint32_t writes = 0;
    uint32_t writes_per_sec = 1000 / TONE_BUF_MS;
    uint32_t target_writes = (uint32_t)duration_sec * writes_per_sec;

    // Pre-fill ring buffer (~50ms)
    for (int i = 0; i < 50 / TONE_BUF_MS && app->dev_connected; i++) {
        uint32_t buf_size = tone_chunk_bytes(sample_rate, bit_depth, TONE_BUF_MS, &frame_remainder);
        tone_gen_fill(&gen, tone_buf, buf_size);
        uac2_host_device_write(app->uac2_dev, tone_buf, buf_size, 100);
    }

    int64_t deadline = write_deadline_us(target_writes);
    while (app->dev_connected && writes < target_writes && esp_timer_get_time() < deadline) {
        uint32_t buf_size = tone_chunk_bytes(sample_rate, bit_depth, TONE_BUF_MS, &frame_remainder);
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
    tone_fade_out_and_drain(app, &gen, tone_buf,
                            tone_chunk_bytes(sample_rate, bit_depth, TONE_BUF_MS, &frame_remainder));

    t0 = esp_timer_get_time();
    uac2_host_device_stop(app->uac2_dev);
    t1 = esp_timer_get_time();
    ESP_LOGI(TAG, "Stream stopped after %d sec (stop took %" PRId64 " us)", seconds, t1 - t0);
    log_event_counters("stream_tone");
    return seconds;
}

static void run_live_suspend_resume_check(app_state_t *app)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== LIVE CHECK: Suspend/resume ===");

    reset_event_counters();
    uac2_host_stream_config_t stream_cfg = {
        .sample_freq = 48000,
        .channels = TONE_CHANNELS,
        .bit_resolution = 24,
    };

    esp_err_t err = start_playback(app->uac2_dev, &stream_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "=== LIVE CHECK: Suspend/resume FAIL (start: %s) ===",
                 esp_err_to_name(err));
        return;
    }

    uint8_t tone_buf[TONE_BUF_SIZE];
    uint32_t chunk = max_tone_chunk_bytes(48000, 24, TONE_BUF_MS);
    tone_gen_t warmup;
    tone_init(&warmup, 48000, 24, TONE_FREQ_HZ);
    (void)prefill_tone_buffer(app->uac2_dev, &warmup, 50);
    uint32_t writes_per_sec = 1000 / TONE_BUF_MS;
    uint32_t warmup_writes = feed_tone_for_writes(app, app->uac2_dev, &warmup,
                                                  writes_per_sec, "warmup");
    tone_fade_out_and_drain(app, &warmup, tone_buf, chunk);

    esp_err_t suspend_err = uac2_host_device_suspend(app->uac2_dev);
    ESP_LOGI(TAG, "  suspend: %s", esp_err_to_name(suspend_err));

    tone_gen_t gen;
    tone_gen_config_t tone_cfg = {
        .sample_rate = 48000,
        .channels = TONE_CHANNELS,
        .bit_depth = 24,
        .frequency = TONE_FREQ_HZ,
        .amplitude = TONE_AMPLITUDE,
    };
    tone_gen_init(&gen, &tone_cfg);
    tone_gen_fill(&gen, tone_buf, TONE_BUF_SIZE);
    esp_err_t write_while_suspended = uac2_host_device_write(app->uac2_dev, tone_buf, TONE_BUF_SIZE, 50);
    ESP_LOGI(TAG, "  write while suspended: %s", esp_err_to_name(write_while_suspended));

    esp_err_t suspend_again_err = uac2_host_device_suspend(app->uac2_dev);
    ESP_LOGI(TAG, "  suspend again: %s", esp_err_to_name(suspend_again_err));

    vTaskDelay(pdMS_TO_TICKS(250));

    esp_err_t resume_err = uac2_host_device_resume(app->uac2_dev);
    ESP_LOGI(TAG, "  resume: %s", esp_err_to_name(resume_err));
    esp_err_t resume_again_err = uac2_host_device_resume(app->uac2_dev);
    ESP_LOGI(TAG, "  resume again: %s", esp_err_to_name(resume_again_err));

    uint32_t resumed_writes = 0;
    if (resume_err == ESP_OK || resume_again_err == ESP_OK) {
        tone_gen_t resumed;
        tone_init(&resumed, 48000, 24, TONE_ALT_HZ);
        resumed_writes = feed_tone_for_writes(app, app->uac2_dev, &resumed,
                                              2 * writes_per_sec, "resumed");
        tone_fade_out_and_drain(app, &resumed, tone_buf, chunk);
    }

    uac2_host_device_stop(app->uac2_dev);
    log_event_counters("suspend_resume");

    bool pass = app->dev_connected &&
                warmup_writes >= writes_per_sec &&
                suspend_err == ESP_OK &&
                write_while_suspended == ESP_ERR_INVALID_STATE &&
                suspend_again_err == ESP_OK &&
                resume_err == ESP_OK &&
                resume_again_err == ESP_OK &&
                resumed_writes >= 2 * writes_per_sec &&
                evt_errors == 0 &&
                evt_disconnects == 0;
    ESP_LOGI(TAG, "=== LIVE CHECK: Suspend/resume %s ===", pass ? "PASS" : "FAIL");
}

#if CONFIG_UAC2_ALLOW_DUPLEX
#define DUPLEX_STREAM_SEC   CONFIG_UAC2_DUPLEX_TEST_SEC

/**
 * Playback and capture at once (CONFIG_UAC2_ALLOW_DUPLEX, with the custom FIFO
 * split from app_main): write a tone while draining the capture ring buffer
 * for DUPLEX_STREAM_SEC. Returns whether both streams kept their rate.
 */
static bool duplex_stream(app_state_t *app, uac2_host_device_handle_t rx_dev, tone_gen_t *gen)
{
    uac2_host_stream_info_t rx_info;
    if (uac2_host_device_get_stream_info(rx_dev, &rx_info) != ESP_OK) {
        ESP_LOGE(TAG, "  duplex: no RX stream info");
        return false;
    }
    uint32_t rx_bytes_per_sec = rx_info.sample_freq * rx_info.channels * rx_info.sub_slot_size;
    ESP_LOGI(TAG, "  duplex: RX alt %d, %dch %d-bit in %d-byte slots, %" PRIu32 " B/s",
             rx_info.alt_setting, rx_info.channels, rx_info.bit_resolution,
             rx_info.sub_slot_size, rx_bytes_per_sec);

    uint32_t frame_remainder = 0;
    uint32_t writes_per_sec = 1000 / TONE_BUF_MS;
    uint8_t tone_buf[TONE_BUF_SIZE];
    uint8_t rx_buf[1024];

    uint32_t writes = 0;
    uint64_t rx_bytes = 0;
    uint64_t rx_bytes_after_warmup = 0;
    int64_t start = esp_timer_get_time();
    int64_t end = start + (int64_t)DUPLEX_STREAM_SEC * 1000000;
    int64_t warmup_end = start + 1000000;
    int64_t warmup_bytes_at = 0;
    int last_sec = 0;
    while (app->dev_connected && esp_timer_get_time() < end) {
        uint32_t buf_size = tone_chunk_bytes(48000, 24, TONE_BUF_MS, &frame_remainder);
        tone_gen_fill(gen, tone_buf, buf_size);
        esp_err_t err = uac2_host_device_write(app->uac2_dev, tone_buf, buf_size, 100);
        if (err == ESP_ERR_INVALID_STATE) break;
        if (err == ESP_OK) writes++;

        uint32_t n = 0;
        while ((err = uac2_host_device_read(rx_dev, rx_buf, sizeof(rx_buf), &n, 0)) == ESP_OK && n > 0) {
            rx_bytes += n;
        }
        if (err == ESP_ERR_INVALID_STATE) break;

        int64_t now = esp_timer_get_time();
        if (warmup_bytes_at == 0 && now >= warmup_end) {
            warmup_bytes_at = now;
            rx_bytes_after_warmup = rx_bytes;
        }
        int sec = (int)((now - start) / 1000000);
        if (sec != last_sec) {
            last_sec = sec;
            ESP_LOGI(TAG, "  duplex %d sec: %" PRIu32 " writes, %" PRIu64 " bytes captured",
                     sec, writes, rx_bytes);
        }
    }

    int64_t elapsed_us = esp_timer_get_time() - start;
    int64_t measured_us = esp_timer_get_time() - warmup_bytes_at;
    uint64_t measured_bytes = rx_bytes - rx_bytes_after_warmup;
    double rx_rate = measured_us > 0 ? (double)measured_bytes * 1e6 / (double)measured_us : 0.0;
    // Writes block once the ring buffer is full, so playback that kept the
    // device's rate accepted one write per 10 ms of device clock, plus at most
    // what the ring buffer took up front: too few means a stall, too many
    // means audio left too fast. The device clock is its feedback (nominal
    // without), with 0.05% for feedback jitter.
    uint32_t fb_q16 = uac2_host_device_get_feedback(app->uac2_dev);
    double device_rate = fb_q16 ? (double)fb_q16 / 65536.0 / 48.0 : 1.0;
    double want_writes = (double)elapsed_us / 1000.0 / TONE_BUF_MS * device_rate;
    uint32_t ring_writes = 0;
    uac2_host_buffer_status_t tx_buf;
    if (uac2_host_device_get_buffer_status(app->uac2_dev, &tx_buf) == ESP_OK) {
        ring_writes = tx_buf.ring_capacity / TONE_BUF_SIZE;
    }
    double slack = ring_writes + 2 + want_writes * 0.0005;
    bool tx_ok = writes + writes_per_sec / 2 >= want_writes &&
                 writes <= want_writes + slack;
    // Capture runs on the same device clock
    double rx_want = rx_bytes_per_sec * device_rate;
    bool rx_ok = warmup_bytes_at != 0 && rx_rate > rx_want * 0.995 && rx_rate < rx_want * 1.005;
    ESP_LOGI(TAG, "  duplex: %" PRIu32 " writes for %.0f expected at feedback %.4f (+%.0f slack) (%s), "
             "capture %.0f B/s for %.0f expected (%s)",
             writes, want_writes, device_rate * 48.0, slack, tx_ok ? "ok" : "off",
             rx_rate, rx_want, rx_ok ? "ok" : "off");
    // Both rate checks scale with elapsed time, so a loop that broke out early
    // (a stream left ACTIVE) could still pass them
    bool full = elapsed_us >= (int64_t)DUPLEX_STREAM_SEC * 1000000;
    if (!full) {
        ESP_LOGE(TAG, "  duplex: ended after %" PRId64 " ms of %d s", elapsed_us / 1000, DUPLEX_STREAM_SEC);
    }
    return tx_ok && rx_ok && full;
}
#endif

#if CONFIG_UAC2_TEST_RESTART_SEC > 0
/*
 * LIVE CHECK: restart mid-stream. Calls esp_restart() CONFIG_UAC2_TEST_RESTART_SEC
 * into Test 12; listen for the next boot's tones. With
 * CONFIG_UAC2_STOP_STREAMS_ON_RESTART the driver stops the stream first.
 */
static void restart_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(CONFIG_UAC2_TEST_RESTART_SEC * 1000));
#if CONFIG_UAC2_STOP_STREAMS_ON_RESTART
    ESP_LOGI(TAG, "=== LIVE CHECK: Restart mid-stream: esp_restart() (driver stops streams first) ===");
#else
    ESP_LOGI(TAG, "=== LIVE CHECK: Restart mid-stream: esp_restart() (streams left running) ===");
#endif
    esp_restart();
}
#endif

static void run_live_duplex_guard_check(app_state_t *app)
{
    ESP_LOGI(TAG, "");
#if CONFIG_UAC2_ALLOW_DUPLEX
    ESP_LOGI(TAG, "=== LIVE CHECK: Duplex streaming (%d s) ===", DUPLEX_STREAM_SEC);
#else
    ESP_LOGI(TAG, "=== LIVE CHECK: Duplex guard ===");
#endif

    uac2_device_info_t info;
    if (uac2_host_device_get_info(app->uac2_dev, &info) != ESP_OK) {
        ESP_LOGE(TAG, "=== LIVE CHECK: Duplex guard FAIL (device info) ===");
        return;
    }

    uint8_t rx_iface_num = 0;
    uac2_host_stream_config_t rx_cfg = {0};
    if (!find_capture_iface_config(&info, &rx_iface_num, &rx_cfg)) {
        ESP_LOGI(TAG, "=== LIVE CHECK: Duplex guard SKIP (no capture iface) ===");
        return;
    }

    uac2_host_device_config_t rx_dev_cfg = {
        .addr = app->dev_addr,
        .iface_num = rx_iface_num,
        .buffer_size = 0,
        .buffer_threshold = 0,
        .callback = device_event_cb,
        .callback_arg = app,
    };

    uac2_host_device_handle_t rx_dev = NULL;
    esp_err_t err = uac2_host_device_open(&rx_dev_cfg, &rx_dev);
    ESP_LOGI(TAG, "  open RX iface %d: %s", rx_iface_num, esp_err_to_name(err));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "=== LIVE CHECK: Duplex guard FAIL (open RX iface) ===");
        return;
    }

    reset_event_counters();
    uac2_host_stream_config_t tx_cfg = {
        .sample_freq = 48000,
        .channels = TONE_CHANNELS,
        .bit_resolution = 24,
    };

    tone_gen_t gen;
    tone_init(&gen, 48000, 24, TONE_FREQ_HZ);
    esp_err_t tx_start_err = start_playback(app->uac2_dev, &tx_cfg);
    ESP_LOGI(TAG, "  start TX: %s", esp_err_to_name(tx_start_err));
    if (tx_start_err == ESP_OK) {
        (void)prefill_tone_buffer(app->uac2_dev, &gen, 50);
        (void)feed_tone_for_writes(app, app->uac2_dev, &gen, (500 / TONE_BUF_MS), NULL);
    }

    esp_err_t rx_start_err = uac2_host_device_start(rx_dev, &rx_cfg);
    ESP_LOGI(TAG, "  start RX while TX active: %s", esp_err_to_name(rx_start_err));
#if CONFIG_UAC2_ALLOW_DUPLEX
    bool streamed = tx_start_err == ESP_OK && rx_start_err == ESP_OK && duplex_stream(app, rx_dev, &gen);
#endif
    if (rx_start_err == ESP_OK) {
        uac2_host_device_stop(rx_dev);
    }

    if (tx_start_err == ESP_OK) {
        uint8_t tone_buf[TONE_BUF_SIZE];
        tone_fade_out_and_drain(app, &gen, tone_buf, max_tone_chunk_bytes(48000, 24, TONE_BUF_MS));
        uac2_host_device_stop(app->uac2_dev);
    }
    close_device_with_retry(rx_dev, "Duplex RX");

#if CONFIG_UAC2_ALLOW_DUPLEX
    bool pass = app->dev_connected &&
                streamed &&
                evt_errors == 0 &&
                evt_stream_errors == 0 &&
                evt_disconnects == 0;
    log_event_counters("duplex_stream");
    ESP_LOGI(TAG, "=== LIVE CHECK: Duplex streaming %s ===", pass ? "PASS" : "FAIL");
#else
    bool pass = app->dev_connected &&
                tx_start_err == ESP_OK &&
                rx_start_err == ESP_ERR_NOT_SUPPORTED &&
                evt_errors == 0 &&
                evt_disconnects == 0;
    log_event_counters("duplex_guard");
    ESP_LOGI(TAG, "=== LIVE CHECK: Duplex guard %s ===", pass ? "PASS" : "FAIL");
#endif
}

/**
 * Against the ep0-hang simulator, which never answers its first clock-valid
 * request and soft-disconnects 12 s later: the request must time out and be
 * abandoned, the next request must be refused while it is still in flight,
 * close must succeed by orphaning it, reopening must be refused until it
 * completes, and the device must be freed and re-enumerate after the
 * disconnect completes the orphaned URB. Leaves the device closed.
 */
static void run_live_ep0_hang_check(app_state_t *app)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== LIVE CHECK: Control-transfer timeout and orphan reclaim (sim ep0-hang) ===");

    bool valid = false;
    int64_t t0 = esp_timer_get_time();
    esp_err_t hang_err = uac2_host_device_get_clock_valid(app->uac2_dev, &valid);
    int64_t waited_ms = (esp_timer_get_time() - t0) / 1000;

    uint32_t rate = 0;
    esp_err_t next_err = uac2_host_device_get_sample_rate(app->uac2_dev, &rate);

    (void)ulTaskNotifyTake(pdTRUE, 0);
    esp_err_t close_err = uac2_host_device_close(app->uac2_dev);
    if (close_err == ESP_OK) {
        app->uac2_dev = NULL;
    }

    uac2_host_device_config_t dev_cfg = {
        .addr = app->dev_addr,
        .iface_num = app->iface_num,
        .callback = device_event_cb,
        .callback_arg = app,
    };
    uac2_host_device_handle_t again = NULL;
    esp_err_t reopen_err = uac2_host_device_open(&dev_cfg, &again);
    if (reopen_err == ESP_OK) {
        close_device_with_retry(again, "ep0-hang reopen");
    }
    ESP_LOGI(TAG, "  clock-valid request: %s after %" PRId64 " ms; next request: %s; "
             "close: %s; reopen while pending: %s",
             esp_err_to_name(hang_err), waited_ms, esp_err_to_name(next_err),
             esp_err_to_name(close_err), esp_err_to_name(reopen_err));

    // ALL_FREE only follows once the driver reclaimed the orphaned URB and
    // dropped its last reference to the disconnected device.
    bool all_free = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(30000)) != 0;
    bool reconnected = false;
    for (int i = 0; i < 100 && !reconnected; i++) {
        portENTER_CRITICAL(&s_app_lock);
        reconnected = app->connect_pending;
        portEXIT_CRITICAL(&s_app_lock);
        if (!reconnected) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    ESP_LOGI(TAG, "  device freed after disconnect: %s; re-enumerated: %s",
             all_free ? "yes" : "NO", reconnected ? "yes (full suite runs next)" : "NO");

    bool pass = hang_err == ESP_ERR_TIMEOUT &&
                waited_ms >= CONFIG_UAC2_CTRL_XFER_TIMEOUT_MS - 100 &&
                next_err == ESP_ERR_INVALID_STATE &&
                close_err == ESP_OK &&
                reopen_err == ESP_ERR_INVALID_STATE &&
                all_free && reconnected;
    ESP_LOGI(TAG, "=== LIVE CHECK: Control-transfer timeout and orphan reclaim %s ===",
             pass ? "PASS" : "FAIL");
}

// Calls set_sample_rate() through a second handle on the same device until
// told to stop, tallying the results.
typedef struct {
    uac2_host_device_handle_t dev;
    volatile bool stop;
    uint32_t ok;
    uint32_t rejected;
    uint32_t other;
    TaskHandle_t waiter;
} rate_hammer_t;

static void rate_hammer_task(void *arg)
{
    rate_hammer_t *h = (rate_hammer_t *)arg;
    static const uint32_t rates[2] = {44100, 48000};
    for (uint32_t i = 0; !h->stop; i++) {
        esp_err_t err = uac2_host_device_set_sample_rate(h->dev, rates[i & 1]);
        if (err == ESP_OK) {
            h->ok++;
        } else if (err == ESP_ERR_INVALID_STATE) {
            h->rejected++;
        } else {
            h->other++;
            ESP_LOGW(TAG, "  hammer set_sample_rate: %s", esp_err_to_name(err));
        }
        vTaskDelay(1);
    }
    xTaskNotifyGive(h->waiter);
    vTaskDelete(NULL);
}

static void run_live_sample_rate_guard_check(app_state_t *app)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== LIVE CHECK: Sample-rate guard ===");

    // The clock belongs to the physical device, so a second interface handle
    // (the capture interface, never started) can try to change it.
    uac2_device_info_t info;
    uint8_t rx_iface_num = 0;
    uac2_host_stream_config_t rx_cfg = {0};
    if (uac2_host_device_get_info(app->uac2_dev, &info) != ESP_OK ||
        !find_capture_iface_config(&info, &rx_iface_num, &rx_cfg)) {
        ESP_LOGI(TAG, "=== LIVE CHECK: Sample-rate guard SKIP (no second interface) ===");
        return;
    }
    uac2_host_device_config_t rx_dev_cfg = {
        .addr = app->dev_addr,
        .iface_num = rx_iface_num,
        .callback = device_event_cb,
        .callback_arg = app,
    };
    uac2_host_device_handle_t rx_dev = NULL;
    esp_err_t err = uac2_host_device_open(&rx_dev_cfg, &rx_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "=== LIVE CHECK: Sample-rate guard FAIL (open iface %d: %s) ===",
                 rx_iface_num, esp_err_to_name(err));
        return;
    }

    reset_event_counters();
    uac2_host_stream_config_t tx_cfg = {
        .sample_freq = 48000,
        .channels = TONE_CHANNELS,
        .bit_resolution = 24,
    };

    // Sequential: rejected while a stream exists, accepted once it stopped
    esp_err_t set_active = ESP_FAIL;
    esp_err_t set_idle = ESP_FAIL;
    uint32_t rate_active = 0;
    uint8_t tone_buf[TONE_BUF_SIZE];
    uint32_t chunk = max_tone_chunk_bytes(48000, 24, TONE_BUF_MS);
    tone_gen_t gen;
    tone_init(&gen, 48000, 24, TONE_FREQ_HZ);
    esp_err_t start_err = start_playback(app->uac2_dev, &tx_cfg);
    if (start_err == ESP_OK) {
        (void)prefill_tone_buffer(app->uac2_dev, &gen, 50);
        set_active = uac2_host_device_set_sample_rate(rx_dev, 44100);
        (void)uac2_host_device_get_sample_rate(app->uac2_dev, &rate_active);
        tone_fade_out_and_drain(app, &gen, tone_buf, chunk);
        uac2_host_device_stop(app->uac2_dev);
        set_idle = uac2_host_device_set_sample_rate(rx_dev, 44100);
    }
    ESP_LOGI(TAG, "  start TX: %s; set 44.1k via iface %d while streaming: %s (clock %" PRIu32
             " Hz); after stop: %s", esp_err_to_name(start_err), rx_iface_num,
             esp_err_to_name(set_active), rate_active, esp_err_to_name(set_idle));

    // Concurrent: another core keeps changing the clock while TX starts and
    // stops; it must never change under an active stream.
    rate_hammer_t hammer = {
        .dev = rx_dev,
        .waiter = xTaskGetCurrentTaskHandle(),
    };
    int starts_ok = 0;
    int mismatches = 0;
    const int cycles = 8;
    (void)ulTaskNotifyTake(pdTRUE, 0);
    bool hammer_running = xTaskCreatePinnedToCore(rate_hammer_task, "rate_hammer", 4096, &hammer,
                                                  UAC2_TASK_PRIORITY + 1, NULL, 1) == pdPASS;
    for (int i = 0; hammer_running && i < cycles && app->dev_connected; i++) {
        if (start_playback(app->uac2_dev, &tx_cfg) != ESP_OK) {
            continue;
        }
        starts_ok++;
        tone_init(&gen, 48000, 24, TONE_FREQ_HZ);
        (void)feed_tone_for_writes(app, app->uac2_dev, &gen, 300 / TONE_BUF_MS, NULL);
        uint32_t rate = 0;
        if (uac2_host_device_get_sample_rate(app->uac2_dev, &rate) != ESP_OK || rate != 48000) {
            ESP_LOGE(TAG, "  cycle %d: clock is %" PRIu32 " Hz under a 48 kHz stream", i, rate);
            mismatches++;
        }
        tone_fade_out_and_drain(app, &gen, tone_buf, chunk);
        uac2_host_device_stop(app->uac2_dev);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    hammer.stop = true;
    // hammer lives on this stack, so wait for it unconditionally; its calls
    // always return (control requests time out)
    if (hammer_running) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    ESP_LOGI(TAG, "  concurrent: %d/%d starts, %d clock mismatches; other core: %" PRIu32
             " set OK, %" PRIu32 " rejected, %" PRIu32 " other errors",
             starts_ok, cycles, mismatches, hammer.ok, hammer.rejected, hammer.other);

    close_device_with_retry(rx_dev, "Sample-rate guard");

    // hammer.rejected > 0 shows the two tasks actually overlapped
    bool pass = app->dev_connected &&
                start_err == ESP_OK &&
                set_active == ESP_ERR_INVALID_STATE &&
                rate_active == 48000 &&
                set_idle == ESP_OK &&
                hammer_running &&
                starts_ok == cycles &&
                mismatches == 0 &&
                hammer.rejected > 0 &&
                hammer.other == 0 &&
                evt_stream_errors == 0;
    log_event_counters("sample_rate_guard");
    ESP_LOGI(TAG, "=== LIVE CHECK: Sample-rate guard %s ===", pass ? "PASS" : "FAIL");
}

static void run_live_driver_reinstall_check(app_state_t *app)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== LIVE CHECK: Driver reinstall with device attached ===");

    close_device_with_retry(app->uac2_dev, "Playback");
    app->uac2_dev = NULL;

    portENTER_CRITICAL(&s_app_lock);
    app->reinstall_check_active = true;
    app->reinstall_reported = false;
    portEXIT_CRITICAL(&s_app_lock);

    // Uninstall waits for the background event task to exit; install must
    // then report the still-attached device from its first event pass.
    int64_t t0 = esp_timer_get_time();
    esp_err_t un_err = uac2_host_uninstall();
    int64_t t1 = esp_timer_get_time();
    esp_err_t in_err = un_err == ESP_OK ? uac2_host_install(&s_driver_cfg) : ESP_ERR_INVALID_STATE;
    ESP_LOGI(TAG, "  uninstall: %s (%" PRId64 " ms), install: %s",
             esp_err_to_name(un_err), (t1 - t0) / 1000, esp_err_to_name(in_err));

    bool reported = false;
    uint8_t addr = 0;
    uint8_t iface = 0;
    for (int i = 0; in_err == ESP_OK && i < 50 && !reported; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
        portENTER_CRITICAL(&s_app_lock);
        reported = app->reinstall_reported;
        addr = app->reinstall_addr;
        iface = app->reinstall_iface;
        portEXIT_CRITICAL(&s_app_lock);
    }

    esp_err_t open_err = ESP_ERR_NOT_FOUND;
    int seconds = 0;
    if (reported) {
        app->dev_addr = addr;
        app->iface_num = iface;
        uac2_host_device_config_t dev_cfg = {
            .addr = addr,
            .iface_num = iface,
            .callback = device_event_cb,
            .callback_arg = app,
        };
        open_err = uac2_host_device_open(&dev_cfg, &app->uac2_dev);
        if (open_err == ESP_OK) {
            seconds = stream_tone(app, 48000, 24, TONE_FREQ_HZ, 2);
        } else {
            app->uac2_dev = NULL;
        }
    }

    portENTER_CRITICAL(&s_app_lock);
    app->reinstall_check_active = false;
    portEXIT_CRITICAL(&s_app_lock);

    ESP_LOGI(TAG, "  re-reported: %s, reopen: %s, streamed %d s",
             reported ? "yes" : "NO", esp_err_to_name(open_err), seconds);
    bool pass = un_err == ESP_OK && in_err == ESP_OK && reported &&
                open_err == ESP_OK && seconds >= 2 && app->dev_connected;
    ESP_LOGI(TAG, "=== LIVE CHECK: Driver reinstall %s ===", pass ? "PASS" : "FAIL");
}

// ── Test suite ───────────────────────────────────────────────────

static void run_stream_tests(app_state_t *app)
{
    esp_err_t err;
    int sec;
    uac2_device_info_t info;

    if (uac2_host_device_get_info(app->uac2_dev, &info) == ESP_OK &&
        !assert_simulator_profile_expectations(&info, app->iface_num)) {
        ESP_LOGE(TAG, "Aborting test suite: simulator profile assertions failed");
        return;
    }

    // ── Test 1: Basic 48kHz/24-bit streaming (10s) ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 1: Basic 48kHz/24-bit streaming (10 sec) ===");
    sec = stream_tone(app, 48000, 24, TONE_FREQ_HZ, 10);
    if (!app->dev_connected) return;
    ESP_LOGI(TAG, "=== TEST 1: %s (%d sec) ===", sec >= 10 ? "PASS" : "FAIL", sec);

    // ── Test 2: Volume/mute control during streaming ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 2: Volume/mute control during streaming ===");
    {
        uac2_device_info_t info;
        const uac2_feature_unit_t *fu = NULL;
        uint32_t volume_map = 0;
        uint32_t mute_map = 0;
        int volume_channel = -1;    // -1: no settable control, step skipped
        int mute_channel = -1;

        if (uac2_host_device_get_info(app->uac2_dev, &info) == ESP_OK) {
            fu = resolve_feature_unit_for_iface_info(&info, app->iface_num);
            if (fu) {
                // Controls the host can set; read-only ones refuse set_*()
                volume_map = fu->volume_wr_ch_map;
                mute_map = fu->mute_wr_ch_map;
                volume_channel = first_supported_channel(volume_map);
                mute_channel = first_supported_channel(mute_map);
            }
        }

        ESP_LOGI(TAG, "  Control channels: mute=%d volume=%d (mute_map=0x%08" PRIX32
                 " volume_map=0x%08" PRIX32 ")",
                 mute_channel, volume_channel, mute_map, volume_map);

        uac2_host_stream_config_t scfg = {
            .sample_freq = TONE_SAMPLE_RATE,
            .channels = TONE_CHANNELS,
            .bit_resolution = TONE_BIT_DEPTH,
        };
        err = start_playback(app->uac2_dev, &scfg);
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
            // Without its feature unit there is nothing to exercise; that is
            // a topology failure, not a device without controls
            bool t2_pass = fu != NULL;
            if (!fu) ESP_LOGE(TAG, "  Playback feature unit topology not found");

            for (int i = 0; i < 50 / TONE_BUF_MS && app->dev_connected; i++) {
                tone_gen_fill(&gen, tbuf, bsz);
                uac2_host_device_write(app->uac2_dev, tbuf, bsz, 100);
            }

            uint32_t writes = 0;
            uint32_t wps = 1000 / TONE_BUF_MS;
            int64_t deadline = write_deadline_us(3 * wps);
            while (app->dev_connected && writes < 3 * wps && esp_timer_get_time() < deadline) {
                tone_gen_fill(&gen, tbuf, bsz);
                esp_err_t wr = uac2_host_device_write(app->uac2_dev, tbuf, bsz, 100);
                if (wr == ESP_ERR_INVALID_STATE) {
                    ESP_LOGE(TAG, "  Stream stopped unexpectedly after %" PRIu32 " writes", writes);
                    t2_pass = false;
                    break;
                }
                // Each control step below runs once, on the write that reaches its count
                if (wr != ESP_OK) continue;
                writes++;

                if (writes == wps / 2) {
                    if (mute_channel >= 0) {
                        err = uac2_host_device_set_mute(app->uac2_dev, (uint8_t)mute_channel, true);
                        ESP_LOGI(TAG, "  Set mute ch%d=true: %s", mute_channel, esp_err_to_name(err));
                        if (err != ESP_OK) t2_pass = false;
                        bool muted = false;
                        err = uac2_host_device_get_mute(app->uac2_dev, (uint8_t)mute_channel, &muted);
                        ESP_LOGI(TAG, "  Get mute ch%d: %s %s",
                                 mute_channel, muted ? "MUTED" : "unmuted", esp_err_to_name(err));
                        if (err != ESP_OK || !muted) t2_pass = false;
                    }
                }
                if (writes == wps) {
                    if (mute_channel >= 0) {
                        err = uac2_host_device_set_mute(app->uac2_dev, (uint8_t)mute_channel, false);
                        ESP_LOGI(TAG, "  Set mute ch%d=false: %s", mute_channel, esp_err_to_name(err));
                        if (err != ESP_OK) t2_pass = false;
                    }
                    if (volume_channel >= 0) {
                        err = uac2_host_device_set_volume(app->uac2_dev, (uint8_t)volume_channel, -12 * 256);
                        ESP_LOGI(TAG, "  Set volume ch%d=-12dB: %s", volume_channel, esp_err_to_name(err));
                        if (err != ESP_OK) t2_pass = false;
                    }
                }
                if (writes == wps + wps / 2) {
                    if (volume_channel >= 0) {
                        int16_t vol = 0;
                        err = uac2_host_device_get_volume(app->uac2_dev, (uint8_t)volume_channel, &vol);
                        ESP_LOGI(TAG, "  Get volume ch%d: %d (%.2f dB) %s",
                                 volume_channel, vol, vol / 256.0, esp_err_to_name(err));
                        // -12 dB was set at writes == wps
                        if (err != ESP_OK || vol != -12 * 256) t2_pass = false;
                    }
                    if (mute_channel >= 0) {
                        bool muted = true;
                        err = uac2_host_device_get_mute(app->uac2_dev, (uint8_t)mute_channel, &muted);
                        ESP_LOGI(TAG, "  Get mute ch%d: %s %s",
                                 mute_channel, muted ? "MUTED" : "unmuted", esp_err_to_name(err));
                        // mute was cleared at writes == wps
                        if (err != ESP_OK || muted) t2_pass = false;
                    }
                }
                if (writes == 2 * wps) {
                    if (volume_channel >= 0) {
                        err = uac2_host_device_set_volume(app->uac2_dev, (uint8_t)volume_channel, 0);
                        ESP_LOGI(TAG, "  Set volume ch%d=0dB: %s", volume_channel, esp_err_to_name(err));
                        if (err != ESP_OK) t2_pass = false;
                    }
                }
            }

            tone_fade_out_and_drain(app, &gen, tbuf, bsz);
            uac2_host_device_stop(app->uac2_dev);
            if (!app->dev_connected) return;
            // Every control step above is keyed to a write count; without the
            // full feed some of them never ran.
            if (writes < 3 * wps) t2_pass = false;
            ESP_LOGI(TAG, "=== TEST 2: %s ===", t2_pass ? "PASS" : "FAIL");
        }
    }

    // ── Test 3: Stop/restart cycle ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 3: Stop/restart cycle (5s -> 2s pause -> 5s) ===");
    sec = stream_tone(app, 48000, 24, TONE_LOW_HZ, 5);
    if (!app->dev_connected) return;
    bool t3_pass = (sec >= 5);
    ESP_LOGI(TAG, "  Pausing 2 seconds...");
    for (int i = 0; i < 20 && app->dev_connected; i++)
        vTaskDelay(pdMS_TO_TICKS(100));
    if (!app->dev_connected) return;
    sec = stream_tone(app, 48000, 24, TONE_HIGH_HZ, 5);
    if (!app->dev_connected) return;
    t3_pass = t3_pass && (sec >= 5);
    ESP_LOGI(TAG, "=== TEST 3: %s ===", t3_pass ? "PASS" : "FAIL");

    // ── Test 4: 44.1kHz sample rate ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 4: 44.1kHz streaming (5 sec) ===");
    {
        uac2_host_stream_config_t t4_cfg = {
            .sample_freq = 44100,
            .channels = TONE_CHANNELS,
            .bit_resolution = 24,
        };
        bool t4_plays = selected_stream_plays_fractional(app->uac2_dev, app->iface_num, &t4_cfg);
        esp_err_t set_441_err = uac2_host_device_set_sample_rate(app->uac2_dev, 44100);
        ESP_LOGI(TAG, "  Set 44100 Hz: %s", esp_err_to_name(set_441_err));
        uint32_t readback = 0;
        esp_err_t get_441_err = uac2_host_device_get_sample_rate(app->uac2_dev, &readback);
        ESP_LOGI(TAG, "  Readback: %" PRIu32 " Hz (%s)", readback, esp_err_to_name(get_441_err));
        ESP_LOGI(TAG, "  44.1kHz playable: %s (needs a feedback endpoint or an adaptive/synchronous one)",
                 t4_plays ? "YES" : "NO");

        esp_err_t start_441_err = ESP_OK;
        sec = 0;
        if (t4_plays) {
            sec = stream_tone(app, 44100, 24, TONE_FREQ_HZ, 5);
        } else {
            start_441_err = uac2_host_device_start(app->uac2_dev, &t4_cfg);
            ESP_LOGI(TAG, "  44.1kHz/async-without-feedback start: %s", esp_err_to_name(start_441_err));
            if (start_441_err == ESP_OK) {
                uac2_host_device_stop(app->uac2_dev);
            }
        }
        if (!app->dev_connected) return;
        esp_err_t restore_set_err = uac2_host_device_set_sample_rate(app->uac2_dev, 48000);
        uint32_t restore_readback = 0;
        esp_err_t restore_get_err = uac2_host_device_get_sample_rate(app->uac2_dev, &restore_readback);
        bool t4_pass = (set_441_err == ESP_OK) &&
                       (get_441_err == ESP_OK) &&
                       (readback == 44100) &&
                       ((t4_plays && sec >= 5) ||
                        (!t4_plays && start_441_err == ESP_ERR_NOT_SUPPORTED)) &&
                       (restore_set_err == ESP_OK) &&
                       (restore_get_err == ESP_OK) &&
                       (restore_readback == 48000);
        ESP_LOGI(TAG, "  Restore 48000 Hz: %" PRIu32 " Hz (%s/%s)",
                 restore_readback, esp_err_to_name(restore_set_err),
                 esp_err_to_name(restore_get_err));
        ESP_LOGI(TAG, "=== TEST 4: %s (%d sec) ===", t4_pass ? "PASS" : "FAIL", sec);
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
            err = start_playback(app->uac2_dev, &scfg);
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
                .bit_depth = TONE_BIT_DEPTH, .frequency = TONE_FREQ_HZ,
                .amplitude = TONE_AMPLITUDE,
            };
            tone_gen_init(&gen, &tcfg);
            uint8_t tbuf[TONE_BUF_SIZE];
            uint32_t bsz = (48000 / 1000) * TONE_BUF_MS * TONE_CHANNELS * 3;
            for (int i = 0; i < 100 && app->dev_connected; i++) {
                tone_gen_fill(&gen, tbuf, bsz);
                uac2_host_device_write(app->uac2_dev, tbuf, bsz, 100);
            }
            tone_fade_out_and_drain(app, &gen, tbuf, bsz);
            uac2_host_device_stop(app->uac2_dev);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        if (!app->dev_connected) return;
        ESP_LOGI(TAG, "=== TEST 5: %s ===", t5_pass ? "PASS" : "FAIL");
    }

    // ── Test 6: Feedback presence & clock validity ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 6: Feedback presence & clock validity (5 sec) ===");
    {
        bool clk_before = false;
        uac2_host_device_get_clock_valid(app->uac2_dev, &clk_before);
        ESP_LOGI(TAG, "  Clock valid BEFORE stream: %s", clk_before ? "yes" : "no");

        uac2_host_stream_config_t scfg = {
            .sample_freq = 48000, .channels = TONE_CHANNELS,
            .bit_resolution = TONE_BIT_DEPTH,
        };
        err = start_playback(app->uac2_dev, &scfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "=== TEST 6: FAIL (start: %s) ===", esp_err_to_name(err));
        } else {
            // The alt start() actually chose
            uac2_host_stream_info_t sinfo = {0};
            bool expects_feedback = uac2_host_device_get_stream_info(app->uac2_dev, &sinfo) == ESP_OK &&
                                    sinfo.fb_ep_addr != 0;
            uint32_t expected_feedback = sample_rate_to_feedback_q16(scfg.sample_freq);
            const uint32_t feedback_tolerance = 256;  // ~0.0039 samples/frame
            tone_gen_t gen;
            tone_gen_config_t tcfg = {
                .sample_rate = 48000, .channels = TONE_CHANNELS,
                .bit_depth = TONE_BIT_DEPTH, .frequency = TONE_FREQ_HZ,
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
            bool clock_valid_seen = false;
            uint32_t stable_feedback_samples = 0;
            int64_t deadline = write_deadline_us(5 * wps);
            while (app->dev_connected && writes < 5 * wps && esp_timer_get_time() < deadline) {
                tone_gen_fill(&gen, tbuf, bsz);
                esp_err_t wr = uac2_host_device_write(app->uac2_dev, tbuf, bsz, 100);
                if (wr == ESP_ERR_INVALID_STATE) break;
                // Sample once per second of completed writes, not on retries
                if (wr != ESP_OK) continue;
                writes++;
                if (writes % wps == 0) {
                    uint32_t fb = uac2_host_device_get_feedback(app->uac2_dev);
                    bool clk = false;
                    uac2_host_device_get_clock_valid(app->uac2_dev, &clk);
                    ESP_LOGI(TAG, "  [%" PRIu32 "s] feedback=%" PRIu32 ".%04" PRIu32
                             " (raw=0x%08" PRIX32 "), clock=%s",
                             writes / wps, (uint32_t)(fb >> 16),
                             (uint32_t)((fb & 0xFFFF) * 10000 / 65536),
                             fb, clk ? "valid" : "INVALID");
                    if (fb != 0) {
                        got_feedback = true;
                    }
                    if (clk) {
                        clock_valid_seen = true;
                    }
                    if (clk && fb != 0 &&
                        q16_abs_diff(fb, expected_feedback) <= feedback_tolerance) {
                        stable_feedback_samples++;
                    }
                }
            }
            tone_fade_out_and_drain(app, &gen, tbuf, bsz);
            uac2_host_device_stop(app->uac2_dev);
            bool clk_after = false;
            uac2_host_device_get_clock_valid(app->uac2_dev, &clk_after);
            ESP_LOGI(TAG, "  Feedback received: %s", got_feedback ? "YES" : "NO");
            ESP_LOGI(TAG, "  Feedback expected: %s", expects_feedback ? "YES" : "NO");
            ESP_LOGI(TAG, "  Clock valid seen: %s", clock_valid_seen ? "YES" : "NO");
            ESP_LOGI(TAG, "  Stable feedback samples: %" PRIu32, stable_feedback_samples);
            if (!app->dev_connected) return;
            if (!expects_feedback) {
                ESP_LOGI(TAG, "=== TEST 6: SKIP (selected alt has no feedback endpoint) ===");
            } else if (writes < 5 * wps) {
                // The write deadline ended the feed early: playback stalled
                ESP_LOGI(TAG, "=== TEST 6: FAIL (stream stalled after %" PRIu32 " writes) ===", writes);
            } else if (got_feedback && clock_valid_seen && clk_after && stable_feedback_samples >= 3) {
                ESP_LOGI(TAG, "=== TEST 6: PASS ===");
            } else {
                ESP_LOGI(TAG, "=== TEST 6: FAIL (feedback/clock did not stabilize) ===");
            }
        }
    }

    // ── Test 7: 16-bit alt setting ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 7: 16-bit streaming (5 sec) ===");
    {
        bool t7_pass = true;
        uac2_host_stream_config_t alt16_cfg = {
            .sample_freq = 48000,
            .channels = TONE_CHANNELS,
            .bit_resolution = 16,
        };
        bool has_feedback_16 = selected_stream_has_feedback(app->uac2_dev, app->iface_num, &alt16_cfg);

        sec = stream_tone(app, 48000, 16, TONE_FREQ_HZ, 5);
        if (!app->dev_connected) return;
        t7_pass = (sec >= 5);

        if (!has_feedback_16) {
            uac2_host_stream_config_t no_feedback_fractional_cfg = {
                .sample_freq = 44100,
                .channels = TONE_CHANNELS,
                .bit_resolution = 16,
            };
            // Adaptive/synchronous endpoints play 44.1 kHz without feedback;
            // an asynchronous one must be refused
            bool plays = selected_stream_plays_fractional(app->uac2_dev, app->iface_num,
                                                          &no_feedback_fractional_cfg);
            err = uac2_host_device_start(app->uac2_dev, &no_feedback_fractional_cfg);
            ESP_LOGI(TAG, "  44.1kHz 16-bit/no-feedback start: %s (expected %s)", esp_err_to_name(err),
                     plays ? "ESP_OK, adaptive/synchronous endpoint" : "ESP_ERR_NOT_SUPPORTED");
            if (err == ESP_OK) {
                uac2_host_device_stop(app->uac2_dev);
                t7_pass = t7_pass && plays;
            } else if (plays || err != ESP_ERR_NOT_SUPPORTED) {
                t7_pass = false;
            }
        } else {
            ESP_LOGI(TAG, "  44.1kHz/no-feedback rejection check: SKIP (selected alt has feedback)");
        }

        ESP_LOGI(TAG, "=== TEST 7: %s (%d sec) ===", t7_pass ? "PASS" : "FAIL", sec);
    }

    // ── Test 8: Rapid measurement cycles (9x) ──
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== TEST 8: Rapid measurement cycles (9x) ===");
    {
        bool t8_pass = true;
        float freqs[9];
        for (int i = 0; i < 9; i++) {
            freqs[i] = TONE_CYCLE_HZ(i);
        }

        for (int i = 0; i < 9 && app->dev_connected; i++) {
            int64_t cycle_start = esp_timer_get_time();
            uac2_host_stream_config_t scfg = {
                .sample_freq = 48000, .channels = TONE_CHANNELS,
                .bit_resolution = TONE_BIT_DEPTH,
            };
            err = start_playback(app->uac2_dev, &scfg);
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
            int64_t deadline = write_deadline_us(3 * wps);
            while (app->dev_connected && writes < 3 * wps && esp_timer_get_time() < deadline) {
                tone_gen_fill(&gen, tbuf, bsz);
                esp_err_t wr = uac2_host_device_write(app->uac2_dev, tbuf, bsz, 100);
                if (wr == ESP_ERR_INVALID_STATE) break;
                if (wr == ESP_OK) writes++;
            }
            tone_fade_out_and_drain(app, &gen, tbuf, bsz);
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
        uac2_device_info_t info;
        const uac2_feature_unit_t *fu = NULL;
        uint32_t volume_map = 0;
        uint32_t mute_map = 0;
        uint32_t volume_wr_map = 0;
        uint32_t mute_wr_map = 0;
        if (uac2_host_device_get_info(app->uac2_dev, &info) == ESP_OK) {
            fu = resolve_feature_unit_for_iface_info(&info, app->iface_num);
            if (fu) {
                volume_map = fu->volume_ch_map;
                mute_map = fu->mute_ch_map;
                volume_wr_map = fu->volume_wr_ch_map;
                mute_wr_map = fu->mute_wr_ch_map;
                ESP_LOGI(TAG, "  FU id=%d channels=%d volume_map=0x%08" PRIX32 " mute_map=0x%08" PRIX32
                         " (settable 0x%08" PRIX32 " 0x%08" PRIX32 ")",
                         fu->unit_id, fu->nr_channels, volume_map, mute_map, volume_wr_map, mute_wr_map);
            } else {
                ESP_LOGW(TAG, "  Playback feature unit topology not found");
            }
        } else {
            ESP_LOGW(TAG, "  Could not read device info for Test 9");
        }

        int16_t test_volume = -256;
        for (int ch = 0; ch <= 2; ch++) {
            bool channel_in_range = fu && ch <= fu->nr_channels;
            bool has_volume = channel_map_supports(volume_map, ch);
            bool expect_volume = channel_map_supports(volume_wr_map, ch);
            uac2_volume_range_t vranges[UAC2_MAX_VOLUME_RANGES];
            uint8_t num_vranges = 0;
            esp_err_t range_err = uac2_host_device_get_volume_range(app->uac2_dev, ch, vranges, &num_vranges);
            ESP_LOGI(TAG, "  Volume range ch%d: %s (count=%d)",
                     ch, esp_err_to_name(range_err), num_vranges);
            if (has_volume) {
                if (range_err != ESP_OK || num_vranges == 0) {
                    t9_pass = false;
                }
            } else if (range_err != ESP_ERR_NOT_SUPPORTED &&
                       !(range_err == ESP_ERR_INVALID_ARG && !channel_in_range)) {
                t9_pass = false;
            }

            err = uac2_host_device_set_volume(app->uac2_dev, ch, test_volume);
            ESP_LOGI(TAG, "  Volume ch%d -> %.1f dB: %s",
                     ch, test_volume / 256.0, esp_err_to_name(err));
            if (expect_volume) {
                if (err != ESP_OK) {
                    t9_pass = false;
                    continue;
                }
                int16_t readback = 0;
                esp_err_t get_err = uac2_host_device_get_volume(app->uac2_dev, ch, &readback);
                ESP_LOGI(TAG, "  Volume ch%d readback: %.2f dB [%s]",
                         ch, readback / 256.0, esp_err_to_name(get_err));
                if (get_err != ESP_OK || readback != test_volume) {
                    t9_pass = false;
                }
                esp_err_t restore_err = uac2_host_device_set_volume(app->uac2_dev, ch, 0);
                int16_t restore_readback = 0;
                esp_err_t restore_get_err = uac2_host_device_get_volume(app->uac2_dev, ch, &restore_readback);
                ESP_LOGI(TAG, "  Volume ch%d restore: %.2f dB [%s/%s]",
                         ch, restore_readback / 256.0,
                         esp_err_to_name(restore_err), esp_err_to_name(restore_get_err));
                if (restore_err != ESP_OK || restore_get_err != ESP_OK || restore_readback != 0) {
                    t9_pass = false;
                }
            } else if (has_volume) {
                // Read-only: refused without a request, still readable
                int16_t readback = 0;
                esp_err_t get_err = uac2_host_device_get_volume(app->uac2_dev, ch, &readback);
                if (err != ESP_ERR_NOT_SUPPORTED || get_err != ESP_OK) {
                    t9_pass = false;
                }
            } else if (err != ESP_ERR_NOT_SUPPORTED &&
                       !(err == ESP_ERR_INVALID_ARG && !channel_in_range)) {
                t9_pass = false;
            }
        }
        for (int ch = 0; ch <= 2; ch++) {
            bool channel_in_range = fu && ch <= fu->nr_channels;
            bool has_mute = channel_map_supports(mute_map, ch);
            bool expect_mute = channel_map_supports(mute_wr_map, ch);
            err = uac2_host_device_set_mute(app->uac2_dev, ch, true);
            ESP_LOGI(TAG, "  Mute ch%d=true: %s", ch, esp_err_to_name(err));
            if (expect_mute) {
                if (err != ESP_OK) {
                    t9_pass = false;
                    continue;
                }
                bool muted = false;
                esp_err_t get_err = uac2_host_device_get_mute(app->uac2_dev, ch, &muted);
                ESP_LOGI(TAG, "  Readback ch%d mute: %s [%s]",
                         ch, muted ? "yes" : "no", esp_err_to_name(get_err));
                if (get_err != ESP_OK || !muted) {
                    t9_pass = false;
                }
                esp_err_t restore_err = uac2_host_device_set_mute(app->uac2_dev, ch, false);
                bool restored = true;
                esp_err_t restore_get_err = uac2_host_device_get_mute(app->uac2_dev, ch, &restored);
                ESP_LOGI(TAG, "  Restore ch%d mute: %s [%s/%s]",
                         ch, restored ? "yes" : "no",
                         esp_err_to_name(restore_err), esp_err_to_name(restore_get_err));
                if (restore_err != ESP_OK || restore_get_err != ESP_OK || restored) {
                    t9_pass = false;
                }
            } else if (has_mute) {
                bool muted = false;
                esp_err_t get_err = uac2_host_device_get_mute(app->uac2_dev, ch, &muted);
                if (err != ESP_ERR_NOT_SUPPORTED || get_err != ESP_OK) {
                    t9_pass = false;
                }
            } else if (err != ESP_ERR_NOT_SUPPORTED &&
                       !(err == ESP_ERR_INVALID_ARG && !channel_in_range)) {
                t9_pass = false;
            }
        }
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
            esp_err_t set_err = uac2_host_device_set_sample_rate(app->uac2_dev, rates[i]);
            int64_t t1 = esp_timer_get_time();
            uint32_t readback = 0;
            esp_err_t get_err = uac2_host_device_get_sample_rate(app->uac2_dev, &readback);
            ESP_LOGI(TAG, "  [%d] Set %" PRIu32 " -> read %" PRIu32 " (%" PRId64 " us) %s",
                     i, rates[i], readback, t1 - t0, esp_err_to_name(set_err));
            if (set_err != ESP_OK || get_err != ESP_OK || readback != rates[i]) {
                t10_pass = false;
            }
            uac2_host_stream_config_t switch_cfg = {
                .sample_freq = rates[i],
                .channels = TONE_CHANNELS,
                .bit_resolution = 24,
            };
            bool plays = selected_stream_plays_fractional(app->uac2_dev, app->iface_num, &switch_cfg);
            if (!plays && (rates[i] % 1000) != 0) {
                esp_err_t start_err = uac2_host_device_start(app->uac2_dev, &switch_cfg);
                ESP_LOGI(TAG, "  [%d] %" PRIu32 " Hz/async-without-feedback start: %s",
                         i, rates[i], esp_err_to_name(start_err));
                if (start_err == ESP_OK) {
                    uac2_host_device_stop(app->uac2_dev);
                    t10_pass = false;
                } else if (start_err != ESP_ERR_NOT_SUPPORTED) {
                    t10_pass = false;
                }
            } else {
                sec = stream_tone(app, rates[i], 24, TONE_FREQ_HZ, 2);
                if (!app->dev_connected) return;
                if (sec < 2) t10_pass = false;
            }
        }
        uint32_t restore_readback = 0;
        esp_err_t restore_set_err = uac2_host_device_set_sample_rate(app->uac2_dev, 48000);
        esp_err_t restore_get_err = uac2_host_device_get_sample_rate(app->uac2_dev, &restore_readback);
        if (restore_set_err != ESP_OK || restore_get_err != ESP_OK || restore_readback != 48000) {
            t10_pass = false;
        }
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
        err = start_playback(app->uac2_dev, &scfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "=== TEST 11: FAIL (start: %s) ===", esp_err_to_name(err));
        } else {
            tone_gen_t gen;
            tone_gen_config_t tcfg = {
                .sample_rate = 48000, .channels = TONE_CHANNELS,
                .bit_depth = TONE_BIT_DEPTH, .frequency = TONE_FREQ_HZ,
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
            int64_t deadline = write_deadline_us(2 * wps);
            while (app->dev_connected && writes < 2 * wps && esp_timer_get_time() < deadline) {
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
            deadline = write_deadline_us(2 * wps);
            while (app->dev_connected && writes < 2 * wps && esp_timer_get_time() < deadline) {
                tone_gen_fill(&gen, tbuf, bsz);
                esp_err_t wr = uac2_host_device_write(app->uac2_dev, tbuf, bsz, 100);
                if (wr == ESP_ERR_INVALID_STATE) break;
                if (wr == ESP_OK) writes++;
            }
            log_event_counters("after recovery");
            tone_fade_out_and_drain(app, &gen, tbuf, bsz);
            uac2_host_device_stop(app->uac2_dev);
            if (!app->dev_connected) return;
            // Writes only complete while URBs keep draining the ring buffer, so
            // a full 2 s of recovery writes shows transfers really resumed.
            bool t11_pass = (writes >= 2 * wps) && (evt_stream_errors == 0);
            ESP_LOGI(TAG, "=== TEST 11: %s (recovered %" PRIu32 " writes) ===",
                     t11_pass ? "PASS" : "FAIL", writes);
        }
    }

    run_live_suspend_resume_check(app);
    if (!app->dev_connected) return;

    run_live_duplex_guard_check(app);
    if (!app->dev_connected) return;

    run_live_sample_rate_guard_check(app);
    if (!app->dev_connected) return;

#if CONFIG_UAC2_TEST_RESTART_SEC > 0
    // Once per power-on: the boot after the restart skips it
    if (esp_reset_reason() != ESP_RST_SW) {
        xTaskCreate(restart_task, "restart", 3072, NULL, 5, NULL);
    } else {
        ESP_LOGI(TAG, "=== LIVE CHECK: Restart mid-stream SKIP (this boot came from a restart) ===");
    }
#endif

    // ── Test 12: Long-running stability ──
    ESP_LOGI(TAG, "");
    if (CONFIG_UAC2_TEST12_DURATION_SEC <= 0) {
        ESP_LOGI(TAG, "=== TEST 12: SKIP (CONFIG_UAC2_TEST12_DURATION_SEC=0) ===");
    } else {
        ESP_LOGI(TAG, "=== TEST 12: Long-running stability (%d sec) ===",
                 CONFIG_UAC2_TEST12_DURATION_SEC);
        int t12_sec = stream_tone(app, 48000, 24, TONE_FREQ_HZ, CONFIG_UAC2_TEST12_DURATION_SEC);
        if (!app->dev_connected) {
            // Unplugging during the soak is a deliberate hot-unplug test
            ESP_LOGI(TAG, "=== TEST 12: ended by disconnect after %d sec ===", t12_sec);
        } else {
            ESP_LOGI(TAG, "=== TEST 12: %s (%d of %d sec) ===",
                     t12_sec >= CONFIG_UAC2_TEST12_DURATION_SEC ? "PASS" : "FAIL",
                     t12_sec, CONFIG_UAC2_TEST12_DURATION_SEC);
        }
    }
    if (!app->dev_connected) return;

    run_live_driver_reinstall_check(app);
}

// ── Device task ──────────────────────────────────────────────────

static void device_task(void *arg)
{
    app_state_t *app = (app_state_t *)arg;
    // Set here rather than by the creator, which may only resume after this
    // task has already exited and cleared it.
    app->dev_task_hdl = xTaskGetCurrentTaskHandle();
    app->connect_cycle++;

    // ── Heap tracking ──
#if CONFIG_HEAP_TRACING_STANDALONE
    // Leak hunting (sdkconfig.heaptrace.defaults): record every allocation of
    // this cycle; the ones still held after teardown are dumped below
    static bool heap_trace_ready;
    if (!heap_trace_ready) {
        ESP_ERROR_CHECK(heap_trace_init_standalone(s_heap_trace_records, HEAP_TRACE_RECORDS));
        heap_trace_ready = true;
    }
    ESP_ERROR_CHECK(heap_trace_start(HEAP_TRACE_LEAKS));
#endif
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

    // The ep0-hang simulator leaves its first clock-valid request unanswered,
    // so its check has to run before anything else talks to the device.
    {
        uac2_device_info_t info;
        if (!app->ep0_hang_check_done &&
            uac2_host_device_get_info(app->uac2_dev, &info) == ESP_OK &&
            detect_simulator_profile(&info) == SIM_PROFILE_EP0_HANG) {
            app->ep0_hang_check_done = true;
            run_live_ep0_hang_check(app);
            if (app->uac2_dev) {
                close_device_with_retry(app->uac2_dev, "Playback");
                app->uac2_dev = NULL;
            }
            goto done;
        }
    }

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
        // ALL_FREE for a detached device can only follow this close, so drop
        // anything stale now rather than right before waiting for it.
        (void)ulTaskNotifyTake(pdTRUE, 0);
        close_device_with_retry(app->uac2_dev, "Playback");
        app->uac2_dev = NULL;
    }

done:
    // ── Stack watermark ──
    UBaseType_t stack_hwm = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGI(TAG, "Device task stack high watermark: %u bytes free (of %d)",
             (unsigned)(stack_hwm * sizeof(StackType_t)), DEV_TASK_STACK_SIZE);

    bool expect_all_free = !app->dev_connected;

    // Wait until the USB host library finishes asynchronous device cleanup
    // only when the device actually disconnected. A plain close() with the
    // device still attached will not produce ALL_FREE and should not be
    // treated as a leak signal.
    if (expect_all_free &&
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(HEAP_SETTLE_TIMEOUT_MS)) == 0) {
        ESP_LOGW(TAG, "Timed out waiting for USB_HOST_LIB_EVENT_FLAGS_ALL_FREE");
    } else if (!expect_all_free) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // ── Heap after host cleanup ──
    size_t heap_after = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    int leak = (int)heap_before - (int)heap_after;
    int baseline_delta = (int)app->heap_baseline - (int)heap_after;
    if (expect_all_free) {
        ESP_LOGI(TAG, "Heap after close: %u free (leak this cycle: %d bytes, total from baseline: %d)",
                 (unsigned)heap_after, leak, baseline_delta);
    } else {
        ESP_LOGI(TAG, "Heap after close with device attached: %u free (retained vs detached baseline: %d bytes; leak check skipped)",
                 (unsigned)heap_after, baseline_delta);
    }
    if (expect_all_free && leak > 64) {
        ESP_LOGW(TAG, "POSSIBLE MEMORY LEAK: %d bytes not freed this cycle", leak);
    }
#if CONFIG_HEAP_TRACING_STANDALONE
    heap_trace_stop();
    ESP_LOGI(TAG, "HEAP TRACE cycle %" PRIu32 " (%s): allocations made this cycle and still held:",
             app->connect_cycle, expect_all_free ? "device detached" : "device attached");
    heap_trace_dump();
#endif

    ESP_LOGI(TAG, "--- Device task exiting (cycle %" PRIu32 ") ---", app->connect_cycle);
    app->dev_connected = false;

    portENTER_CRITICAL(&s_app_lock);
    bool pending = app->connect_pending;
    uint8_t next_addr = app->pending_addr;
    uint8_t next_iface = app->pending_iface;
    app->connect_pending = false;
    app->dev_task_hdl = NULL;
    if (!pending) {
        app->task_active = false;
    }
    portEXIT_CRITICAL(&s_app_lock);

    if (pending) {
        ESP_LOGI(TAG, "Starting queued reconnect: addr=%d iface=%d", next_addr, next_iface);
        start_device_task(app, next_addr, next_iface);
    } else {
        ESP_LOGI(TAG, "Disconnect and reconnect USB to run again (cycle %" PRIu32 " next)",
                 app->connect_cycle + 1);
    }
    vTaskDelete(NULL);
}

// Caller must already own the task slot (task_active set under s_app_lock).
static void start_device_task(app_state_t *app, uint8_t addr, uint8_t iface_num)
{
    app->dev_addr = addr;
    app->iface_num = iface_num;
    app->dev_connected = true;

    BaseType_t ret = xTaskCreatePinnedToCore(device_task, "uac2_dev",
                                              DEV_TASK_STACK_SIZE, app,
                                              UAC2_TASK_PRIORITY + 1, NULL, 0);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create device task");
        app->dev_connected = false;
        portENTER_CRITICAL(&s_app_lock);
        app->task_active = false;
        portEXIT_CRITICAL(&s_app_lock);
    }
}

// ── Driver-level callback ────────────────────────────────────────

static void driver_event_cb(uint8_t addr, uint8_t iface_num,
                            const uac2_host_driver_event_t event, void *arg)
{
    app_state_t *app = (app_state_t *)arg;

    if (event == UAC2_HOST_DRIVER_EVENT_TX_CONNECTED) {
        bool start = false;
        bool queued = false;
        bool recorded = false;
        portENTER_CRITICAL(&s_app_lock);
        if (app->reinstall_check_active) {
            // The running device task reopens it itself
            app->reinstall_addr = addr;
            app->reinstall_iface = iface_num;
            app->reinstall_reported = true;
            recorded = true;
        } else if (!app->task_active) {
            app->task_active = true;
            start = true;
        } else {
            // A device task is still running: it may be tearing down after a
            // disconnect, or have been started for a device that is already
            // gone and is failing to open. dev_connected cannot tell these
            // apart, so keep the newest connection and run it when it exits.
            app->pending_addr = addr;
            app->pending_iface = iface_num;
            app->connect_pending = true;
            queued = true;
        }
        portEXIT_CRITICAL(&s_app_lock);

        if (recorded) {
            ESP_LOGI(TAG, "TX interface re-reported after driver reinstall: addr=%d iface=%d",
                     addr, iface_num);
        } else if (start) {
            ESP_LOGI(TAG, "TX interface connected: addr=%d iface=%d", addr, iface_num);
            start_device_task(app, addr, iface_num);
        } else if (queued) {
            ESP_LOGI(TAG, "TX interface connected: addr=%d iface=%d (queued until previous cycle exits)",
                     addr, iface_num);
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
#if CONFIG_UAC2_ALLOW_DUPLEX
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
    // Room for playback and capture at once (the bias presets leave IN 128 bytes)
    host_config.fifo_settings_custom.nptx_fifo_lines = UAC2_HOST_DUPLEX_FIFO_NPTX_LINES;
    host_config.fifo_settings_custom.ptx_fifo_lines = UAC2_HOST_DUPLEX_FIFO_PTX_LINES;
    host_config.fifo_settings_custom.rx_fifo_lines = UAC2_HOST_DUPLEX_FIFO_RX_LINES;
    ESP_LOGI(TAG, "USB FIFO split for duplex: NPTX %d / PTX %d / RX %d lines",
             UAC2_HOST_DUPLEX_FIFO_NPTX_LINES, UAC2_HOST_DUPLEX_FIFO_PTX_LINES,
             UAC2_HOST_DUPLEX_FIFO_RX_LINES);
#else
#warning "CONFIG_UAC2_ALLOW_DUPLEX needs ESP-IDF 5.5+ (fifo_settings_custom); the host will refuse the capture endpoint"
#endif
#endif
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    xTaskNotifyGive((TaskHandle_t)arg);

    while (1) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);

        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            portENTER_CRITICAL(&s_app_lock);
            bool keep_devices = s_app.reinstall_check_active;
            portEXIT_CRITICAL(&s_app_lock);
            if (keep_devices) {
                // The UAC2 driver is being reinstalled with the device attached
                ESP_LOGI(TAG, "No more clients (driver reinstall in progress, keeping devices)");
            } else {
                ESP_LOGW(TAG, "No more clients");
                usb_host_device_free_all();
            }
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "All USB devices freed, ready for reconnect");
            // Read and notify under the lock the device task clears its handle
            // with, so the task cannot be deleted in between. The FromISR
            // variant never yields, which is required inside a critical section.
            portENTER_CRITICAL(&s_app_lock);
            if (s_app.dev_task_hdl) {
                vTaskNotifyGiveFromISR((TaskHandle_t)s_app.dev_task_hdl, NULL);
            }
            portEXIT_CRITICAL(&s_app_lock);
        }
    }
}

// ── Entry point ──────────────────────────────────────────────────

void app_main(void)
{
    ESP_LOGI(TAG, "esp-uac2-host — Comprehensive UAC2 test suite v0.1");
    ESP_LOGI(TAG, "Driver v%d.%d.%d",
             UAC2_HOST_VER_MAJOR, UAC2_HOST_VER_MINOR, UAC2_HOST_VER_PATCH);
    ESP_LOGI(TAG, "Reset reason: %d%s", (int)esp_reset_reason(),
             esp_reset_reason() == ESP_RST_SW ? " (software restart)" : "");

    memset(&s_app, 0, sizeof(s_app));

    // Run self-test against static miniDSP descriptors
    run_descriptor_self_test();

    // Start USB Host Library task
    TaskHandle_t host_task_hdl;
    xTaskCreatePinnedToCore(usb_host_lib_task, "usb_host", 4096,
                            xTaskGetCurrentTaskHandle(),
                            HOST_LIB_TASK_PRIORITY, &host_task_hdl, 0);

    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));

    // Install UAC2 Host driver (creates background event task). Kept for the
    // driver reinstall live check.
    s_driver_cfg = (uac2_host_driver_config_t) {
        .create_background_task = true,
        .task_priority = UAC2_TASK_PRIORITY,
        .stack_size = UAC2_TASK_STACK_SIZE,
        .core_id = 0,
        .callback = driver_event_cb,
        .callback_arg = &s_app,
    };
    ESP_ERROR_CHECK(uac2_host_install(&s_driver_cfg));

    ESP_LOGI(TAG, "USB Host + UAC2 driver initialized, waiting for device...");
}
