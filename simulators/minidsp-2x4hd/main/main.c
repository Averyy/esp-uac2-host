/*
 * miniDSP 2x4 HD Simulator — Full Composite Device
 *
 * Faithful replica of the real miniDSP 2x4 HD on USB in the default build.
 * The no-feedback test build is intentionally reduced for host guard testing:
 * both playback alts omit feedback endpoints and capture streaming alt 1 is
 * removed to stay within ESP32-S3 FIFO limits.
 * The channel-only feature-unit test build keeps the normal endpoints but
 * removes playback master-channel controls so only channels 1 and 2 advertise
 * mute/volume support.
 *
 * - UAC2 audio: playback (IF1, alt 1 = 24-bit, alt 2 = 16-bit)
 * - Feedback endpoint: simulator sends 4-byte 16.16 at FS via TinyUSB's
 *   explicit feedback support; real miniDSP also sends 4-byte 16.16
 * - HID interface (IF4): 64-byte vendor reports, full command protocol
 * - EEPROM state: preset, source, volume, mute, serial, mod tokens
 * - DSP parameter state: routing, PEQ, gain, delay, compressor (flat defaults)
 * - Fault injection modes for robustness testing
 *
 * VID=0x2752 PID=0x0011 bcdDevice=0x0185
 * Config descriptor: 373 bytes — matches real device exactly (5 interfaces)
 * Feedback: 4-byte 16.16 format (matching real XMOS behavior)
 *
 * SPDX-License-Identifier: MIT
 */

#include <string.h>
#include <stdio.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_private/usb_phy.h"
#include "tusb.h"

static const char *TAG = "minidsp-sim";
static const DRAM_ATTR char TAG_ISR[] = "minidsp-sim";  // ISR-safe copy in DRAM

#if CONFIG_MINIDSP_SIM_NO_FEEDBACK_TEST_BUILD && CONFIG_MINIDSP_SIM_PLAYBACK_CHANNEL_ONLY_FU_TEST_BUILD
#error "Select only one miniDSP simulator test profile at a time"
#endif

// ── USB PHY init ──────────────────────────────────────────────────

static usb_phy_handle_t phy_hdl;

static esp_err_t usb_phy_init(void)
{
    usb_phy_config_t phy_conf = {
        .controller = USB_PHY_CTRL_OTG,
        .target = USB_PHY_TARGET_INT,
        .otg_mode = USB_OTG_MODE_DEVICE,
        .otg_speed = USB_PHY_SPEED_UNDEFINED,
    };
    return usb_new_phy(&phy_conf, &phy_hdl);
}

// ── Entity IDs (match real miniDSP exactly) ───────────────────────

#define CLOCK_SOURCE_ID         41      // 0x29
#define CLOCK_SELECTOR_ID       40      // 0x28
#define INPUT_TERMINAL_PB_ID    2       // USB Streaming (playback)
#define FEATURE_UNIT_PB_ID      10      // 0x0A — playback path
#define OUTPUT_TERMINAL_PB_ID   20      // 0x14 — Speaker
#define INPUT_TERMINAL_CAP_ID   1       // Microphone (capture)
#define FEATURE_UNIT_CAP_ID     11      // 0x0B — capture path
#define OUTPUT_TERMINAL_CAP_ID  22      // 0x16 — USB Streaming (capture)

// ── Interface & Endpoint numbers ──────────────────────────────────

enum {
    ITF_NUM_AUDIO_CONTROL = 0,
    ITF_NUM_AUDIO_STREAMING_PB = 1,     // Playback
    ITF_NUM_AUDIO_STREAMING_CAP = 2,    // Capture
    ITF_NUM_DFU = 3,                    // DFU (XMOS firmware update)
    ITF_NUM_HID = 4,                    // HID (miniDSP vendor control)
    ITF_NUM_TOTAL = 5
};

#define EPNUM_AUDIO_OUT     0x01    // Playback data
#define EPNUM_AUDIO_FB      0x81    // Feedback
#define EPNUM_AUDIO_CAP     0x82    // Capture data
#define EPNUM_HID_IN        0x83    // HID responses (matches real device)
#define EPNUM_HID_OUT       0x02    // HID commands

// ── Audio control state ───────────────────────────────────────────

static _Atomic uint32_t current_sample_rate = 48000;
static uint8_t  clock_valid = 1;

// Playback Feature Unit (ID=10): master + 2ch
static bool     pb_mute[3] = {false, false, false};
static int16_t  pb_volume[3] = {0, 0, 0};              // 1/256 dB

// Capture Feature Unit (ID=11): master + 4ch (XMOS default)
static bool     cap_mute[5] = {false, false, false, false, false};
static int16_t  cap_volume[5] = {0, 0, 0, 0, 0};

static volatile uint32_t total_pb_bytes = 0;
static volatile uint32_t pb_frames_received = 0;
static volatile uint8_t  current_pb_alt = 0;     // 0=idle, 1=24-bit, 2=16-bit

#if CFG_TUD_AUDIO_ENABLE_EP_OUT && CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
static uint32_t sample_rate_to_feedback_q16(uint32_t sample_rate_hz)
{
    return (uint32_t)(((uint64_t)sample_rate_hz << 16) / 1000);
}
#endif

static bool no_feedback_mode_enabled(void)
{
#if CONFIG_MINIDSP_SIM_NO_FEEDBACK_TEST_BUILD
    return true;
#else
    return false;
#endif
}

static bool playback_channel_only_fu_mode_enabled(void)
{
#if CONFIG_MINIDSP_SIM_PLAYBACK_CHANNEL_ONLY_FU_TEST_BUILD
    return true;
#else
    return false;
#endif
}

static bool playback_fu_channel_supported(uint8_t channel)
{
    if (playback_channel_only_fu_mode_enabled()) {
        return channel > 0 && channel < 3;
    }
    return channel < 3;
}

#if CONFIG_MINIDSP_SIM_NO_FEEDBACK_TEST_BUILD
#define SIM_PRODUCT_STRING "miniDSP 2x4HD [sim no-fb]"
#elif CONFIG_MINIDSP_SIM_PLAYBACK_CHANNEL_ONLY_FU_TEST_BUILD
#define SIM_PRODUCT_STRING "miniDSP 2x4HD [sim ch-only]"
#else
#define SIM_PRODUCT_STRING "miniDSP 2x4HD"
#endif

static inline uint32_t load_current_sample_rate(void)
{
    return atomic_load_explicit(&current_sample_rate, memory_order_relaxed);
}

static inline void store_current_sample_rate(uint32_t sample_rate_hz)
{
    atomic_store_explicit(&current_sample_rate, sample_rate_hz, memory_order_relaxed);
}

// ── EEPROM state (Flash page 0xFF) ───────────────────────────────

static struct {
    uint8_t  current_preset;        // 0xFF:0xD8 — 0-3
    uint8_t  current_source;        // 0xFF:0xD9 — 0=analog, 1=toslink, 2=USB
    uint8_t  master_volume;         // 0xFF:0xDA — encoded: dB = -0.5 * value
    uint8_t  master_mute;           // 0xFF:0xDB — 0=unmuted, 1=muted
    uint8_t  dirac_bypass;          // 0xFF:0xE0 — 0=enabled, 1=bypassed
    uint8_t  display_idle;          // 0xFF:0xE8
    uint8_t  display_brightness;    // 0xFF:0xE9
    uint8_t  dre_state;             // 0xFF:0xED
    uint16_t serial_number;         // 0xFF:0xFE — actual = 900000 + value (BE)
    uint32_t mod_tokens[4];         // 0xFF:0xC8 — per-preset modification counters
    uint8_t  dsp_id;                // 0xFF:0xA1 — DSP version (100 for 2x4 HD)
} eeprom = {
    .current_preset = 0,
    .current_source = 2,            // USB
    .master_volume = 0,             // 0 dB
    .master_mute = 0,               // unmuted
    .dirac_bypass = 0,
    .display_idle = 30,
    .display_brightness = 3,
    .dre_state = 0,
    .serial_number = 1234,          // actual = 901234
    .mod_tokens = {0, 0, 0, 0},
    .dsp_id = 100,                  // DSP 100 = 2x4 HD
};

// Per-source volume offsets (page 0xFE)
static int8_t source_vol_offsets[3] = {0, 0, 0};   // analog, toslink, USB

// ── DSP parameter state ──────────────────────────────────────────

// Simple flat storage: address → float value
// DSP 100 (2x4 HD) has ~5000 addresses but we only store what's written
#define DSP_PARAM_MAX   256     // max distinct addresses we'll track

static struct {
    uint32_t addr;
    float    value;
} dsp_params[DSP_PARAM_MAX];
static int dsp_param_count = 0;

static float dsp_param_get(uint32_t addr)
{
    for (int i = 0; i < dsp_param_count; i++) {
        if (dsp_params[i].addr == addr) return dsp_params[i].value;
    }
    // Default: 0.0 for most, except PEQ b0 coefficients = 1.0
    return 0.0f;
}

static void dsp_param_set(uint32_t addr, float value)
{
    for (int i = 0; i < dsp_param_count; i++) {
        if (dsp_params[i].addr == addr) {
            dsp_params[i].value = value;
            return;
        }
    }
    if (dsp_param_count < DSP_PARAM_MAX) {
        dsp_params[dsp_param_count].addr = addr;
        dsp_params[dsp_param_count].value = value;
        dsp_param_count++;
    } else {
        ESP_LOGW(TAG, "DSP param table full (%d), dropped addr=0x%06" PRIX32, DSP_PARAM_MAX, addr);
    }
}

// ── Fault injection ──────────────────────────────────────────────

static struct {
    volatile bool     config_switch_delay;   // NAK audio for 4-5s after SetConfig
    volatile bool     empty_hid_response;    // return zero-length HID response
    volatile uint8_t  hid_nak_count;         // reject first N HID writes
    volatile bool     command_timeout;       // withhold HID response for >10s
    uint32_t slow_boot_ms;                   // delay TinyUSB init
    volatile int64_t  config_switch_until;   // timestamp when NAK period ends
    volatile uint8_t  hid_naks_remaining;    // counter for NAK injection
} fault = {0};

// ── HID response queue ──────────────────────────────────────────

static uint8_t  hid_response[64];
static bool     hid_response_pending = false;

// ── Descriptors ─────────────────────────────────────────────────

// --- Device Descriptor ---
tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = 0x2752,
    .idProduct          = 0x0011,       // Real miniDSP PID
    .bcdDevice          = 0x0185,       // Matches real device firmware version
    .iManufacturer      = 1,
    .iProduct           = 11,           // Matches real device (iProduct=11)
    .iSerialNumber      = 0,            // Real miniDSP has no serial string
    .bNumConfigurations = 1             // Real device has 2, but TinyUSB serves 1 (configs are identical)
};

uint8_t const *tud_descriptor_device_cb(void)
{
    return (uint8_t const *)&desc_device;
}

// ── DFU stub driver ──────────────────────────────────────────────
// TinyUSB requires every interface in the config descriptor to be claimed
// by a driver during SET_CONFIGURATION. The real miniDSP has a DFU interface
// (class 0xFE) that we include for descriptor fidelity. This stub claims it.

#include "device/usbd_pvt.h"

static void dfu_stub_init(void) {}
static bool dfu_stub_deinit(void) { return true; }
static void dfu_stub_reset(uint8_t rhport) { (void)rhport; }

static uint16_t dfu_stub_open(uint8_t rhport, tusb_desc_interface_t const *desc_intf, uint16_t max_len)
{
    (void)rhport;
    (void)max_len;
    // Only claim DFU Runtime interfaces (class=0xFE, subclass=0x01, protocol=0x01)
    if (desc_intf->bInterfaceClass != 0xFE ||
        desc_intf->bInterfaceSubClass != 0x01) {
        return 0;
    }
    // Consume interface descriptor (9) + DFU functional descriptor (9) = 18 bytes
    return 18;
}

static bool dfu_stub_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
    (void)rhport; (void)stage; (void)request;
    return false;  // STALL any DFU control requests
}

static bool dfu_stub_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
    (void)rhport; (void)ep_addr; (void)result; (void)xferred_bytes;
    return false;
}

static usbd_class_driver_t const _dfu_stub_driver = {
    .name             = "DFU-STUB",
    .init             = dfu_stub_init,
    .deinit           = dfu_stub_deinit,
    .reset            = dfu_stub_reset,
    .open             = dfu_stub_open,
    .control_xfer_cb  = dfu_stub_control_xfer_cb,
    .xfer_cb          = dfu_stub_xfer_cb,
};

usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count)
{
    *driver_count = 1;
    return &_dfu_stub_driver;
}

// --- Configuration Descriptor ---

// AC entity lengths
#define AC_HEADER_LEN           9
#define CLOCK_SOURCE_LEN        8
#define CLOCK_SELECTOR_LEN      8
#define INPUT_TERMINAL_LEN      17
#define OUTPUT_TERMINAL_LEN     12
#define FEATURE_UNIT_PB_LEN     18      // master + 2ch
#define FEATURE_UNIT_CAP_LEN    26      // master + 4ch

#define AC_TOTAL_LEN    (AC_HEADER_LEN + CLOCK_SOURCE_LEN + CLOCK_SELECTOR_LEN + \
                         INPUT_TERMINAL_LEN + FEATURE_UNIT_PB_LEN + OUTPUT_TERMINAL_LEN + \
                         INPUT_TERMINAL_LEN + FEATURE_UNIT_CAP_LEN + OUTPUT_TERMINAL_LEN)
// = 9 + 8 + 8 + 17 + 18 + 12 + 17 + 26 + 12 = 127

// AS descriptor lengths
#define AS_GENERAL_LEN      16
#define FORMAT_TYPE_LEN     6
#define CS_EP_LEN           8

// Playback per-alt lengths
#define AS_PB_ALT1_LEN_REAL        (9 + AS_GENERAL_LEN + FORMAT_TYPE_LEN + 7 + CS_EP_LEN + 7)  // 53
#define AS_PB_ALT2_LEN_REAL        (9 + AS_GENERAL_LEN + FORMAT_TYPE_LEN + 7 + CS_EP_LEN + 7)  // 53
#define AS_PB_ALT1_LEN_NO_FEEDBACK (9 + AS_GENERAL_LEN + FORMAT_TYPE_LEN + 7 + CS_EP_LEN)       // 46
#define AS_PB_ALT2_LEN_NO_FEEDBACK (9 + AS_GENERAL_LEN + FORMAT_TYPE_LEN + 7 + CS_EP_LEN)       // 46

// Capture alt 1: no feedback EP (only data EP + CS EP)
#define AS_CAP_ALT1_LEN    (9 + AS_GENERAL_LEN + FORMAT_TYPE_LEN + 7 + CS_EP_LEN)       // 46

// Audio function total (for CFG_TUD_AUDIO_FUNC_1_DESC_LEN)
// Real build:
//   IAD(8) + AC_IF(9) + AC(127) + PB_alt0(9) + PB_alt1(53)
//   + PB_alt2(53) + CAP_alt0(9) + CAP_alt1(46) = 314
// No-feedback build:
//   IAD(8) + AC_IF(9) + AC(127) + PB_alt0(9) + PB_alt1(46)
//   + PB_alt2(46) + CAP_alt0(9) = 254
#define AUDIO_FUNC_DESC_LEN_REAL        (8 + 9 + AC_TOTAL_LEN + 9 + AS_PB_ALT1_LEN_REAL + AS_PB_ALT2_LEN_REAL + 9 + AS_CAP_ALT1_LEN)
#define AUDIO_FUNC_DESC_LEN_NO_FEEDBACK (8 + 9 + AC_TOTAL_LEN + 9 + AS_PB_ALT1_LEN_NO_FEEDBACK + AS_PB_ALT2_LEN_NO_FEEDBACK + 9)

// DFU portion (interface + DFU functional descriptor)
#define DFU_DESC_LEN        (9 + 9)     // = 18

// HID portion
#define HID_DESC_LEN        (9 + 9 + 7 + 7)    // interface + HID desc + 2 EPs = 32

// Total: matches real miniDSP 2x4 HD (373 bytes)
#define CONFIG_TOTAL_LEN_REAL        (9 + AUDIO_FUNC_DESC_LEN_REAL + DFU_DESC_LEN + HID_DESC_LEN)
#define CONFIG_TOTAL_LEN_NO_FEEDBACK (9 + AUDIO_FUNC_DESC_LEN_NO_FEEDBACK + DFU_DESC_LEN + HID_DESC_LEN)

#if CONFIG_MINIDSP_SIM_PLAYBACK_CHANNEL_ONLY_FU_TEST_BUILD
#define PLAYBACK_FU_MASTER_CTRL_BYTES U32_TO_U8S_LE(0x00000000)
#else
#define PLAYBACK_FU_MASTER_CTRL_BYTES U32_TO_U8S_LE(0x0000000F)
#endif

// Verify at compile time
_Static_assert(AC_TOTAL_LEN == 127, "AC total length mismatch");
_Static_assert(AUDIO_FUNC_DESC_LEN_REAL == 314, "Audio function desc length mismatch");
_Static_assert(AUDIO_FUNC_DESC_LEN_NO_FEEDBACK == 254, "Audio function desc length mismatch");
_Static_assert(CONFIG_TOTAL_LEN_REAL == 373, "Config total must be 373 bytes (real device)");
_Static_assert(CONFIG_TOTAL_LEN_NO_FEEDBACK == 313, "Config total must be 313 bytes in no-feedback test mode");

#define DESC_CONFIG_PREFIX(config_total_len) \
    9, TUSB_DESC_CONFIGURATION, \
    U16_TO_U8S_LE(config_total_len), \
    ITF_NUM_TOTAL, \
    1, \
    0, \
    0xC0, \
    0, \
    8, TUSB_DESC_INTERFACE_ASSOCIATION, \
    ITF_NUM_AUDIO_CONTROL, \
    3, \
    TUSB_CLASS_AUDIO, \
    0x00, \
    0x20, \
    0, \
    9, TUSB_DESC_INTERFACE, \
    ITF_NUM_AUDIO_CONTROL, 0, 0, \
    TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_CONTROL, AUDIO_INT_PROTOCOL_CODE_V2, 11, \
    AC_HEADER_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_HEADER, \
    U16_TO_U8S_LE(0x0200), 0x08, U16_TO_U8S_LE(AC_TOTAL_LEN), 0x00, \
    CLOCK_SOURCE_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_CLOCK_SOURCE, \
    CLOCK_SOURCE_ID, 0x03, 0x07, 0x00, 0x09, \
    CLOCK_SELECTOR_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_CLOCK_SELECTOR, \
    CLOCK_SELECTOR_ID, 1, CLOCK_SOURCE_ID, 0x03, 0x08, \
    INPUT_TERMINAL_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_INPUT_TERMINAL, \
    INPUT_TERMINAL_PB_ID, U16_TO_U8S_LE(AUDIO_TERM_TYPE_USB_STREAMING), \
    0x00, CLOCK_SELECTOR_ID, 2, U32_TO_U8S_LE(0x00000000), 0x00, U16_TO_U8S_LE(0x0016), 0x0B, \
    FEATURE_UNIT_PB_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_FEATURE_UNIT, \
    FEATURE_UNIT_PB_ID, INPUT_TERMINAL_PB_ID, \
    PLAYBACK_FU_MASTER_CTRL_BYTES, U32_TO_U8S_LE(0x0000000F), U32_TO_U8S_LE(0x0000000F), 0x00, \
    OUTPUT_TERMINAL_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_OUTPUT_TERMINAL, \
    OUTPUT_TERMINAL_PB_ID, U16_TO_U8S_LE(AUDIO_TERM_TYPE_OUT_DESKTOP_SPEAKER), \
    0x00, FEATURE_UNIT_PB_ID, CLOCK_SELECTOR_ID, U16_TO_U8S_LE(0x0000), 0x00, \
    INPUT_TERMINAL_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_INPUT_TERMINAL, \
    INPUT_TERMINAL_CAP_ID, U16_TO_U8S_LE(0x0201), \
    0x00, CLOCK_SELECTOR_ID, 2, U32_TO_U8S_LE(0x00000000), 0x00, U16_TO_U8S_LE(0x0018), 0x00, \
    FEATURE_UNIT_CAP_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_FEATURE_UNIT, \
    FEATURE_UNIT_CAP_ID, INPUT_TERMINAL_CAP_ID, \
    U32_TO_U8S_LE(0x0000000F), U32_TO_U8S_LE(0x0000000F), U32_TO_U8S_LE(0x0000000F), \
    U32_TO_U8S_LE(0x0000000F), U32_TO_U8S_LE(0x0000000F), 0x00, \
    OUTPUT_TERMINAL_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_OUTPUT_TERMINAL, \
    OUTPUT_TERMINAL_CAP_ID, U16_TO_U8S_LE(AUDIO_TERM_TYPE_USB_STREAMING), \
    0x00, FEATURE_UNIT_CAP_ID, CLOCK_SELECTOR_ID, U16_TO_U8S_LE(0x0000), 0x0B, \
    9, TUSB_DESC_INTERFACE, \
    ITF_NUM_AUDIO_STREAMING_PB, 0, 0, \
    TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_STREAMING, AUDIO_INT_PROTOCOL_CODE_V2, 0,

#define DESC_CONFIG_ALT1_WITH_FEEDBACK \
    9, TUSB_DESC_INTERFACE, \
    ITF_NUM_AUDIO_STREAMING_PB, 1, 2, \
    TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_STREAMING, AUDIO_INT_PROTOCOL_CODE_V2, 0, \
    AS_GENERAL_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AS_INTERFACE_AS_GENERAL, \
    INPUT_TERMINAL_PB_ID, 0x00, AUDIO_FORMAT_TYPE_I, U32_TO_U8S_LE(0x00000001), \
    2, U32_TO_U8S_LE(0x00000000), 0x16, \
    FORMAT_TYPE_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AS_INTERFACE_FORMAT_TYPE, \
    AUDIO_FORMAT_TYPE_I, 3, 24, \
    7, TUSB_DESC_ENDPOINT, EPNUM_AUDIO_OUT, \
    0x05, U16_TO_U8S_LE(294), 1, \
    CS_EP_LEN, TUSB_DESC_CS_ENDPOINT, AUDIO_CS_EP_SUBTYPE_GENERAL, \
    0x00, 0x00, 0x02, U16_TO_U8S_LE(0x0008), \
    7, TUSB_DESC_ENDPOINT, EPNUM_AUDIO_FB, \
    0x11, U16_TO_U8S_LE(4), 4,

#define DESC_CONFIG_ALT1_NO_FEEDBACK \
    9, TUSB_DESC_INTERFACE, \
    ITF_NUM_AUDIO_STREAMING_PB, 1, 1, \
    TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_STREAMING, AUDIO_INT_PROTOCOL_CODE_V2, 0, \
    AS_GENERAL_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AS_INTERFACE_AS_GENERAL, \
    INPUT_TERMINAL_PB_ID, 0x00, AUDIO_FORMAT_TYPE_I, U32_TO_U8S_LE(0x00000001), \
    2, U32_TO_U8S_LE(0x00000000), 0x16, \
    FORMAT_TYPE_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AS_INTERFACE_FORMAT_TYPE, \
    AUDIO_FORMAT_TYPE_I, 3, 24, \
    7, TUSB_DESC_ENDPOINT, EPNUM_AUDIO_OUT, \
    0x09, U16_TO_U8S_LE(294), 1, \
    CS_EP_LEN, TUSB_DESC_CS_ENDPOINT, AUDIO_CS_EP_SUBTYPE_GENERAL, \
    0x00, 0x00, 0x02, U16_TO_U8S_LE(0x0008),

#define DESC_CONFIG_ALT2_WITH_FEEDBACK \
    9, TUSB_DESC_INTERFACE, \
    ITF_NUM_AUDIO_STREAMING_PB, 2, 2, \
    TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_STREAMING, AUDIO_INT_PROTOCOL_CODE_V2, 0, \
    AS_GENERAL_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AS_INTERFACE_AS_GENERAL, \
    INPUT_TERMINAL_PB_ID, 0x00, AUDIO_FORMAT_TYPE_I, U32_TO_U8S_LE(0x00000001), \
    2, U32_TO_U8S_LE(0x00000000), 0x16, \
    FORMAT_TYPE_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AS_INTERFACE_FORMAT_TYPE, \
    AUDIO_FORMAT_TYPE_I, 2, 16, \
    7, TUSB_DESC_ENDPOINT, EPNUM_AUDIO_OUT, \
    0x05, U16_TO_U8S_LE(196), 1, \
    CS_EP_LEN, TUSB_DESC_CS_ENDPOINT, AUDIO_CS_EP_SUBTYPE_GENERAL, \
    0x00, 0x00, 0x02, U16_TO_U8S_LE(0x0008), \
    7, TUSB_DESC_ENDPOINT, EPNUM_AUDIO_FB, \
    0x11, U16_TO_U8S_LE(4), 4,

#define DESC_CONFIG_ALT2_NO_FEEDBACK \
    9, TUSB_DESC_INTERFACE, \
    ITF_NUM_AUDIO_STREAMING_PB, 2, 1, \
    TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_STREAMING, AUDIO_INT_PROTOCOL_CODE_V2, 0, \
    AS_GENERAL_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AS_INTERFACE_AS_GENERAL, \
    INPUT_TERMINAL_PB_ID, 0x00, AUDIO_FORMAT_TYPE_I, U32_TO_U8S_LE(0x00000001), \
    2, U32_TO_U8S_LE(0x00000000), 0x16, \
    FORMAT_TYPE_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AS_INTERFACE_FORMAT_TYPE, \
    AUDIO_FORMAT_TYPE_I, 2, 16, \
    7, TUSB_DESC_ENDPOINT, EPNUM_AUDIO_OUT, \
    0x09, U16_TO_U8S_LE(196), 1, \
    CS_EP_LEN, TUSB_DESC_CS_ENDPOINT, AUDIO_CS_EP_SUBTYPE_GENERAL, \
    0x00, 0x00, 0x02, U16_TO_U8S_LE(0x0008),

#define DESC_CONFIG_SUFFIX \
    9, TUSB_DESC_INTERFACE, \
    ITF_NUM_AUDIO_STREAMING_CAP, 0, 0, \
    TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_STREAMING, AUDIO_INT_PROTOCOL_CODE_V2, 11, \
    9, TUSB_DESC_INTERFACE, \
    ITF_NUM_AUDIO_STREAMING_CAP, 1, 1, \
    TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_STREAMING, AUDIO_INT_PROTOCOL_CODE_V2, 11, \
    AS_GENERAL_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AS_INTERFACE_AS_GENERAL, \
    OUTPUT_TERMINAL_CAP_ID, 0x00, AUDIO_FORMAT_TYPE_I, U32_TO_U8S_LE(0x00000001), \
    2, U32_TO_U8S_LE(0x00000000), 0x18, \
    FORMAT_TYPE_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AS_INTERFACE_FORMAT_TYPE, \
    AUDIO_FORMAT_TYPE_I, 3, 24, \
    7, TUSB_DESC_ENDPOINT, EPNUM_AUDIO_CAP, \
    0x05, U16_TO_U8S_LE(294), 1, \
    CS_EP_LEN, TUSB_DESC_CS_ENDPOINT, AUDIO_CS_EP_SUBTYPE_GENERAL, \
    0x00, 0x00, 0x02, U16_TO_U8S_LE(0x0008), \
    9, TUSB_DESC_INTERFACE, \
    ITF_NUM_DFU, 0, 0, \
    0xFE, 0x01, 0x01, 0x0A, \
    9, 0x21, \
    0x07, U16_TO_U8S_LE(0x00FA), U16_TO_U8S_LE(0x0040), U16_TO_U8S_LE(0x0110), \
    9, TUSB_DESC_INTERFACE, \
    ITF_NUM_HID, 0, 2, \
    0x03, 0x00, 0x00, 0x00, \
    9, 0x21, \
    U16_TO_U8S_LE(0x0110), 0x00, 1, 0x22, U16_TO_U8S_LE(28), \
    7, TUSB_DESC_ENDPOINT, EPNUM_HID_IN, \
    0x03, U16_TO_U8S_LE(64), 1, \
    7, TUSB_DESC_ENDPOINT, EPNUM_HID_OUT, \
    0x03, U16_TO_U8S_LE(64), 1

#define DESC_CONFIG_SUFFIX_NO_CAPTURE_STREAM \
    9, TUSB_DESC_INTERFACE, \
    ITF_NUM_AUDIO_STREAMING_CAP, 0, 0, \
    TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_STREAMING, AUDIO_INT_PROTOCOL_CODE_V2, 11, \
    9, TUSB_DESC_INTERFACE, \
    ITF_NUM_DFU, 0, 0, \
    0xFE, 0x01, 0x01, 0x0A, \
    9, 0x21, \
    0x07, U16_TO_U8S_LE(0x00FA), U16_TO_U8S_LE(0x0040), U16_TO_U8S_LE(0x0110), \
    9, TUSB_DESC_INTERFACE, \
    ITF_NUM_HID, 0, 2, \
    0x03, 0x00, 0x00, 0x00, \
    9, 0x21, \
    U16_TO_U8S_LE(0x0110), 0x00, 1, 0x22, U16_TO_U8S_LE(28), \
    7, TUSB_DESC_ENDPOINT, EPNUM_HID_IN, \
    0x03, U16_TO_U8S_LE(64), 1, \
    7, TUSB_DESC_ENDPOINT, EPNUM_HID_OUT, \
    0x03, U16_TO_U8S_LE(64), 1

static uint8_t const desc_configuration_real[] = {
    DESC_CONFIG_PREFIX(CONFIG_TOTAL_LEN_REAL)
    DESC_CONFIG_ALT1_WITH_FEEDBACK
    DESC_CONFIG_ALT2_WITH_FEEDBACK
    DESC_CONFIG_SUFFIX
};

static uint8_t const desc_configuration_no_feedback[] = {
    DESC_CONFIG_PREFIX(CONFIG_TOTAL_LEN_NO_FEEDBACK)
    DESC_CONFIG_ALT1_NO_FEEDBACK
    DESC_CONFIG_ALT2_NO_FEEDBACK
    DESC_CONFIG_SUFFIX_NO_CAPTURE_STREAM
};

_Static_assert(sizeof(desc_configuration_real) == CONFIG_TOTAL_LEN_REAL,
               "Real config descriptor must match CONFIG_TOTAL_LEN_REAL");
_Static_assert(sizeof(desc_configuration_no_feedback) == CONFIG_TOTAL_LEN_NO_FEEDBACK,
               "No-feedback config descriptor must match CONFIG_TOTAL_LEN_NO_FEEDBACK");

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return no_feedback_mode_enabled() ? desc_configuration_no_feedback : desc_configuration_real;
}

// --- HID Report Descriptor (28 bytes) ---
// Captured from real miniDSP 2x4 HD via hidapi get_report_descriptor()
static uint8_t const desc_hid_report[] = {
    0x06, 0x00, 0xFF,       // Usage Page (Vendor Defined 0xFF00)
    0x09, 0x01,             // Usage (Vendor Usage 1)
    0xA1, 0x01,             // Collection (Application)
      0x19, 0x01,           //   Usage Minimum (1)
      0x29, 0x40,           //   Usage Maximum (64)
      0x15, 0x01,           //   Logical Minimum (1)
      0x25, 0x40,           //   Logical Maximum (64)
      0x75, 0x08,           //   Report Size (8 bits)
      0x95, 0x40,           //   Report Count (64)
      0x81, 0x00,           //   Input (Data, Array, Abs)
      0x19, 0x01,           //   Usage Minimum (1)
      0x29, 0x40,           //   Usage Maximum (64)
      0x91, 0x00,           //   Output (Data, Array, Abs)
    0xC0                    // End Collection
};

_Static_assert(sizeof(desc_hid_report) == 28, "HID report descriptor must be 28 bytes (real device)");

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    (void)instance;
    return desc_hid_report;
}

// --- String Descriptors ---
// Real device: iManufacturer=1 ("miniDSP"), iProduct=11 ("miniDSP 2x4HD")
// Indices 2-10 are used by XMOS firmware for various descriptors
static char const *string_desc_arr[] = {
    (const char[]){0x09, 0x04},  // 0: English
    "miniDSP",                    // 1: Manufacturer
    "",                           // 2: (unused)
    "",                           // 3: (unused)
    "",                           // 4: (unused)
    "",                           // 5: (unused)
    "",                           // 6: (unused)
    "",                           // 7: (unused)
    "",                           // 8: (clock selector string)
    "",                           // 9: (clock source string)
    "",                           // 10: (unused)
    SIM_PRODUCT_STRING,           // 11: Product
};

static uint16_t _desc_str[33];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    size_t chr_count;

    if (index == 0) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) return NULL;
        const char *str = string_desc_arr[index];
        chr_count = strlen(str);
        if (chr_count > 31) chr_count = 31;
        for (size_t i = 0; i < chr_count; i++) {
            _desc_str[1 + i] = str[i];
        }
    }

    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}

// ── Audio class callbacks ─────────────────────────────────────────

bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff)
{
    (void)rhport;
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

    TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);

    // Clock Source (ID=41): sample rate
    if (entityID == CLOCK_SOURCE_ID) {
        if (ctrlSel == AUDIO_CS_CTRL_SAM_FREQ) {
            TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_4_t));
            uint32_t sample_rate = (uint32_t)((audio_control_cur_4_t const *)pBuff)->bCur;
            store_current_sample_rate(sample_rate);
            ESP_LOGI(TAG, "SET sample rate: %lu Hz", (unsigned long)sample_rate);
            // The active feedback method is configured on SET_INTERFACE.
            // Avoid touching feedback state from the control-transfer path.
            return true;
        }
    }

    // Playback Feature Unit (ID=10): mute, volume
    if (entityID == FEATURE_UNIT_PB_ID) {
        if (ctrlSel == AUDIO_FU_CTRL_MUTE && playback_fu_channel_supported(channelNum)) {
            TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_1_t));
            pb_mute[channelNum] = ((audio_control_cur_1_t *)pBuff)->bCur;
            ESP_LOGI(TAG, "SET PB mute ch%d: %d", channelNum, pb_mute[channelNum]);
            return true;
        }
        if (ctrlSel == AUDIO_FU_CTRL_VOLUME && playback_fu_channel_supported(channelNum)) {
            TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_2_t));
            pb_volume[channelNum] = ((audio_control_cur_2_t *)pBuff)->bCur;
            ESP_LOGI(TAG, "SET PB volume ch%d: %d (1/256 dB)", channelNum, pb_volume[channelNum]);
            return true;
        }
    }

    // Capture Feature Unit (ID=11): mute, volume
    if (entityID == FEATURE_UNIT_CAP_ID) {
        if (ctrlSel == AUDIO_FU_CTRL_MUTE && channelNum < 5) {
            TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_1_t));
            cap_mute[channelNum] = ((audio_control_cur_1_t *)pBuff)->bCur;
            ESP_LOGI(TAG, "SET CAP mute ch%d: %d", channelNum, cap_mute[channelNum]);
            return true;
        }
        if (ctrlSel == AUDIO_FU_CTRL_VOLUME && channelNum < 5) {
            TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_2_t));
            cap_volume[channelNum] = ((audio_control_cur_2_t *)pBuff)->bCur;
            ESP_LOGI(TAG, "SET CAP volume ch%d: %d (1/256 dB)", channelNum, cap_volume[channelNum]);
            return true;
        }
    }

    // Clock Selector (ID=40)
    if (entityID == CLOCK_SELECTOR_ID) {
        if (ctrlSel == 0x01) {
            ESP_LOGI(TAG, "SET clock selector: input %d", pBuff[0]);
            return true;
        }
    }

    ESP_LOGW(TAG, "Unsupported SET entity=%d ctrl=%d ch=%d", entityID, ctrlSel, channelNum);
    return false;
}

bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    uint8_t channelNum = TU_U16_LOW(p_request->wValue);
    uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
    uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

    // Clock Source (ID=41)
    if (entityID == CLOCK_SOURCE_ID) {
        if (ctrlSel == AUDIO_CS_CTRL_SAM_FREQ) {
            if (p_request->bRequest == AUDIO_CS_REQ_CUR) {
                uint32_t sample_rate = load_current_sample_rate();
                ESP_LOGI(TAG, "GET sample rate CUR -> %lu Hz", (unsigned long)sample_rate);
                audio_control_cur_4_t cur = { .bCur = (int32_t)sample_rate };
                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &cur, sizeof(cur));
            }
            if (p_request->bRequest == AUDIO_CS_REQ_RANGE) {
                ESP_LOGI(TAG, "GET sample rate RANGE");
                audio_control_range_4_n_t(2) range = {
                    .wNumSubRanges = 2,
                    .subrange = {
                        { .bMin = 44100, .bMax = 44100, .bRes = 0 },
                        { .bMin = 48000, .bMax = 48000, .bRes = 0 },
                    }
                };
                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &range, sizeof(range));
            }
        }
        if (ctrlSel == AUDIO_CS_CTRL_CLK_VALID) {
            ESP_LOGI(TAG, "GET clock valid -> %d", clock_valid);
            audio_control_cur_1_t valid = { .bCur = clock_valid };
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &valid, sizeof(valid));
        }
    }

    // Playback Feature Unit (ID=10)
    if (entityID == FEATURE_UNIT_PB_ID) {
        if (ctrlSel == AUDIO_FU_CTRL_MUTE && playback_fu_channel_supported(channelNum)) {
            audio_control_cur_1_t cur = { .bCur = pb_mute[channelNum] };
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &cur, sizeof(cur));
        }
        if (ctrlSel == AUDIO_FU_CTRL_VOLUME) {
            if (p_request->bRequest == AUDIO_CS_REQ_CUR &&
                playback_fu_channel_supported(channelNum)) {
                audio_control_cur_2_t cur = { .bCur = pb_volume[channelNum] };
                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &cur, sizeof(cur));
            }
            if (p_request->bRequest == AUDIO_CS_REQ_RANGE &&
                playback_fu_channel_supported(channelNum)) {
                audio_control_range_2_n_t(1) range = {
                    .wNumSubRanges = 1,
                    .subrange = {{ .bMin = -90 * 256, .bMax = 0, .bRes = 256 }}
                };
                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &range, sizeof(range));
            }
        }
    }

    // Capture Feature Unit (ID=11)
    if (entityID == FEATURE_UNIT_CAP_ID) {
        if (ctrlSel == AUDIO_FU_CTRL_MUTE && channelNum < 5) {
            audio_control_cur_1_t cur = { .bCur = cap_mute[channelNum] };
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &cur, sizeof(cur));
        }
        if (ctrlSel == AUDIO_FU_CTRL_VOLUME) {
            if (p_request->bRequest == AUDIO_CS_REQ_CUR && channelNum < 5) {
                audio_control_cur_2_t cur = { .bCur = cap_volume[channelNum] };
                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &cur, sizeof(cur));
            }
            if (p_request->bRequest == AUDIO_CS_REQ_RANGE) {
                audio_control_range_2_n_t(1) range = {
                    .wNumSubRanges = 1,
                    .subrange = {{ .bMin = -90 * 256, .bMax = 0, .bRes = 256 }}
                };
                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &range, sizeof(range));
            }
        }
    }

    // Clock Selector (ID=40)
    if (entityID == CLOCK_SELECTOR_ID) {
        if (ctrlSel == 0x01) {
            audio_control_cur_1_t cur = { .bCur = 1 };
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &cur, sizeof(cur));
        }
    }

    ESP_LOGW(TAG, "Unsupported GET entity=%d ctrl=%d ch=%d req=%d",
             entityID, ctrlSel, channelNum, p_request->bRequest);
    return false;
}

// Minimal required audio callbacks
bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff)
{
    (void)rhport; (void)p_request; (void)pBuff;
    return false;
}

bool tud_audio_set_req_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *pBuff)
{
    (void)rhport; (void)p_request; (void)pBuff;
    return false;
}

bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    (void)rhport; (void)p_request;
    return false;
}

bool tud_audio_get_req_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    (void)rhport; (void)p_request;
    return false;
}

bool tud_audio_set_itf_close_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    (void)rhport;
    uint8_t itf = TU_U16_LOW(p_request->wIndex);
    if (itf == ITF_NUM_AUDIO_STREAMING_PB) {
        current_pb_alt = 0;
        ESP_LOGI(TAG, "Playback stream stopped (alt 0)");
    }
    return true;
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    (void)rhport;
    uint8_t itf = TU_U16_LOW(p_request->wIndex);
    uint8_t alt = (uint8_t)p_request->wValue;

    if (itf == ITF_NUM_AUDIO_STREAMING_PB) {
        current_pb_alt = alt;
        ESP_LOGI(TAG, "Playback SET_INTERFACE alt=%d (%s, %s)",
                 alt,
                 alt == 0 ? "STOP" : (alt == 1 ? "24-bit" : "16-bit"),
                 alt > 0 ? "START" : "idle");

#if CFG_TUD_AUDIO_ENABLE_EP_OUT && CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
        if (alt > 0) {
            tud_audio_fb_set(sample_rate_to_feedback_q16(load_current_sample_rate()));
        }
#endif
    }
    return true;
}

#if CFG_TUD_AUDIO_ENABLE_EP_OUT && CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP
void tud_audio_feedback_params_cb(uint8_t func_id, uint8_t alt_itf, audio_feedback_params_t *feedback_param)
{
    (void)func_id;
    (void)alt_itf;

    // Keep TinyUSB's feedback endpoint scheduling active, but drive the actual
    // 16.16 value explicitly from the current sample rate below.
    feedback_param->method = AUDIO_FEEDBACK_METHOD_FREQUENCY_FIXED;
    feedback_param->sample_freq = load_current_sample_rate();
    feedback_param->frequency.mclk_freq = 12000000;
}

TU_ATTR_FAST_FUNC void tud_audio_feedback_interval_isr(uint8_t func_id, uint32_t frame_number, uint8_t interval_shift)
{
    (void)func_id;
    (void)frame_number;
    (void)interval_shift;

    tud_audio_fb_set(sample_rate_to_feedback_q16(load_current_sample_rate()));
}
#endif

// Playback: receive audio data from host
// NOTE: This runs in USB ISR context — only use ISR-safe functions (no ESP_LOGx, no locks).
bool tud_audio_rx_done_isr(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting)
{
    (void)rhport; (void)func_id; (void)ep_out; (void)cur_alt_setting;

    // Config switch fault injection: simulate NAK during config switch
    if (fault.config_switch_delay && fault.config_switch_until > 0) {
        if (esp_timer_get_time() < fault.config_switch_until) {
            // Discard audio during config switch
            static uint8_t buf[294];
            while (n_bytes_received > 0) {
                uint16_t to_read = n_bytes_received > sizeof(buf) ? sizeof(buf) : n_bytes_received;
                uint16_t got = tud_audio_read(buf, to_read);
                n_bytes_received -= got;
                if (got == 0) break;
            }
            return true;
        }
        fault.config_switch_until = 0;  // NAK period over
        ESP_DRAM_LOGI(TAG_ISR, "Config switch delay ended, resuming audio");
    }

    total_pb_bytes += n_bytes_received;
    pb_frames_received++;

    if (pb_frames_received % 1000 == 0) {
        ESP_DRAM_LOGI(TAG_ISR, "Playback: %lu frames, %lu bytes (alt %d)",
                      (unsigned long)pb_frames_received,
                      (unsigned long)total_pb_bytes,
                      current_pb_alt);
    }

    // Flush received data
    static uint8_t buf[294];
    while (n_bytes_received > 0) {
        uint16_t to_read = n_bytes_received > sizeof(buf) ? sizeof(buf) : n_bytes_received;
        uint16_t got = tud_audio_read(buf, to_read);
        n_bytes_received -= got;
        if (got == 0) break;
    }
    return true;
}

// ── USB lifecycle callbacks ──────────────────────────────────────

void tud_mount_cb(void) { ESP_LOGI(TAG, "USB: Host connected (mounted)"); }
void tud_umount_cb(void) { ESP_LOGW(TAG, "USB: Host disconnected (unmounted)"); }
void tud_suspend_cb(bool remote_wakeup_en) { ESP_LOGI(TAG, "USB: Suspended (wake=%d)", remote_wakeup_en); }
void tud_resume_cb(void) { ESP_LOGI(TAG, "USB: Resumed"); }

// ── HID frame protocol ───────────────────────────────────────────

// Decode incoming HID frame: extract payload from 64-byte frame
// Frame: [length] [payload...] [checksum] [0xFF padding]
// length is self-inclusive (includes length byte itself)
static int hid_frame_decode(const uint8_t *frame, uint16_t bufsize, uint8_t *payload, uint8_t *payload_len)
{
    if (bufsize < 2) return -1;
    uint8_t len = frame[0];
    if (len < 2 || len > 63 || len >= bufsize) return -1;  // invalid or exceeds buffer

    // Verify checksum
    uint8_t sum = 0;
    for (int i = 0; i < len; i++) {
        sum += frame[i];
    }
    if (frame[len] != (sum & 0xFF)) {
        ESP_LOGW(TAG, "HID checksum mismatch: expected 0x%02X, got 0x%02X", sum & 0xFF, frame[len]);
        return -1;
    }

    // Extract payload (bytes 1..len-1, excluding length byte)
    *payload_len = len - 1;
    memcpy(payload, &frame[1], *payload_len);
    return 0;
}

// Encode outgoing HID frame: build 64-byte response
static void hid_frame_encode(const uint8_t *payload, uint8_t payload_len, uint8_t *frame)
{
    if (payload_len > 62) payload_len = 62;
    memset(frame, 0xFF, 64);             // 0xFF padding
    frame[0] = payload_len + 1;          // length (self-inclusive)
    memcpy(&frame[1], payload, payload_len);

    // Checksum: sum of all bytes before checksum position
    uint8_t sum = 0;
    uint8_t total = payload_len + 1;     // length byte + payload
    for (int i = 0; i < total; i++) {
        sum += frame[i];
    }
    frame[total] = sum & 0xFF;
}

// ── HID command handlers ─────────────────────────────────────────

// cmd 0x31: ReadHardwareId
static void cmd_read_hardware_id(uint8_t *resp, uint8_t *resp_len)
{
    resp[0] = 0x31;
    resp[1] = 0x01;     // fw_major = 1
    resp[2] = 0x55;     // fw_minor = 85 (0x55) → firmware 1.85
    resp[3] = 0x0A;     // hw_id = 10 (2x4 HD)
    *resp_len = 4;
    ESP_LOGI(TAG, "HID: ReadHardwareId → hw_id=10, fw=1.85");
}

// cmd 0x05: ReadFlash
static void cmd_read_flash(const uint8_t *payload, uint8_t len, uint8_t *resp, uint8_t *resp_len)
{
    if (len < 3) { resp[0] = 0x00; *resp_len = 1; return; }

    uint8_t page = payload[1];
    uint8_t addr = payload[2];
    uint8_t count = (len >= 4) ? payload[3] : 1;

    ESP_LOGI(TAG, "HID: ReadFlash page=0x%02X addr=0x%02X count=%d", page, addr, count);

    resp[0] = 0x05;
    *resp_len = 1;

    if (page == 0xFF) {
        // Bounds check: prevent read past end of page
        if ((uint16_t)addr + count > 256) count = 256 - addr;
        // Build a flat view of the EEPROM page
        static uint8_t eeprom_page[256];
        memset(eeprom_page, 0, sizeof(eeprom_page));
        eeprom_page[0xA1] = eeprom.dsp_id;
        // Mod tokens at 0xC8 (4 presets × 4 bytes, big-endian)
        for (int i = 0; i < 4; i++) {
            eeprom_page[0xC8 + i * 4 + 0] = (eeprom.mod_tokens[i] >> 24) & 0xFF;
            eeprom_page[0xC8 + i * 4 + 1] = (eeprom.mod_tokens[i] >> 16) & 0xFF;
            eeprom_page[0xC8 + i * 4 + 2] = (eeprom.mod_tokens[i] >> 8) & 0xFF;
            eeprom_page[0xC8 + i * 4 + 3] = eeprom.mod_tokens[i] & 0xFF;
        }
        eeprom_page[0xD8] = eeprom.current_preset;
        eeprom_page[0xD9] = eeprom.current_source;
        eeprom_page[0xDA] = eeprom.master_volume;
        eeprom_page[0xDB] = eeprom.master_mute;
        eeprom_page[0xE0] = eeprom.dirac_bypass;
        eeprom_page[0xE8] = eeprom.display_idle;
        eeprom_page[0xE9] = eeprom.display_brightness;
        eeprom_page[0xED] = eeprom.dre_state;
        eeprom_page[0xFE] = (eeprom.serial_number >> 8) & 0xFF;    // BE
        eeprom_page[0xFF] = eeprom.serial_number & 0xFF;

        // Copy requested range
        if (count > 60) count = 60;  // max payload
        memcpy(&resp[1], &eeprom_page[addr], count);
        *resp_len = 1 + count;
    } else if (page == 0xFE) {
        // Per-source volume offsets
        if (count > 3) count = 3;
        memcpy(&resp[1], source_vol_offsets, count);
        *resp_len = 1 + count;
    }
}

// cmd 0x04: WriteFlash
static void cmd_write_flash(const uint8_t *payload, uint8_t len, uint8_t *resp, uint8_t *resp_len)
{
    if (len < 4) { resp[0] = 0x00; *resp_len = 1; return; }

    uint8_t page = payload[1];
    uint8_t addr = payload[2];
    uint8_t data_len = len - 3;

    ESP_LOGI(TAG, "HID: WriteFlash page=0x%02X addr=0x%02X len=%d", page, addr, data_len);

    if (page == 0xFF) {
        // EEPROM page — write individual bytes to state
        // Bounds check: prevent wrap past end of page
        if ((uint16_t)addr + data_len > 256) data_len = 256 - addr;
        for (int i = 0; i < data_len; i++) {
            uint8_t a = addr + i;
            uint8_t v = payload[3 + i];
            switch (a) {
            case 0xA1: eeprom.dsp_id = v; break;
            case 0xD8: eeprom.current_preset = v; break;
            case 0xD9: eeprom.current_source = v; break;
            case 0xDA: eeprom.master_volume = v; break;
            case 0xDB: eeprom.master_mute = v; break;
            case 0xE0: eeprom.dirac_bypass = v; break;
            case 0xE8: eeprom.display_idle = v; break;
            case 0xE9: eeprom.display_brightness = v; break;
            case 0xED: eeprom.dre_state = v; break;
            case 0xFE: eeprom.serial_number = (eeprom.serial_number & 0x00FF) | ((uint16_t)v << 8); break;
            case 0xFF: eeprom.serial_number = (eeprom.serial_number & 0xFF00) | v; break;
            default:
                // Mod tokens at 0xC8..0xD7 (4 presets × 4 bytes, big-endian)
                if (a >= 0xC8 && a < 0xD8) {
                    int preset = (a - 0xC8) / 4;
                    int byte_pos = (a - 0xC8) % 4;
                    int shift = (3 - byte_pos) * 8;
                    eeprom.mod_tokens[preset] &= ~((uint32_t)0xFF << shift);
                    eeprom.mod_tokens[preset] |= ((uint32_t)v << shift);
                }
                break;
            }
        }
    } else if (page == 0xFE && addr == 0x00) {
        // Per-source volume offsets
        for (int i = 0; i < data_len && i < 3; i++) {
            source_vol_offsets[i] = (int8_t)payload[3 + i];
        }
    }

    resp[0] = 0x01;  // ACK
    *resp_len = 1;
}

// cmd 0x14: ReadFloats
// Wire format (3-byte addr): [0x14, addr_hi, addr_mid, addr_lo, count]
static void cmd_read_floats(const uint8_t *payload, uint8_t len, uint8_t *resp, uint8_t *resp_len)
{
    if (len < 5) { resp[0] = 0x00; *resp_len = 1; return; }

    uint32_t addr = ((uint32_t)payload[1] << 16) | ((uint32_t)payload[2] << 8) | payload[3];
    uint8_t count = payload[4];
    if (count > 14) count = 14;  // protocol max

    ESP_LOGI(TAG, "HID: ReadFloats addr=0x%06" PRIX32 " count=%d", addr, count);

    resp[0] = 0x14;
    for (int i = 0; i < count; i++) {
        float val = dsp_param_get(addr + i);
        uint32_t bits;
        memcpy(&bits, &val, 4);
        resp[1 + i * 4 + 0] = (bits >> 0) & 0xFF;   // LE
        resp[1 + i * 4 + 1] = (bits >> 8) & 0xFF;
        resp[1 + i * 4 + 2] = (bits >> 16) & 0xFF;
        resp[1 + i * 4 + 3] = (bits >> 24) & 0xFF;
    }
    *resp_len = 1 + count * 4;
}

// cmd 0x13: WriteDSP
// Wire format (3-byte addr): [0x13, mode, addr_hi, addr_mid, addr_lo, val0..val3]
static void cmd_write_dsp(const uint8_t *payload, uint8_t len, uint8_t *resp, uint8_t *resp_len)
{
    if (len < 6) { resp[0] = 0x00; *resp_len = 1; return; }

    uint8_t mode = payload[1];
    uint32_t addr = ((uint32_t)payload[2] << 16) | ((uint32_t)payload[3] << 8) | payload[4];
    int val_offset = 5;

    if (val_offset + 4 <= len) {
        float val;
        uint32_t bits = (uint32_t)payload[val_offset] | ((uint32_t)payload[val_offset+1] << 8) |
                        ((uint32_t)payload[val_offset+2] << 16) | ((uint32_t)payload[val_offset+3] << 24);
        memcpy(&val, &bits, 4);
        dsp_param_set(addr, val);
        ESP_LOGI(TAG, "HID: WriteDSP %s addr=0x%06" PRIX32 " val=%.6f",
                 mode == 0x80 ? "SAVE" : "RAM", addr, val);
    }

    resp[0] = 0x01;  // ACK
    *resp_len = 1;
}

// cmd 0x30: WriteBiquad
// Wire format (3-byte addr): [0x30, 0x80, addr_hi, addr_mid, addr_lo, 0x00, 0x00, 5×float32_LE]
static void cmd_write_biquad(const uint8_t *payload, uint8_t len, uint8_t *resp, uint8_t *resp_len)
{
    if (len < 27) { resp[0] = 0x00; *resp_len = 1; return; }

    uint32_t addr = ((uint32_t)payload[2] << 16) | ((uint32_t)payload[3] << 8) | payload[4];
    ESP_LOGI(TAG, "HID: WriteBiquad addr=0x%06" PRIX32, addr);

    // Store 5 coefficients (b0, b1, b2, a1, a2)
    for (int i = 0; i < 5; i++) {
        int off = 7 + i * 4;
        uint32_t bits = (uint32_t)payload[off] | ((uint32_t)payload[off+1] << 8) |
                        ((uint32_t)payload[off+2] << 16) | ((uint32_t)payload[off+3] << 24);
        float val;
        memcpy(&val, &bits, 4);
        dsp_param_set(addr + i, val);
    }

    resp[0] = 0x01;  // ACK
    *resp_len = 1;
}

// cmd 0x42: SetVolume
static void cmd_set_volume(const uint8_t *payload, uint8_t len, uint8_t *resp, uint8_t *resp_len)
{
    if (len < 2) { resp[0] = 0x00; *resp_len = 1; return; }
    eeprom.master_volume = payload[1];
    float db = -0.5f * eeprom.master_volume;
    ESP_LOGI(TAG, "HID: SetVolume raw=%d (%.1f dB)", eeprom.master_volume, db);
    resp[0] = 0x01;
    *resp_len = 1;
}

// cmd 0x17: SetMute
static void cmd_set_mute(const uint8_t *payload, uint8_t len, uint8_t *resp, uint8_t *resp_len)
{
    if (len < 2) { resp[0] = 0x00; *resp_len = 1; return; }
    eeprom.master_mute = payload[1];
    ESP_LOGI(TAG, "HID: SetMute %s", eeprom.master_mute ? "MUTED" : "UNMUTED");
    resp[0] = 0x01;
    *resp_len = 1;
}

// cmd 0x25: SetConfig (preset switch)
static void cmd_set_config(const uint8_t *payload, uint8_t len, uint8_t *resp, uint8_t *resp_len)
{
    if (len < 2) { resp[0] = 0x00; *resp_len = 1; return; }
    eeprom.current_preset = payload[1] & 0x03;
    ESP_LOGI(TAG, "HID: SetConfig preset=%d", eeprom.current_preset);

    // Trigger config switch delay fault injection
    if (fault.config_switch_delay) {
        fault.config_switch_until = esp_timer_get_time() + 4500000;  // 4.5 seconds
        ESP_LOGW(TAG, "FAULT: Config switch delay active for 4.5s");
    }

    resp[0] = 0xAB;  // Preset change complete
    *resp_len = 1;
}

// cmd 0x34: SetSource
static void cmd_set_source(const uint8_t *payload, uint8_t len, uint8_t *resp, uint8_t *resp_len)
{
    if (len < 2) { resp[0] = 0x00; *resp_len = 1; return; }
    eeprom.current_source = payload[1];
    ESP_LOGI(TAG, "HID: SetSource %d (%s)",
             eeprom.current_source,
             eeprom.current_source == 0 ? "Analog" :
             eeprom.current_source == 1 ? "TOSLINK" : "USB");
    resp[0] = 0x01;
    *resp_len = 1;
}

// cmd 0x19: BypassFilter
// Wire format (3-byte addr): [0x19, bypass, addr_hi, addr_mid, addr_lo]
static void cmd_bypass_filter(const uint8_t *payload, uint8_t len, uint8_t *resp, uint8_t *resp_len)
{
    if (len < 5) { resp[0] = 0x00; *resp_len = 1; return; }
    uint8_t bypass = payload[1];
    uint32_t addr = ((uint32_t)payload[2] << 16) | ((uint32_t)payload[3] << 8) | payload[4];
    ESP_LOGI(TAG, "HID: BypassFilter addr=0x%06" PRIX32 " bypass=%s", addr, bypass ? "ON" : "OFF");
    resp[0] = 0x01;
    *resp_len = 1;
}

// cmd 0x3F: DiracBypass
static void cmd_dirac_bypass(const uint8_t *payload, uint8_t len, uint8_t *resp, uint8_t *resp_len)
{
    if (len < 2) { resp[0] = 0x00; *resp_len = 1; return; }
    eeprom.dirac_bypass = payload[1];
    ESP_LOGI(TAG, "HID: DiracBypass %s", eeprom.dirac_bypass ? "BYPASSED" : "ENABLED");
    resp[0] = 0x01;
    *resp_len = 1;
}

// cmd 0x03: Reset
static void cmd_reset(uint8_t *resp, uint8_t *resp_len)
{
    ESP_LOGI(TAG, "HID: Reset");
    resp[0] = 0xAA;  // Reset complete
    *resp_len = 1;
}

// cmd 0x39: FirLoadStart
static void cmd_fir_load_start(const uint8_t *payload, uint8_t len, uint8_t *resp, uint8_t *resp_len)
{
    (void)len;
    uint8_t index = (len >= 2) ? payload[1] : 0;
    ESP_LOGI(TAG, "HID: FirLoadStart index=%d", index);
    resp[0] = 0x39;
    // Return max coefficient count (2048 for 2x4 HD outputs)
    resp[1] = (2048 >> 8) & 0xFF;
    resp[2] = 2048 & 0xFF;
    *resp_len = 3;
}

// cmd 0x3A: FirLoadData
static void cmd_fir_load_data(const uint8_t *payload, uint8_t len, uint8_t *resp, uint8_t *resp_len)
{
    (void)payload; (void)len;
    ESP_LOGI(TAG, "HID: FirLoadData (%d bytes)", len);
    resp[0] = 0x01;  // ACK
    *resp_len = 1;
}

// cmd 0x3B: ReloadDspParam
static void cmd_reload_dsp_param(uint8_t *resp, uint8_t *resp_len)
{
    ESP_LOGI(TAG, "HID: ReloadDspParam");
    resp[0] = 0x01;  // ACK
    *resp_len = 1;
}

// cmd 0x27: CopyPreset
static void cmd_copy_preset(uint8_t *resp, uint8_t *resp_len)
{
    ESP_LOGI(TAG, "HID: CopyPreset");
    resp[0] = 0x01;  // ACK
    *resp_len = 1;
}

// cmd 0x1A: SetDisplayBrightness
static void cmd_set_display_brightness(const uint8_t *payload, uint8_t len, uint8_t *resp, uint8_t *resp_len)
{
    if (len >= 2) eeprom.display_brightness = payload[1] & 0x7F;
    ESP_LOGI(TAG, "HID: SetDisplayBrightness %d", eeprom.display_brightness);
    resp[0] = 0x01;
    *resp_len = 1;
}

// cmd 0x1B: SetDisplayIdleTime
static void cmd_set_display_idle(const uint8_t *payload, uint8_t len, uint8_t *resp, uint8_t *resp_len)
{
    if (len >= 2) eeprom.display_idle = payload[1];
    ESP_LOGI(TAG, "HID: SetDisplayIdleTime %d sec", eeprom.display_idle);
    resp[0] = 0x01;
    *resp_len = 1;
}

// cmd 0x1E: SetDRE
static void cmd_set_dre(const uint8_t *payload, uint8_t len, uint8_t *resp, uint8_t *resp_len)
{
    if (len >= 3) eeprom.dre_state = payload[2];
    ESP_LOGI(TAG, "HID: SetDRE %s", eeprom.dre_state ? "ON" : "OFF");
    resp[0] = 0x01;
    *resp_len = 1;
}

// Process a decoded HID command
static void hid_process_command(const uint8_t *payload, uint8_t payload_len)
{
    if (payload_len == 0) return;

    uint8_t resp[62];
    uint8_t resp_len = 0;
    uint8_t cmd = payload[0];

    switch (cmd) {
    case 0x31: cmd_read_hardware_id(resp, &resp_len); break;
    case 0x05: cmd_read_flash(payload, payload_len, resp, &resp_len); break;
    case 0x04: cmd_write_flash(payload, payload_len, resp, &resp_len); break;
    case 0x14: cmd_read_floats(payload, payload_len, resp, &resp_len); break;
    case 0x13: cmd_write_dsp(payload, payload_len, resp, &resp_len); break;
    case 0x30: cmd_write_biquad(payload, payload_len, resp, &resp_len); break;
    case 0x42: cmd_set_volume(payload, payload_len, resp, &resp_len); break;
    case 0x17: cmd_set_mute(payload, payload_len, resp, &resp_len); break;
    case 0x25: cmd_set_config(payload, payload_len, resp, &resp_len); break;
    case 0x34: cmd_set_source(payload, payload_len, resp, &resp_len); break;
    case 0x19: cmd_bypass_filter(payload, payload_len, resp, &resp_len); break;
    case 0x3F: cmd_dirac_bypass(payload, payload_len, resp, &resp_len); break;
    case 0x03: cmd_reset(resp, &resp_len); break;
    case 0x39: cmd_fir_load_start(payload, payload_len, resp, &resp_len); break;
    case 0x3A: cmd_fir_load_data(payload, payload_len, resp, &resp_len); break;
    case 0x3B: cmd_reload_dsp_param(resp, &resp_len); break;
    case 0x27: cmd_copy_preset(resp, &resp_len); break;
    case 0x1A: cmd_set_display_brightness(payload, payload_len, resp, &resp_len); break;
    case 0x1B: cmd_set_display_idle(payload, payload_len, resp, &resp_len); break;
    case 0x1E: cmd_set_dre(payload, payload_len, resp, &resp_len); break;
    default:
        ESP_LOGW(TAG, "HID: Unknown command 0x%02X (len=%d)", cmd, payload_len);
        resp[0] = 0x00;  // Error
        resp_len = 1;
        break;
    }

    // Fault injection: empty response
    if (fault.empty_hid_response) {
        ESP_LOGW(TAG, "FAULT: Sending empty HID response");
        memset(hid_response, 0xFF, 64);
        hid_response[0] = 0x01;    // length=1 (just length byte, no payload)
        hid_response[1] = 0x01;    // checksum
        hid_response_pending = true;
        return;
    }

    // Fault injection: command timeout
    if (fault.command_timeout) {
        ESP_LOGW(TAG, "FAULT: Withholding HID response (timeout simulation)");
        return;  // Don't send response
    }

    // Normal response
    hid_frame_encode(resp, resp_len, hid_response);
    hid_response_pending = true;
}

// ── HID class callbacks ──────────────────────────────────────────

// Called when host sends an OUT report (command)
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type,
                            uint8_t const *buffer, uint16_t bufsize)
{
    (void)instance; (void)report_id; (void)report_type;

    if (bufsize < 1) return;

    // Fault injection: NAK first N writes
    if (fault.hid_naks_remaining > 0) {
        fault.hid_naks_remaining--;
        ESP_LOGW(TAG, "FAULT: NAK HID write (%d remaining)", fault.hid_naks_remaining);
        return;
    }

    // Decode frame
    uint8_t payload[62];
    uint8_t payload_len = 0;
    if (hid_frame_decode(buffer, bufsize, payload, &payload_len) != 0) {
        ESP_LOGW(TAG, "HID: Frame decode failed");
        return;
    }

    hid_process_command(payload, payload_len);

    // Send response immediately if pending
    if (hid_response_pending) {
        tud_hid_report(0, hid_response, 64);
        hid_response_pending = false;
    }
}

// Called when host requests a report via GET_REPORT control transfer
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type,
                                uint8_t *buffer, uint16_t reqlen)
{
    (void)instance; (void)report_id; (void)report_type;

    if (hid_response_pending && reqlen >= 64) {
        memcpy(buffer, hid_response, 64);
        hid_response_pending = false;
        return 64;
    }
    return 0;
}

// ── Fault injection serial interface ─────────────────────────────

// Process serial commands for runtime fault control
// Called from main loop to check for UART input
static void check_serial_commands(void)
{
    int c = getchar();
    if (c == EOF) return;

    switch (c) {
    case '1':
        fault.config_switch_delay = !fault.config_switch_delay;
        ESP_LOGI(TAG, "FAULT: Config switch delay %s",
                 fault.config_switch_delay ? "ENABLED" : "DISABLED");
        break;
    case '2':
        fault.empty_hid_response = !fault.empty_hid_response;
        ESP_LOGI(TAG, "FAULT: Empty HID response %s",
                 fault.empty_hid_response ? "ENABLED" : "DISABLED");
        break;
    case '3':
        fault.hid_nak_count = 3;
        fault.hid_naks_remaining = 3;
        ESP_LOGI(TAG, "FAULT: HID NAK next 3 writes");
        break;
    case '4':
        fault.command_timeout = !fault.command_timeout;
        ESP_LOGI(TAG, "FAULT: Command timeout %s",
                 fault.command_timeout ? "ENABLED" : "DISABLED");
        break;
    case 's':
        ESP_LOGI(TAG, "═══ Simulator State ═══");
        ESP_LOGI(TAG, "  Preset: %d  Source: %d  Volume: %d (%.1f dB)  Mute: %d",
                 eeprom.current_preset, eeprom.current_source,
                 eeprom.master_volume, -0.5f * eeprom.master_volume,
                 eeprom.master_mute);
        ESP_LOGI(TAG, "  Playback: alt=%d frames=%lu bytes=%lu",
                 current_pb_alt, (unsigned long)pb_frames_received,
                 (unsigned long)total_pb_bytes);
        ESP_LOGI(TAG, "  DSP params stored: %d", dsp_param_count);
        ESP_LOGI(TAG, "  Sample rate: %lu Hz  Clock valid: %d",
                 (unsigned long)load_current_sample_rate(), clock_valid);
        ESP_LOGI(TAG, "  Descriptor mode: playback alts %s feedback endpoints",
                 no_feedback_mode_enabled() ? "WITHOUT" : "WITH");
        ESP_LOGI(TAG, "  Playback FU master channel: %s",
                 playback_channel_only_fu_mode_enabled() ? "UNSUPPORTED (channels 1/2 only)" : "SUPPORTED");
        ESP_LOGI(TAG, "  Faults: csw_delay=%d empty_hid=%d nak=%d timeout=%d",
                 fault.config_switch_delay, fault.empty_hid_response,
                 fault.hid_naks_remaining, fault.command_timeout);
        break;
    case 'h':
        ESP_LOGI(TAG, "Fault injection keys:");
        ESP_LOGI(TAG, "  1: Toggle config switch delay (4.5s audio NAK after SetConfig)");
        ESP_LOGI(TAG, "  2: Toggle empty HID response");
        ESP_LOGI(TAG, "  3: NAK next 3 HID writes");
        ESP_LOGI(TAG, "  4: Toggle command timeout");
        ESP_LOGI(TAG, "  s: Show simulator state");
        ESP_LOGI(TAG, "  h: Show this help");
        break;
    }
}

// ── Main ──────────────────────────────────────────────────────────

static void tusb_device_task(void *arg)
{
    (void)arg;
    while (1) {
        tud_task();
        taskYIELD();
    }
}

static void monitor_task(void *arg)
{
    (void)arg;
    while (1) {
        check_serial_commands();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "═══════════════════════════════════════════");
    ESP_LOGI(TAG, " miniDSP 2x4 HD Simulator");
    ESP_LOGI(TAG, " VID=0x%04X PID=0x%04X bcdDevice=0x%04X",
             desc_device.idVendor, desc_device.idProduct, desc_device.bcdDevice);
    ESP_LOGI(TAG, "═══════════════════════════════════════════");
    ESP_LOGI(TAG, "Audio: Playback IF1 (24-bit alt1 + 16-bit alt2%s)",
             no_feedback_mode_enabled() ? ", no feedback test mode" : "");
    ESP_LOGI(TAG, "HID:   IF4 (64-byte vendor reports, full command protocol)");
    ESP_LOGI(TAG, "Clock: Source ID=%d, Selector ID=%d (44.1/48 kHz)",
             CLOCK_SOURCE_ID, CLOCK_SELECTOR_ID);
    ESP_LOGI(TAG, "Playback: IT%d → FU%d → OT%d (Speaker)",
             INPUT_TERMINAL_PB_ID, FEATURE_UNIT_PB_ID, OUTPUT_TERMINAL_PB_ID);
    ESP_LOGI(TAG, "Playback FU controls: %s",
             playback_channel_only_fu_mode_enabled()
                 ? "channels 1/2 only (master channel 0 unsupported)"
                 : "master + channels 1/2");
    ESP_LOGI(TAG, "Capture:  IT%d → FU%d → OT%d (USB Streaming)",
             INPUT_TERMINAL_CAP_ID, FEATURE_UNIT_CAP_ID, OUTPUT_TERMINAL_CAP_ID);
    if (no_feedback_mode_enabled()) {
        ESP_LOGI(TAG, "Endpoints: 0x01 OUT (pb, no feedback on playback alts), 0x83/0x02 (HID)");
    } else {
        ESP_LOGI(TAG, "Endpoints: 0x01 OUT (pb), 0x81 IN (fb), 0x82 IN (cap), 0x83/0x02 (HID)");
    }
    ESP_LOGI(TAG, "EEPROM: preset=%d source=%d vol=%d mute=%d serial=%d",
             eeprom.current_preset, eeprom.current_source,
             eeprom.master_volume, eeprom.master_mute, eeprom.serial_number);
    ESP_LOGI(TAG, "Press 'h' for fault injection help");

    // Fault injection: slow boot
    if (fault.slow_boot_ms > 0) {
        ESP_LOGW(TAG, "FAULT: Slow boot delay %lu ms", (unsigned long)fault.slow_boot_ms);
        vTaskDelay(pdMS_TO_TICKS(fault.slow_boot_ms));
    }

    // Initialize USB PHY
    ESP_ERROR_CHECK(usb_phy_init());

    // Initialize TinyUSB
    tusb_rhport_init_t dev_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_FULL,
    };
    if (!tusb_init(0, &dev_init)) {
        ESP_LOGE(TAG, "TinyUSB init failed, restarting in 3s...");
        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_restart();
    }

    ESP_LOGI(TAG, "TinyUSB initialized, waiting for host...");

    // TinyUSB device task (priority 5, core 0)
    BaseType_t ret;
    ret = xTaskCreatePinnedToCore(tusb_device_task, "tusb_dev", 8192, NULL, 5, NULL, 0);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create tusb_dev task");
    }

    // Monitor task for serial commands (priority 2, core 1)
    ret = xTaskCreatePinnedToCore(monitor_task, "monitor", 2048, NULL, 2, NULL, 1);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create monitor task");
    }
}
