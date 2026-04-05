/*
 * miniDSP 2x4 HD Simulator — Full Composite Device
 *
 * Faithful replica of the real miniDSP 2x4 HD on USB:
 * - UAC2 audio: playback (IF1, alt 1 = 24-bit, alt 2 = 16-bit)
 * - Feedback endpoint: 10.14 format at Full Speed (via TinyUSB auto-conversion)
 * - HID interface (IF2): 64-byte vendor reports, full command protocol
 * - EEPROM state: preset, source, volume, mute, serial, mod tokens
 * - DSP parameter state: routing, PEQ, gain, delay, compressor (flat defaults)
 * - Fault injection modes for robustness testing
 *
 * VID=0x2752 PID=0x0011 bcdDevice=0x06F2
 * Config descriptor: 300 bytes (playback-only — capture AS removed for TinyUSB N_AS_INT fix)
 *
 * SPDX-License-Identifier: MIT
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_private/usb_phy.h"
#include "tusb.h"

static const char *TAG = "minidsp-sim";
static const DRAM_ATTR char TAG_ISR[] = "minidsp-sim";  // ISR-safe copy in DRAM

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
    ITF_NUM_HID = 2,
    ITF_NUM_TOTAL = 3
};

#define EPNUM_AUDIO_OUT     0x01    // Playback data
#define EPNUM_AUDIO_FB      0x81    // Feedback
#define EPNUM_HID_IN        0x82    // HID responses
#define EPNUM_HID_OUT       0x02    // HID commands

// ── Audio control state ───────────────────────────────────────────

static uint32_t current_sample_rate = 48000;
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
    uint16_t addr;
    float    value;
} dsp_params[DSP_PARAM_MAX];
static int dsp_param_count = 0;

static float dsp_param_get(uint16_t addr)
{
    for (int i = 0; i < dsp_param_count; i++) {
        if (dsp_params[i].addr == addr) return dsp_params[i].value;
    }
    // Default: 0.0 for most, except PEQ b0 coefficients = 1.0
    return 0.0f;
}

static void dsp_param_set(uint16_t addr, float value)
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
        ESP_LOGW(TAG, "DSP param table full (%d), dropped addr=0x%04X", DSP_PARAM_MAX, addr);
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
    .bcdDevice          = 0x06F2,
    .iManufacturer      = 1,
    .iProduct           = 3,
    .iSerialNumber      = 0,        // Real miniDSP has no serial string
    .bNumConfigurations = 1
};

uint8_t const *tud_descriptor_device_cb(void)
{
    return (uint8_t const *)&desc_device;
}

// --- Configuration Descriptor (300 bytes, playback-only) ---

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

// Per-alt-setting lengths
#define AS_ALT_24BIT_LEN    (9 + AS_GENERAL_LEN + FORMAT_TYPE_LEN + 7 + CS_EP_LEN + 7)  // 53
#define AS_ALT_16BIT_LEN    (9 + AS_GENERAL_LEN + FORMAT_TYPE_LEN + 7 + CS_EP_LEN + 7)  // 53

// Audio function total (for CFG_TUD_AUDIO_FUNC_1_DESC_LEN)
// IAD(8) + AC_IF(9) + AC_entities(127) + AS_PB_alt0(9) + AS_PB_alt1(53) + AS_PB_alt2(53) = 259
#define AUDIO_FUNC_DESC_LEN (8 + 9 + AC_TOTAL_LEN + 9 + AS_ALT_24BIT_LEN + AS_ALT_16BIT_LEN)  // = 259

// HID portion
#define HID_DESC_LEN        (9 + 9 + 7 + 7)    // interface + HID desc + 2 EPs = 32

// Total (no DFU — no test value, and TinyUSB asserts on unclaimed interfaces)
#define CONFIG_TOTAL_LEN    (9 + AUDIO_FUNC_DESC_LEN + HID_DESC_LEN)  // = 300

// Verify at compile time
_Static_assert(AC_TOTAL_LEN == 127, "AC total length mismatch");
_Static_assert(AUDIO_FUNC_DESC_LEN == 259, "Audio function desc length mismatch");
_Static_assert(CONFIG_TOTAL_LEN == 300, "Config total length mismatch");

static uint8_t const desc_configuration[] = {
    // ═══════════════════════════════════════════════════════════════
    //  Configuration Descriptor (9 bytes) [offset 0]
    // ═══════════════════════════════════════════════════════════════
    9, TUSB_DESC_CONFIGURATION,
    U16_TO_U8S_LE(CONFIG_TOTAL_LEN),    // wTotalLength = 300
    ITF_NUM_TOTAL,                       // bNumInterfaces = 3
    1,                                   // bConfigurationValue
    0,                                   // iConfiguration
    0xC0,                                // bmAttributes = self-powered
    0,                                   // bMaxPower = 0

    // ═══════════════════════════════════════════════════════════════
    //  IAD: Audio function (IF0-IF1) [offset 9]
    // ═══════════════════════════════════════════════════════════════
    8, TUSB_DESC_INTERFACE_ASSOCIATION,
    ITF_NUM_AUDIO_CONTROL,               // bFirstInterface = 0
    2,                                   // bInterfaceCount = 2 (AC + 1×AS)
    TUSB_CLASS_AUDIO,                    // bFunctionClass
    0x00,                                // bFunctionSubClass
    0x20,                                // bFunctionProtocol = UAC2
    0,                                   // iFunction

    // ═══════════════════════════════════════════════════════════════
    //  AC Interface 0, Alt 0 [offset 17]
    // ═══════════════════════════════════════════════════════════════
    9, TUSB_DESC_INTERFACE,
    ITF_NUM_AUDIO_CONTROL,               // bInterfaceNumber = 0
    0,                                   // bAlternateSetting = 0
    0,                                   // bNumEndpoints = 0
    TUSB_CLASS_AUDIO,
    AUDIO_SUBCLASS_CONTROL,
    AUDIO_INT_PROTOCOL_CODE_V2,          // 0x20
    0,                                   // iInterface

    // ─── AC Header (9 bytes) [offset 26] ───
    AC_HEADER_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_HEADER,
    U16_TO_U8S_LE(0x0200),              // bcdADC = 2.00
    0x08,                                // bCategory = IO_BOX
    U16_TO_U8S_LE(AC_TOTAL_LEN),        // wTotalLength = 127
    0x00,                                // bmControls

    // ─── Clock Source ID=41 (8 bytes) [offset 35] ───
    CLOCK_SOURCE_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_CLOCK_SOURCE,
    CLOCK_SOURCE_ID,                     // bClockID = 41
    0x03,                                // bmAttributes = internal, non-fixed
    0x07,                                // bmControls = freq r/w, validity r
    0x00,                                // bAssocTerminal
    0x00,                                // iClockSource

    // ─── Clock Selector ID=40 (8 bytes) [offset 43] ───
    CLOCK_SELECTOR_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_CLOCK_SELECTOR,
    CLOCK_SELECTOR_ID,                   // bClockID = 40
    1,                                   // bNrInPins = 1
    CLOCK_SOURCE_ID,                     // baCSourceID[0] = 41
    0x03,                                // bmControls = selector r/w
    0x00,                                // iClockSelector

    // ─── Input Terminal ID=2 (17 bytes) [offset 51] — Playback ───
    INPUT_TERMINAL_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_INPUT_TERMINAL,
    INPUT_TERMINAL_PB_ID,                // bTerminalID = 2
    U16_TO_U8S_LE(AUDIO_TERM_TYPE_USB_STREAMING),
    0x00,                                // bAssocTerminal
    CLOCK_SELECTOR_ID,                   // bCSourceID = 40
    2,                                   // bNrChannels
    U32_TO_U8S_LE(0x00000000),          // bmChannelConfig
    0x00,                                // iChannelNames
    U16_TO_U8S_LE(0x0000),              // bmControls
    0x00,                                // iTerminal

    // ─── Feature Unit ID=10 (18 bytes) [offset 68] — Playback ───
    FEATURE_UNIT_PB_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_FEATURE_UNIT,
    FEATURE_UNIT_PB_ID,                  // bUnitID = 10
    INPUT_TERMINAL_PB_ID,                // bSourceID = 2
    U32_TO_U8S_LE(0x0000000F),          // bmaControls(0) master — mute+vol+bass+mid
    U32_TO_U8S_LE(0x0000000F),          // bmaControls(1) ch1
    U32_TO_U8S_LE(0x0000000F),          // bmaControls(2) ch2
    0x00,                                // iFeature

    // ─── Output Terminal ID=20 (12 bytes) [offset 86] — Speaker ───
    OUTPUT_TERMINAL_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_OUTPUT_TERMINAL,
    OUTPUT_TERMINAL_PB_ID,               // bTerminalID = 20
    U16_TO_U8S_LE(AUDIO_TERM_TYPE_OUT_DESKTOP_SPEAKER),
    0x00,                                // bAssocTerminal
    FEATURE_UNIT_PB_ID,                  // bSourceID = 10
    CLOCK_SELECTOR_ID,                   // bCSourceID = 40
    U16_TO_U8S_LE(0x0000),              // bmControls
    0x00,                                // iTerminal

    // ─── Input Terminal ID=1 (17 bytes) [offset 98] — Capture (Microphone) ───
    INPUT_TERMINAL_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_INPUT_TERMINAL,
    INPUT_TERMINAL_CAP_ID,               // bTerminalID = 1
    U16_TO_U8S_LE(0x0201),              // wTerminalType = MICROPHONE
    0x00,                                // bAssocTerminal
    CLOCK_SELECTOR_ID,                   // bCSourceID = 40
    2,                                   // bNrChannels
    U32_TO_U8S_LE(0x00000000),          // bmChannelConfig
    0x00,                                // iChannelNames
    U16_TO_U8S_LE(0x0000),              // bmControls
    0x00,                                // iTerminal

    // ─── Feature Unit ID=11 (26 bytes) [offset 115] — Capture ───
    FEATURE_UNIT_CAP_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_FEATURE_UNIT,
    FEATURE_UNIT_CAP_ID,                 // bUnitID = 11
    INPUT_TERMINAL_CAP_ID,               // bSourceID = 1
    U32_TO_U8S_LE(0x0000000F),          // bmaControls(0) master
    U32_TO_U8S_LE(0x0000000F),          // bmaControls(1) ch1
    U32_TO_U8S_LE(0x0000000F),          // bmaControls(2) ch2
    U32_TO_U8S_LE(0x0000000F),          // bmaControls(3) ch3
    U32_TO_U8S_LE(0x0000000F),          // bmaControls(4) ch4
    0x00,                                // iFeature

    // ─── Output Terminal ID=22 (12 bytes) [offset 141] — Capture to USB ───
    OUTPUT_TERMINAL_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_OUTPUT_TERMINAL,
    OUTPUT_TERMINAL_CAP_ID,              // bTerminalID = 22
    U16_TO_U8S_LE(AUDIO_TERM_TYPE_USB_STREAMING),
    0x00,                                // bAssocTerminal
    FEATURE_UNIT_CAP_ID,                 // bSourceID = 11
    CLOCK_SELECTOR_ID,                   // bCSourceID = 40
    U16_TO_U8S_LE(0x0000),              // bmControls
    0x00,                                // iTerminal

    // ═══════════════════════════════════════════════════════════════
    //  AS Interface 1 (Playback) — Alt 0 (zero-bandwidth) [offset 153]
    // ═══════════════════════════════════════════════════════════════
    9, TUSB_DESC_INTERFACE,
    ITF_NUM_AUDIO_STREAMING_PB, 0, 0,
    TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_STREAMING, AUDIO_INT_PROTOCOL_CODE_V2, 0,

    // ═══════════════════════════════════════════════════════════════
    //  AS Interface 1, Alt 1 — 24-bit stereo playback [offset 162]
    // ═══════════════════════════════════════════════════════════════
    9, TUSB_DESC_INTERFACE,
    ITF_NUM_AUDIO_STREAMING_PB, 1, 2,   // alt 1, 2 endpoints
    TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_STREAMING, AUDIO_INT_PROTOCOL_CODE_V2, 0,

    // AS General (16 bytes) [offset 171]
    AS_GENERAL_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AS_INTERFACE_AS_GENERAL,
    INPUT_TERMINAL_PB_ID,                // bTerminalLink = 2
    0x00,                                // bmControls
    AUDIO_FORMAT_TYPE_I,
    U32_TO_U8S_LE(0x00000001),          // bmFormats = PCM
    2,                                   // bNrChannels
    U32_TO_U8S_LE(0x00000000),          // bmChannelConfig
    0x00,                                // iChannelNames

    // Format Type I — 24-bit (6 bytes) [offset 187]
    FORMAT_TYPE_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AS_INTERFACE_FORMAT_TYPE,
    AUDIO_FORMAT_TYPE_I, 3, 24,          // 3-byte subslot, 24-bit

    // EP 0x01 OUT — async iso, MPS=294 (7 bytes) [offset 193]
    7, TUSB_DESC_ENDPOINT, EPNUM_AUDIO_OUT,
    0x05, U16_TO_U8S_LE(294), 1,        // async iso, interval=1

    // CS EP General (8 bytes) [offset 200]
    CS_EP_LEN, TUSB_DESC_CS_ENDPOINT, AUDIO_CS_EP_SUBTYPE_GENERAL,
    0x00, 0x00, 0x02, U16_TO_U8S_LE(0x0008),   // lock delay 8ms

    // Feedback EP 0x81 IN (7 bytes) [offset 208]
    7, TUSB_DESC_ENDPOINT, EPNUM_AUDIO_FB,
    0x11, U16_TO_U8S_LE(4), 4,          // iso feedback, interval=4 (8ms)

    // ═══════════════════════════════════════════════════════════════
    //  AS Interface 1, Alt 2 — 16-bit stereo playback [offset 215]
    // ═══════════════════════════════════════════════════════════════
    9, TUSB_DESC_INTERFACE,
    ITF_NUM_AUDIO_STREAMING_PB, 2, 2,   // alt 2, 2 endpoints
    TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_STREAMING, AUDIO_INT_PROTOCOL_CODE_V2, 0,

    // AS General (16 bytes) [offset 224]
    AS_GENERAL_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AS_INTERFACE_AS_GENERAL,
    INPUT_TERMINAL_PB_ID,
    0x00, AUDIO_FORMAT_TYPE_I,
    U32_TO_U8S_LE(0x00000001),          // PCM
    2, U32_TO_U8S_LE(0x00000000), 0x00,

    // Format Type I — 16-bit (6 bytes) [offset 240]
    FORMAT_TYPE_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AS_INTERFACE_FORMAT_TYPE,
    AUDIO_FORMAT_TYPE_I, 2, 16,          // 2-byte subslot, 16-bit

    // EP 0x01 OUT — async iso, MPS=196 (7 bytes) [offset 246]
    7, TUSB_DESC_ENDPOINT, EPNUM_AUDIO_OUT,
    0x05, U16_TO_U8S_LE(196), 1,

    // CS EP General (8 bytes) [offset 253]
    CS_EP_LEN, TUSB_DESC_CS_ENDPOINT, AUDIO_CS_EP_SUBTYPE_GENERAL,
    0x00, 0x00, 0x02, U16_TO_U8S_LE(0x0008),

    // Feedback EP 0x81 IN (7 bytes) [offset 261]
    7, TUSB_DESC_ENDPOINT, EPNUM_AUDIO_FB,
    0x11, U16_TO_U8S_LE(4), 4,

    // ═══════════════════════════════════════════════════════════════
    //  HID Interface 2 [offset 268]
    // ═══════════════════════════════════════════════════════════════
    9, TUSB_DESC_INTERFACE,
    ITF_NUM_HID, 0, 2,                  // 2 endpoints (IN + OUT)
    0x03,                                // bInterfaceClass = HID
    0x00,                                // bInterfaceSubClass = no boot
    0x00,                                // bInterfaceProtocol
    0x00,                                // iInterface

    // HID Descriptor (9 bytes) [offset 277]
    9, 0x21,                             // bDescriptorType = HID
    U16_TO_U8S_LE(0x0110),              // bcdHID = 1.10
    0x00,                                // bCountryCode
    1,                                   // bNumDescriptors
    0x22,                                // bDescriptorType = Report
    U16_TO_U8S_LE(34),                  // wDescriptorLength = 34

    // EP 0x82 IN — interrupt, MPS=64 (7 bytes) [offset 286]
    7, TUSB_DESC_ENDPOINT, EPNUM_HID_IN,
    0x03, U16_TO_U8S_LE(64), 1,         // interrupt, 1ms

    // EP 0x02 OUT — interrupt, MPS=64 (7 bytes) [offset 293]
    7, TUSB_DESC_ENDPOINT, EPNUM_HID_OUT,
    0x03, U16_TO_U8S_LE(64), 1,

    // DFU interface omitted — no test value, and TinyUSB asserts on unclaimed interfaces
};

_Static_assert(sizeof(desc_configuration) == 300, "Config descriptor must be 300 bytes");

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return desc_configuration;
}

// --- HID Report Descriptor (34 bytes) ---
static uint8_t const desc_hid_report[] = {
    0x06, 0x00, 0xFF,  // Usage Page (Vendor Defined)
    0x09, 0x01,        // Usage (Vendor Usage 1)
    0xA1, 0x01,        // Collection (Application)
    0x09, 0x02,        //   Usage (Vendor Usage 2)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x75, 0x08,        //   Report Size (8 bits)
    0x95, 0x40,        //   Report Count (64)
    0x81, 0x02,        //   Input (Data, Variable, Absolute)
    0x09, 0x03,        //   Usage (Vendor Usage 3)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x75, 0x08,        //   Report Size (8 bits)
    0x95, 0x40,        //   Report Count (64)
    0x91, 0x02,        //   Output (Data, Variable, Absolute)
    0xC0               // End Collection
};

_Static_assert(sizeof(desc_hid_report) == 34, "HID report descriptor must be 34 bytes");

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    (void)instance;
    return desc_hid_report;
}

// --- String Descriptors ---
static char const *string_desc_arr[] = {
    (const char[]){0x09, 0x04},  // 0: English
    "miniDSP",                    // 1: Manufacturer (iManufacturer=1)
    "",                           // 2: (unused — real device skips this index)
    "2x4HD",                      // 3: Product (iProduct=3, matches real device)
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
            current_sample_rate = (uint32_t)((audio_control_cur_4_t const *)pBuff)->bCur;
            ESP_LOGI(TAG, "SET sample rate: %lu Hz", (unsigned long)current_sample_rate);
            // Don't call tud_audio_fb_set here — TinyUSB manages feedback
            // internally. Calling it during a control transfer can assert.
            return true;
        }
    }

    // Playback Feature Unit (ID=10): mute, volume
    if (entityID == FEATURE_UNIT_PB_ID) {
        if (ctrlSel == AUDIO_FU_CTRL_MUTE && channelNum < 3) {
            TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_1_t));
            pb_mute[channelNum] = ((audio_control_cur_1_t *)pBuff)->bCur;
            ESP_LOGI(TAG, "SET PB mute ch%d: %d", channelNum, pb_mute[channelNum]);
            return true;
        }
        if (ctrlSel == AUDIO_FU_CTRL_VOLUME && channelNum < 3) {
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
                ESP_LOGI(TAG, "GET sample rate CUR -> %lu Hz", (unsigned long)current_sample_rate);
                audio_control_cur_4_t cur = { .bCur = (int32_t)current_sample_rate };
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
        if (ctrlSel == AUDIO_FU_CTRL_MUTE && channelNum < 3) {
            audio_control_cur_1_t cur = { .bCur = pb_mute[channelNum] };
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &cur, sizeof(cur));
        }
        if (ctrlSel == AUDIO_FU_CTRL_VOLUME) {
            if (p_request->bRequest == AUDIO_CS_REQ_CUR && channelNum < 3) {
                audio_control_cur_2_t cur = { .bCur = pb_volume[channelNum] };
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

        if (alt > 0) {
            // Feedback is handled by TinyUSB internally after SET_INTERFACE.
            // We set the initial value in the SET sample rate handler instead.
        }
    }
    return true;
}

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
static void cmd_read_floats(const uint8_t *payload, uint8_t len, uint8_t *resp, uint8_t *resp_len)
{
    if (len < 4) { resp[0] = 0x00; *resp_len = 1; return; }

    uint16_t addr = ((uint16_t)payload[1] << 8) | payload[2];
    uint8_t count = payload[3];
    if (count > 14) count = 14;  // protocol max

    ESP_LOGI(TAG, "HID: ReadFloats addr=0x%04X count=%d", addr, count);

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
static void cmd_write_dsp(const uint8_t *payload, uint8_t len, uint8_t *resp, uint8_t *resp_len)
{
    if (len < 5) { resp[0] = 0x00; *resp_len = 1; return; }

    // payload[1] = 0x80 (save) or 0xA0 (RAM-only)
    // payload[2..3] = address (2 bytes for hw_id=10, 2x4 HD)
    // payload[4..7] = value (4 bytes float, LE)
    uint8_t mode = payload[1];
    uint16_t addr = ((uint16_t)payload[2] << 8) | payload[3];
    int val_offset = 4;

    if (val_offset + 4 <= len) {
        float val;
        uint32_t bits = (uint32_t)payload[val_offset] | ((uint32_t)payload[val_offset+1] << 8) |
                        ((uint32_t)payload[val_offset+2] << 16) | ((uint32_t)payload[val_offset+3] << 24);
        memcpy(&val, &bits, 4);
        dsp_param_set(addr, val);
        ESP_LOGI(TAG, "HID: WriteDSP %s addr=0x%04X val=%.6f",
                 mode == 0x80 ? "SAVE" : "RAM", addr, val);
    }

    resp[0] = 0x01;  // ACK
    *resp_len = 1;
}

// cmd 0x30: WriteBiquad
static void cmd_write_biquad(const uint8_t *payload, uint8_t len, uint8_t *resp, uint8_t *resp_len)
{
    // [0x30, 0x80, addr_hi, addr_lo, 0x00, 0x00, 5×float32_LE]
    if (len < 26) { resp[0] = 0x00; *resp_len = 1; return; }

    uint16_t addr = ((uint16_t)payload[2] << 8) | payload[3];
    ESP_LOGI(TAG, "HID: WriteBiquad addr=0x%04X", addr);

    // Store 5 coefficients (b0, b1, b2, a1, a2)
    for (int i = 0; i < 5; i++) {
        int off = 6 + i * 4;
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
static void cmd_bypass_filter(const uint8_t *payload, uint8_t len, uint8_t *resp, uint8_t *resp_len)
{
    if (len < 4) { resp[0] = 0x00; *resp_len = 1; return; }
    uint8_t bypass = payload[1];
    uint16_t addr = ((uint16_t)payload[2] << 8) | payload[3];
    ESP_LOGI(TAG, "HID: BypassFilter addr=0x%04X bypass=%s", addr, bypass ? "ON" : "OFF");
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
                 (unsigned long)current_sample_rate, clock_valid);
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
    ESP_LOGI(TAG, "Audio: Playback IF1 (24-bit alt1 + 16-bit alt2)");
    ESP_LOGI(TAG, "HID:   IF2 (64-byte vendor reports, full command protocol)");
    ESP_LOGI(TAG, "Clock: Source ID=%d, Selector ID=%d (44.1/48 kHz)",
             CLOCK_SOURCE_ID, CLOCK_SELECTOR_ID);
    ESP_LOGI(TAG, "Playback: IT%d → FU%d → OT%d (Speaker)",
             INPUT_TERMINAL_PB_ID, FEATURE_UNIT_PB_ID, OUTPUT_TERMINAL_PB_ID);
    ESP_LOGI(TAG, "Capture:  IT%d → FU%d → OT%d (USB Streaming)",
             INPUT_TERMINAL_CAP_ID, FEATURE_UNIT_CAP_ID, OUTPUT_TERMINAL_CAP_ID);
    ESP_LOGI(TAG, "Endpoints: 0x01 OUT (pb), 0x81 IN (fb), 0x82/0x02 (HID)");
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
