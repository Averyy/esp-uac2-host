/*
 * UAC2 Test Device — miniDSP 2x4 HD Simulator
 *
 * Presents UAC2 descriptors mimicking the miniDSP 2x4 HD:
 * - Clock Source (ID=41, internal programmable)
 * - Clock Selector (ID=40, 1 input from source 41)
 * - Input Terminal (ID=2, USB Streaming, 2ch)
 * - Feature Unit (ID=10, src=2, 2ch)
 * - Output Terminal (ID=20, Speaker, src=10)
 * - AS Interface 1 Alt 1: 24-bit stereo, EP 0x01 OUT, feedback EP 0x81 IN
 *
 * Responds to CUR/RANGE requests for sample rate (48kHz only at FS),
 * clock validity, volume, and mute. Accepts and discards isochronous
 * audio data. Sends 48kHz feedback on EP 0x81.
 *
 * SPDX-License-Identifier: MIT
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_private/usb_phy.h"
#include "tusb.h"

static const char *TAG = "uac2-test-dev";
static const DRAM_ATTR char TAG_ISR[] = "uac2-test-dev";  // ISR-safe copy in DRAM

// ── USB PHY init (replaces esp_tinyusb wrapper) ────────────────────

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

// ── Audio control state ────────────────────────────────────────────

static uint32_t current_sample_rate = 48000;
static uint8_t  clock_valid = 1;
static bool     mute_state[3] = {false, false, false};  // master + 2ch
static int16_t  volume_state[3] = {0, 0, 0};            // in 1/256 dB
static uint32_t total_audio_bytes = 0;
static uint32_t audio_frames_received = 0;

// ── Descriptors (miniDSP 2x4 HD layout) ────────────────────────────

// Entity IDs matching miniDSP exactly
#define CLOCK_SOURCE_ID     41
#define CLOCK_SELECTOR_ID   40
#define INPUT_TERMINAL_ID   2       // USB Streaming -> device (playback)
#define FEATURE_UNIT_ID     10
#define OUTPUT_TERMINAL_ID  20      // Speaker

enum {
    ITF_NUM_AUDIO_CONTROL = 0,
    ITF_NUM_AUDIO_STREAMING,
    ITF_NUM_TOTAL
};

#define EPNUM_AUDIO_OUT     0x01
#define EPNUM_AUDIO_FB      0x81

// --- Device Descriptor ---
tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = 0x2752,       // miniDSP VID
    .idProduct          = 0x9999,       // different PID to avoid confusion
    .bcdDevice          = 0x06F2,       // same as real miniDSP
    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber      = 3,
    .bNumConfigurations = 1
};

uint8_t const *tud_descriptor_device_cb(void)
{
    return (uint8_t const *)&desc_device;
}

// --- Configuration Descriptor (raw bytes, miniDSP-like) ---

// AC entity lengths
#define AC_HEADER_LEN       9
#define CLOCK_SOURCE_LEN    8
#define CLOCK_SELECTOR_LEN  8   // 7 + bNrInPins(1)
#define INPUT_TERMINAL_LEN  17
#define OUTPUT_TERMINAL_LEN 12
#define FEATURE_UNIT_LEN    18  // 6 + (2+1)*4 bytes bmaControls + 1 iFeature - 1
#define AC_TOTAL_LEN        (AC_HEADER_LEN + CLOCK_SOURCE_LEN + CLOCK_SELECTOR_LEN + \
                             INPUT_TERMINAL_LEN + FEATURE_UNIT_LEN + OUTPUT_TERMINAL_LEN)
// AS descriptor lengths
#define AS_GENERAL_LEN      16
#define FORMAT_TYPE_LEN     6
#define CS_EP_LEN           8

// Total config descriptor length
#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + \
                             /* IAD */ 8 + \
                             /* AC interface */ 9 + AC_TOTAL_LEN + \
                             /* AS interface alt 0 */ 9 + \
                             /* AS interface alt 1 */ 9 + AS_GENERAL_LEN + FORMAT_TYPE_LEN + \
                             /* EP out */ 7 + CS_EP_LEN + \
                             /* EP feedback */ 7)

static uint8_t const desc_configuration[] = {
    // ── Configuration Descriptor ──
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0xC0, 0),

    // ── IAD: Audio function ──
    8, TUSB_DESC_INTERFACE_ASSOCIATION, ITF_NUM_AUDIO_CONTROL, 2,
    TUSB_CLASS_AUDIO, 0x00, AUDIO_FUNC_PROTOCOL_CODE_V2, 0,

    // ── AC Interface 0 ──
    9, TUSB_DESC_INTERFACE, ITF_NUM_AUDIO_CONTROL, 0, 0,
    TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_CONTROL, AUDIO_INT_PROTOCOL_CODE_V2, 0,

    // AC Header (bcdADC=2.00, category=I/O Box)
    AC_HEADER_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_HEADER,
    U16_TO_U8S_LE(0x0200), 0x08, U16_TO_U8S_LE(AC_TOTAL_LEN), 0x00,

    // Clock Source ID=41 (internal programmable, freq r/w, validity r)
    CLOCK_SOURCE_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_CLOCK_SOURCE,
    CLOCK_SOURCE_ID, 0x03, 0x07, 0x00, 0x00,

    // Clock Selector ID=40 (1 input from source 41)
    CLOCK_SELECTOR_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_CLOCK_SELECTOR,
    CLOCK_SELECTOR_ID, 1, CLOCK_SOURCE_ID, 0x03, 0x00,

    // Input Terminal ID=2 (USB Streaming, 2ch stereo, clock=40)
    INPUT_TERMINAL_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_INPUT_TERMINAL,
    INPUT_TERMINAL_ID, U16_TO_U8S_LE(AUDIO_TERM_TYPE_USB_STREAMING),
    0x00, CLOCK_SELECTOR_ID, 2, U32_TO_U8S_LE(0x00000003), 0x00,
    U16_TO_U8S_LE(0x0000), 0x00,

    // Feature Unit ID=10 (src=2, master+2ch, mute+volume)
    FEATURE_UNIT_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_FEATURE_UNIT,
    FEATURE_UNIT_ID, INPUT_TERMINAL_ID,
    U32_TO_U8S_LE(0x00000003),  // master: mute + volume
    U32_TO_U8S_LE(0x00000003),  // ch1
    U32_TO_U8S_LE(0x00000003),  // ch2
    0x00,

    // Output Terminal ID=20 (Speaker, src=10, clock=40)
    OUTPUT_TERMINAL_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_OUTPUT_TERMINAL,
    OUTPUT_TERMINAL_ID, U16_TO_U8S_LE(AUDIO_TERM_TYPE_OUT_DESKTOP_SPEAKER),
    0x00, FEATURE_UNIT_ID, CLOCK_SELECTOR_ID,
    U16_TO_U8S_LE(0x0000), 0x00,

    // ── AS Interface 1, Alt 0 (zero-bandwidth) ──
    9, TUSB_DESC_INTERFACE, ITF_NUM_AUDIO_STREAMING, 0, 0,
    TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_STREAMING, AUDIO_INT_PROTOCOL_CODE_V2, 0,

    // ── AS Interface 1, Alt 1 (24-bit stereo playback) ──
    9, TUSB_DESC_INTERFACE, ITF_NUM_AUDIO_STREAMING, 1, 2,
    TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_STREAMING, AUDIO_INT_PROTOCOL_CODE_V2, 0,

    // AS General (terminal link=2, format type I, PCM, 2ch)
    AS_GENERAL_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AS_INTERFACE_AS_GENERAL,
    INPUT_TERMINAL_ID, 0x00, AUDIO_FORMAT_TYPE_I,
    U32_TO_U8S_LE(0x00000001), 2, U32_TO_U8S_LE(0x00000003), 0x00,

    // Format Type I (24-bit in 3-byte subslot)
    FORMAT_TYPE_LEN, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AS_INTERFACE_FORMAT_TYPE,
    AUDIO_FORMAT_TYPE_I, 3, 24,

    // EP 0x01 OUT (isochronous async, MPS=294, interval=1)
    7, TUSB_DESC_ENDPOINT, EPNUM_AUDIO_OUT,
    0x05, U16_TO_U8S_LE(294), 1,

    // CS EP General
    CS_EP_LEN, TUSB_DESC_CS_ENDPOINT, AUDIO_CS_EP_SUBTYPE_GENERAL,
    0x00, 0x00, 0x00, U16_TO_U8S_LE(0x0008),  // wLockDelay=8ms (same as miniDSP)

    // EP 0x81 IN feedback (isochronous, MPS=4, interval=4 = every 8 frames)
    7, TUSB_DESC_ENDPOINT, EPNUM_AUDIO_FB,
    0x11, U16_TO_U8S_LE(4), 4,
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return desc_configuration;
}

// --- String Descriptors ---
static char const *string_desc_arr[] = {
    (const char[]){0x09, 0x04},  // 0: English
    "miniDSP",                    // 1: Manufacturer (same as real device)
    "2x4HD UAC2 Simulator",       // 2: Product
    "TEST0001",                   // 3: Serial
};

static uint16_t _desc_str[32 + 1];

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

// ── Audio class callbacks ──────────────────────────────────────────

// SET requests on entities
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

            // Update feedback value for new rate
            // At FS, feedback is in 16.16 format (TinyUSB handles 10.14 conversion)
            uint32_t fb_value = ((uint32_t)current_sample_rate) << 16;
            fb_value /= 1000;  // Convert Hz to samples-per-frame (kHz in 16.16)
            tud_audio_fb_set(fb_value);

            return true;
        }
    }

    // Feature Unit (ID=10): mute, volume
    if (entityID == FEATURE_UNIT_ID) {
        if (ctrlSel == AUDIO_FU_CTRL_MUTE && channelNum < 3) {
            TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_1_t));
            mute_state[channelNum] = ((audio_control_cur_1_t *)pBuff)->bCur;
            ESP_LOGI(TAG, "SET mute ch%d: %d", channelNum, mute_state[channelNum]);
            return true;
        }
        if (ctrlSel == AUDIO_FU_CTRL_VOLUME && channelNum < 3) {
            TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_2_t));
            volume_state[channelNum] = ((audio_control_cur_2_t *)pBuff)->bCur;
            ESP_LOGI(TAG, "SET volume ch%d: %d (1/256 dB)", channelNum, volume_state[channelNum]);
            return true;
        }
    }

    // Clock Selector (ID=40): select input clock
    if (entityID == CLOCK_SELECTOR_ID) {
        if (ctrlSel == 0x01) {  // CX_CLOCK_SELECTOR_CONTROL
            ESP_LOGI(TAG, "SET clock selector: input %d", pBuff[0]);
            return true;  // Accept any selection (we only have 1 input)
        }
    }

    ESP_LOGW(TAG, "Unsupported SET entity=%d ctrl=%d ch=%d", entityID, ctrlSel, channelNum);
    return false;
}

// GET requests on entities
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
                // miniDSP at FS supports 44.1 and 48 kHz
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

    // Feature Unit (ID=10)
    if (entityID == FEATURE_UNIT_ID) {
        if (ctrlSel == AUDIO_FU_CTRL_MUTE && channelNum < 3) {
            ESP_LOGI(TAG, "GET mute ch%d -> %d", channelNum, mute_state[channelNum]);
            audio_control_cur_1_t cur = { .bCur = mute_state[channelNum] };
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &cur, sizeof(cur));
        }
        if (ctrlSel == AUDIO_FU_CTRL_VOLUME) {
            if (p_request->bRequest == AUDIO_CS_REQ_CUR && channelNum < 3) {
                ESP_LOGI(TAG, "GET volume CUR ch%d -> %d", channelNum, volume_state[channelNum]);
                audio_control_cur_2_t cur = { .bCur = volume_state[channelNum] };
                return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &cur, sizeof(cur));
            }
            if (p_request->bRequest == AUDIO_CS_REQ_RANGE) {
                ESP_LOGI(TAG, "GET volume RANGE ch%d", channelNum);
                // -90dB to 0dB in 1dB steps (miniDSP-like range)
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
        if (ctrlSel == 0x01) {  // CX_CLOCK_SELECTOR_CONTROL
            ESP_LOGI(TAG, "GET clock selector -> input 1");
            audio_control_cur_1_t cur = { .bCur = 1 };  // always input 1
            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, &cur, sizeof(cur));
        }
    }

    ESP_LOGW(TAG, "Unsupported GET entity=%d ctrl=%d ch=%d req=%d",
             entityID, ctrlSel, channelNum, p_request->bRequest);
    return false;
}

// Minimal required callbacks
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
    (void)rhport; (void)p_request;
    ESP_LOGI(TAG, "Stream stopped (alt 0 selected)");
    return true;
}

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    (void)rhport;
    uint8_t alt = (uint8_t)p_request->wValue;
    ESP_LOGI(TAG, "SET_INTERFACE alt=%d — %s",
             alt, alt > 0 ? "stream START" : "stream STOP");

    if (alt > 0) {
        // Set initial feedback for the current sample rate
        uint32_t fb_value = ((uint32_t)current_sample_rate) << 16;
        fb_value /= 1000;
        tud_audio_fb_set(fb_value);
        ESP_LOGI(TAG, "Feedback set to 0x%08lX (%lu.%03lu kHz)",
                 (unsigned long)fb_value,
                 (unsigned long)(fb_value >> 16),
                 (unsigned long)((fb_value & 0xFFFF) * 1000 / 65536));
    }
    return true;
}

// NOTE: Runs in USB ISR context — only use ISR-safe functions (no ESP_LOGx, no locks).
bool tud_audio_rx_done_isr(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting)
{
    (void)rhport; (void)func_id; (void)ep_out; (void)cur_alt_setting;

    total_audio_bytes += n_bytes_received;
    audio_frames_received++;

    // Log every 1000 frames (~1 second at 1ms per frame)
    if (audio_frames_received % 1000 == 0) {
        ESP_DRAM_LOGI(TAG_ISR, "Audio: %lu frames, %lu bytes total",
                      (unsigned long)audio_frames_received,
                      (unsigned long)total_audio_bytes);
    }

    // Flush received data
    uint8_t buf[294];
    while (n_bytes_received > 0) {
        uint16_t to_read = n_bytes_received > sizeof(buf) ? sizeof(buf) : n_bytes_received;
        uint16_t got = tud_audio_read(buf, to_read);
        n_bytes_received -= got;
        if (got == 0) break;
    }
    return true;
}

// ── Main ───────────────────────────────────────────────────────────

static void tusb_device_task(void *arg)
{
    (void)arg;
    while (1) {
        tud_task();
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "UAC2 Test Device — miniDSP 2x4 HD Simulator");
    ESP_LOGI(TAG, "  VID=0x%04X PID=0x%04X bcdDevice=0x%04X",
             desc_device.idVendor, desc_device.idProduct, desc_device.bcdDevice);
    ESP_LOGI(TAG, "  Clock Source ID=%d, Clock Selector ID=%d",
             CLOCK_SOURCE_ID, CLOCK_SELECTOR_ID);
    ESP_LOGI(TAG, "  Input Terminal ID=%d, Feature Unit ID=%d, Output Terminal ID=%d",
             INPUT_TERMINAL_ID, FEATURE_UNIT_ID, OUTPUT_TERMINAL_ID);
    ESP_LOGI(TAG, "  EP OUT=0x%02X (MPS=294), EP FB=0x%02X (interval=4)",
             EPNUM_AUDIO_OUT, EPNUM_AUDIO_FB);
    ESP_LOGI(TAG, "  24-bit stereo, 48kHz default, supports 44.1/48kHz");

    // Initialize USB PHY
    ESP_ERROR_CHECK(usb_phy_init());

    // Initialize TinyUSB
    tusb_rhport_init_t dev_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_FULL,
    };
    if (!tusb_init(0, &dev_init)) {
        ESP_LOGE(TAG, "TinyUSB init failed");
        return;
    }

    ESP_LOGI(TAG, "TinyUSB initialized, waiting for host...");

    // TinyUSB device task
    xTaskCreatePinnedToCore(tusb_device_task, "tusb_dev", 4096, NULL, 5, NULL, 0);
}
