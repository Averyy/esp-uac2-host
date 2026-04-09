/*
 * TinyUSB configuration for miniDSP 2x4 HD simulator
 * Composite device: UAC2 audio (playback + capture) + HID (vendor control)
 * Matches real miniDSP 2x4 HD: 5 interfaces (AC, AS_PB, AS_CAP, DFU, HID)
 */

#pragma once

#include "sdkconfig.h"
#include "tusb_option.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CFG_TUSB_RHPORT0_MODE   (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#define CFG_TUSB_OS             OPT_OS_FREERTOS
#define CFG_TUSB_MEM_ALIGN      TU_ATTR_ALIGNED(4)
#define CFG_TUSB_DEBUG          1
#define CFG_TUSB_DEBUG_PRINTF   esp_rom_printf

#define CFG_TUD_ENDPOINT0_SIZE  64
#define CFG_TUD_DWC2_SLAVE_ENABLE 1

// Class enables — audio + HID composite
#define CFG_TUD_CDC             0
#define CFG_TUD_MSC             0
#define CFG_TUD_HID             1
#define CFG_TUD_MIDI            0
#define CFG_TUD_VENDOR          0
#define CFG_TUD_DFU             0

// ── Audio class config ──────────────────────────────────────────────

#define CFG_TUD_AUDIO                               1

// Audio function descriptor length — playback + capture AS interfaces.
// Normal build:
//   IAD(8) + AC_IF(9) + AC_entities(127) + AS_PB_alt0(9) + AS_PB_alt1(53)
//   + AS_PB_alt2(53) + AS_CAP_alt0(9) + AS_CAP_alt1(46) = 314
// No-feedback playback build:
//   IAD(8) + AC_IF(9) + AC_entities(127) + AS_PB_alt0(9) + AS_PB_alt1(46)
//   + AS_PB_alt2(46) + AS_CAP_alt0(9) = 254
#if CONFIG_MINIDSP_SIM_NO_FEEDBACK_TEST_BUILD
#define CFG_TUD_AUDIO_FUNC_1_DESC_LEN               254
#else
#define CFG_TUD_AUDIO_FUNC_1_DESC_LEN               314
#endif
#define CFG_TUD_AUDIO_FUNC_1_N_AS_INT               2   // playback + capture
#define CFG_TUD_AUDIO_FUNC_1_CTRL_BUF_SZ            64

// Playback (host → device, EP OUT)
#define CFG_TUD_AUDIO_ENABLE_EP_OUT                 1
#define CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_RX  3       // 24-bit max
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX          2       // stereo
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX          294     // 48kHz/24-bit/2ch + async headroom
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ       (294 * 4)

// Capture (device → host, EP IN) — present in the real-device build.
// The no-feedback playback test build drops the capture stream to stay within
// ESP32-S3 FIFO limits while exercising the host-side no-feedback path.
#if CONFIG_MINIDSP_SIM_NO_FEEDBACK_TEST_BUILD
#define CFG_TUD_AUDIO_ENABLE_EP_IN                  0
#else
#define CFG_TUD_AUDIO_ENABLE_EP_IN                  1
#endif
#define CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX  3       // 24-bit
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX          2       // stereo
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX           294
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ        (294 * 2)  // minimal buffer, never used

// Feedback endpoint — sends rate feedback to host
#if CONFIG_MINIDSP_SIM_NO_FEEDBACK_TEST_BUILD
#define CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP            0
#else
#define CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP            1
#endif
// Disable format correction: real miniDSP sends 4-byte 16.16 even at Full Speed
#define CFG_TUD_AUDIO_ENABLE_FEEDBACK_FORMAT_CORRECTION 0

// Sample rate
#define CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE            48000

// ── HID class config ────────────────────────────────────────────────

#define CFG_TUD_HID_EP_BUFSIZE  64  // 64-byte vendor reports (no report ID)

#ifdef __cplusplus
}
#endif
