/*
 * TinyUSB configuration for UAC2 test device (miniDSP 2x4 HD simulator)
 */

#pragma once

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

// Only audio class
#define CFG_TUD_CDC             0
#define CFG_TUD_MSC             0
#define CFG_TUD_HID             0
#define CFG_TUD_MIDI            0
#define CFG_TUD_VENDOR          0

// Audio class config — speaker (host sends audio to us)
#define CFG_TUD_AUDIO                               1
#define CFG_TUD_AUDIO_FUNC_1_DESC_LEN               TUD_AUDIO_DESC_LEN
#define CFG_TUD_AUDIO_FUNC_1_N_AS_INT               1
#define CFG_TUD_AUDIO_FUNC_1_CTRL_BUF_SZ            64

#define CFG_TUD_AUDIO_ENABLE_EP_OUT                 1
#define CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_RX  3       // 24-bit
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX          2       // stereo
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX          294     // match miniDSP MPS
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ       (294 * 4)

// Feedback endpoint — sends rate feedback to host
#define CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP            1
#define CFG_TUD_AUDIO_ENABLE_FEEDBACK_FORMAT_CORRECTION 1   // auto-convert 16.16 to 10.14 at FS

// Sample rate
#define CFG_TUD_AUDIO_FUNC_1_SAMPLE_RATE            48000

// TUD_AUDIO_DESC_LEN is set by main.c's descriptor (computed from raw bytes)
// We define it here so tusb_config.h is self-consistent
#ifndef TUD_AUDIO_DESC_LEN
#define TUD_AUDIO_DESC_LEN  151
#endif

#ifdef __cplusplus
}
#endif
