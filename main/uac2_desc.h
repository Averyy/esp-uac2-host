/**
 * @file uac2_desc.h
 * @brief USB Audio Class 2.0 descriptor definitions and parser
 *
 * Struct definitions derived from USBX ux_class_audio20.h (MIT) and
 * CherryUSB usb_audio.h (Apache-2.0), rewritten with standard C types
 * for ESP-IDF.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ── USB Audio Class codes ───────────────────────────────────────────

#define UAC2_CLASS_AUDIO                0x01
#define UAC2_SUBCLASS_AUDIOCONTROL      0x01
#define UAC2_SUBCLASS_AUDIOSTREAMING    0x02
#define UAC2_SUBCLASS_MIDISTREAMING     0x03
#define UAC2_PROTOCOL_VERSION_02_00     0x20

// Descriptor types
#define UAC2_CS_INTERFACE               0x24
#define UAC2_CS_ENDPOINT                0x25

// AC interface descriptor subtypes
#define UAC2_AC_HEADER                  0x01
#define UAC2_AC_INPUT_TERMINAL          0x02
#define UAC2_AC_OUTPUT_TERMINAL         0x03
#define UAC2_AC_MIXER_UNIT              0x04
#define UAC2_AC_SELECTOR_UNIT           0x05
#define UAC2_AC_FEATURE_UNIT            0x06
#define UAC2_AC_EFFECT_UNIT             0x07
#define UAC2_AC_PROCESSING_UNIT         0x08
#define UAC2_AC_EXTENSION_UNIT          0x09
#define UAC2_AC_CLOCK_SOURCE            0x0A
#define UAC2_AC_CLOCK_SELECTOR          0x0B
#define UAC2_AC_CLOCK_MULTIPLIER        0x0C
#define UAC2_AC_SAMPLE_RATE_CONVERTER   0x0D

// AS interface descriptor subtypes
#define UAC2_AS_GENERAL                 0x01
#define UAC2_AS_FORMAT_TYPE             0x02

// Endpoint descriptor subtypes
#define UAC2_EP_GENERAL                 0x01

// Terminal types
#define UAC2_TERMINAL_USB_STREAMING     0x0101
#define UAC2_TERMINAL_MICROPHONE        0x0201
#define UAC2_TERMINAL_SPEAKER           0x0301

// Format types
#define UAC2_FORMAT_TYPE_I              0x01
#define UAC2_FORMAT_PCM                 0x00000001

// Clock source attributes
#define UAC2_CLOCK_EXTERNAL             0x00
#define UAC2_CLOCK_INTERNAL_FIXED       0x01
#define UAC2_CLOCK_INTERNAL_VARIABLE    0x02
#define UAC2_CLOCK_INTERNAL_PROGRAMMABLE 0x03

// Audio function categories
#define UAC2_CATEGORY_IO_BOX            0x08

// Control request codes
#define UAC2_REQUEST_CUR                0x01
#define UAC2_REQUEST_RANGE              0x02
#define UAC2_REQUEST_MEM                0x03

// Clock source control selectors
#define UAC2_CS_SAM_FREQ_CONTROL        0x01
#define UAC2_CS_CLOCK_VALID_CONTROL     0x02

// ── Descriptor structs (packed) ─────────────────────────────────────

// AC Interface Header (UAC2 - different from UAC1!)
typedef struct __attribute__((packed)) {
    uint8_t  bLength;
    uint8_t  bDescriptorType;       // UAC2_CS_INTERFACE
    uint8_t  bDescriptorSubtype;    // UAC2_AC_HEADER
    uint16_t bcdADC;                // 0x0200 for UAC2
    uint8_t  bCategory;
    uint16_t wTotalLength;          // total AC descriptor length
    uint8_t  bmControls;
} uac2_ac_header_desc_t;

// Clock Source
typedef struct __attribute__((packed)) {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;    // UAC2_AC_CLOCK_SOURCE
    uint8_t  bClockID;
    uint8_t  bmAttributes;          // clock type + sync to SOF
    uint8_t  bmControls;            // freq control + validity
    uint8_t  bAssocTerminal;
    uint8_t  iClockSource;
} uac2_clock_source_desc_t;

// Clock Selector (variable length — bNrInPins sources follow)
typedef struct __attribute__((packed)) {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;    // UAC2_AC_CLOCK_SELECTOR
    uint8_t  bClockID;
    uint8_t  bNrInPins;
    // uint8_t baCSourceID[bNrInPins]  -- variable
    // uint8_t bmControls              -- after array
    // uint8_t iClockSelector          -- after bmControls
} uac2_clock_selector_desc_t;

// Clock Multiplier
typedef struct __attribute__((packed)) {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;    // UAC2_AC_CLOCK_MULTIPLIER
    uint8_t  bClockID;
    uint8_t  bCSourceID;
    uint8_t  bmControls;
    uint8_t  iClockMultiplier;
} uac2_clock_multiplier_desc_t;

// Input Terminal
typedef struct __attribute__((packed)) {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;    // UAC2_AC_INPUT_TERMINAL
    uint8_t  bTerminalID;
    uint16_t wTerminalType;
    uint8_t  bAssocTerminal;
    uint8_t  bCSourceID;            // clock source/selector ID
    uint8_t  bNrChannels;
    uint32_t bmChannelConfig;
    uint8_t  iChannelNames;
    uint16_t bmControls;
    uint8_t  iTerminal;
} uac2_input_terminal_desc_t;

// Output Terminal
typedef struct __attribute__((packed)) {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;    // UAC2_AC_OUTPUT_TERMINAL
    uint8_t  bTerminalID;
    uint16_t wTerminalType;
    uint8_t  bAssocTerminal;
    uint8_t  bSourceID;
    uint8_t  bCSourceID;            // clock source/selector ID
    uint16_t bmControls;
    uint8_t  iTerminal;
} uac2_output_terminal_desc_t;

// Feature Unit (variable length — bmaControls array depends on channel count)
typedef struct __attribute__((packed)) {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;    // UAC2_AC_FEATURE_UNIT
    uint8_t  bUnitID;
    uint8_t  bSourceID;
    // uint32_t bmaControls[]  -- (ch+1) entries of 4 bytes each
    // uint8_t  iFeature       -- after array
} uac2_feature_unit_desc_t;

// AS General (AudioStreaming interface descriptor)
typedef struct __attribute__((packed)) {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;    // UAC2_AS_GENERAL
    uint8_t  bTerminalLink;
    uint8_t  bmControls;
    uint8_t  bFormatType;
    uint32_t bmFormats;
    uint8_t  bNrChannels;
    uint32_t bmChannelConfig;
    uint8_t  iChannelNames;
} uac2_as_general_desc_t;

// Format Type I
typedef struct __attribute__((packed)) {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bDescriptorSubtype;    // UAC2_AS_FORMAT_TYPE
    uint8_t  bFormatType;           // UAC2_FORMAT_TYPE_I
    uint8_t  bSubSlotSize;          // bytes per subslot (e.g., 3 for 24-bit)
    uint8_t  bBitResolution;        // bits per sample (e.g., 24)
} uac2_format_type_i_desc_t;

// CS Endpoint General
typedef struct __attribute__((packed)) {
    uint8_t  bLength;
    uint8_t  bDescriptorType;       // UAC2_CS_ENDPOINT
    uint8_t  bDescriptorSubtype;    // UAC2_EP_GENERAL
    uint8_t  bmAttributes;
    uint8_t  bmControls;
    uint8_t  bLockDelayUnits;
    uint16_t wLockDelay;
} uac2_cs_endpoint_desc_t;

// ── Parsed result structs ───────────────────────────────────────────

#define UAC2_MAX_CLOCK_SOURCES      4
#define UAC2_MAX_CLOCK_SELECTORS    4
#define UAC2_MAX_TERMINALS          8
#define UAC2_MAX_FEATURE_UNITS      4
#define UAC2_MAX_AS_INTERFACES      4
#define UAC2_MAX_ALT_SETTINGS       4

typedef struct {
    uint8_t  clock_id;
    uint8_t  attributes;            // clock type
    uint8_t  controls;
    uint8_t  assoc_terminal;
} uac2_clock_source_t;

typedef struct {
    uint8_t  clock_id;
    uint8_t  nr_pins;
    uint8_t  source_ids[4];         // up to 4 input clock sources
} uac2_clock_selector_t;

typedef struct {
    uint8_t  terminal_id;
    uint16_t terminal_type;
    uint8_t  clock_source_id;
    uint8_t  nr_channels;
    bool     is_input;              // true = input terminal, false = output
    uint8_t  source_id;             // for output terminal: source unit ID
} uac2_terminal_t;

typedef struct {
    uint8_t  unit_id;
    uint8_t  source_id;
    uint8_t  nr_channels;           // derived from bLength
} uac2_feature_unit_t;

typedef struct {
    uint8_t  interface_num;
    uint8_t  alt_setting;
    uint8_t  terminal_link;
    uint8_t  format_type;
    uint8_t  nr_channels;
    uint8_t  sub_slot_size;         // bytes per sample (e.g., 3)
    uint8_t  bit_resolution;        // bits per sample (e.g., 24)
    // Endpoint info
    uint8_t  ep_addr;               // data endpoint address
    uint8_t  ep_attributes;         // sync type etc.
    uint16_t ep_max_packet_size;
    uint8_t  ep_interval;
    // Feedback endpoint (if async)
    uint8_t  fb_ep_addr;            // 0 if no feedback endpoint
    uint16_t fb_ep_max_packet_size;
    uint8_t  fb_ep_interval;
} uac2_as_iface_t;

typedef struct {
    bool     is_uac2;
    uint16_t bcdADC;
    uint8_t  category;
    uint8_t  ac_iface_num;          // Audio Control interface number

    // Clock topology
    uint8_t              num_clock_sources;
    uac2_clock_source_t  clock_sources[UAC2_MAX_CLOCK_SOURCES];
    uint8_t              num_clock_selectors;
    uac2_clock_selector_t clock_selectors[UAC2_MAX_CLOCK_SELECTORS];

    // Terminals
    uint8_t              num_terminals;
    uac2_terminal_t      terminals[UAC2_MAX_TERMINALS];

    // Feature units
    uint8_t              num_feature_units;
    uac2_feature_unit_t  feature_units[UAC2_MAX_FEATURE_UNITS];

    // Audio streaming interfaces (one per alt setting with endpoints)
    uint8_t              num_as_ifaces;
    uac2_as_iface_t      as_ifaces[UAC2_MAX_AS_INTERFACES];
} uac2_device_info_t;

// ── API ─────────────────────────────────────────────────────────────

/**
 * Parse a USB configuration descriptor and extract UAC2 information.
 *
 * @param config_desc  Pointer to the raw configuration descriptor bytes
 * @param total_length Total length of the configuration descriptor
 * @param info         Output: parsed UAC2 device info
 * @return true if a UAC2 audio function was found, false otherwise
 */
bool uac2_parse_config_descriptor(const uint8_t *config_desc, uint16_t total_length,
                                  uac2_device_info_t *info);

/**
 * Log parsed UAC2 device info to serial.
 */
void uac2_log_device_info(const uac2_device_info_t *info);

/**
 * Get a human-readable string for a terminal type.
 */
const char *uac2_terminal_type_str(uint16_t terminal_type);

#ifdef __cplusplus
}
#endif
