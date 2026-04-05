/**
 * @file uac2_desc.c
 * @brief USB Audio Class 2.0 descriptor parser
 *
 * SPDX-License-Identifier: MIT
 */

#include <string.h>
#include "uac2_desc.h"
#include "esp_log.h"

static const char *TAG = "uac2-desc";

// ── Terminal type strings ───────────────────────────────────────────

const char *uac2_terminal_type_str(uint16_t terminal_type)
{
    switch (terminal_type) {
    case 0x0100: return "USB Undefined";
    case 0x0101: return "USB Streaming";
    case 0x01FF: return "USB Vendor";
    case 0x0200: return "Input Undefined";
    case 0x0201: return "Microphone";
    case 0x0202: return "Desktop Microphone";
    case 0x0203: return "Personal Microphone";
    case 0x0204: return "Omni Microphone";
    case 0x0205: return "Microphone Array";
    case 0x0206: return "Proc Microphone Array";
    case 0x0300: return "Output Undefined";
    case 0x0301: return "Speaker";
    case 0x0302: return "Headphones";
    case 0x0303: return "Head Mounted Display";
    case 0x0304: return "Desktop Speaker";
    case 0x0305: return "Room Speaker";
    case 0x0306: return "Comm Speaker";
    case 0x0307: return "LFE Speaker";
    default:     return "Unknown";
    }
}

static const char *clock_type_str(uint8_t attr)
{
    switch (attr & 0x03) {
    case 0: return "External";
    case 1: return "Internal Fixed";
    case 2: return "Internal Variable";
    case 3: return "Internal Programmable";
    default: return "Unknown";
    }
}

// ── Helpers for reading unaligned little-endian values ───────────────

static inline uint16_t read_u16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline uint32_t read_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// ── AC descriptor parsing ───────────────────────────────────────────

static void parse_ac_header(const uint8_t *desc, uac2_device_info_t *info)
{
    info->bcdADC = read_u16(&desc[3]);
    info->category = desc[5];
    // wTotalLength at desc[6..7], bmControls at desc[8]

    if (info->bcdADC >= 0x0200) {
        info->is_uac2 = true;
    }
}

static void parse_clock_source(const uint8_t *desc, uac2_device_info_t *info)
{
    if (info->num_clock_sources >= UAC2_MAX_CLOCK_SOURCES) return;
    uac2_clock_source_t *cs = &info->clock_sources[info->num_clock_sources++];
    cs->clock_id = desc[3];
    cs->attributes = desc[4];
    cs->controls = desc[5];
    cs->assoc_terminal = desc[6];
}

static void parse_clock_selector(const uint8_t *desc, uac2_device_info_t *info)
{
    if (info->num_clock_selectors >= UAC2_MAX_CLOCK_SELECTORS) return;
    uac2_clock_selector_t *cx = &info->clock_selectors[info->num_clock_selectors++];
    cx->clock_id = desc[3];
    cx->nr_pins = desc[4];
    for (int i = 0; i < cx->nr_pins && i < 4; i++) {
        cx->source_ids[i] = desc[5 + i];
    }
}

static void parse_input_terminal(const uint8_t *desc, uac2_device_info_t *info)
{
    if (info->num_terminals >= UAC2_MAX_TERMINALS) return;
    uac2_terminal_t *t = &info->terminals[info->num_terminals++];
    t->is_input = true;
    t->terminal_id = desc[3];
    t->terminal_type = read_u16(&desc[4]);
    // bAssocTerminal at desc[6]
    t->clock_source_id = desc[7];
    t->nr_channels = desc[8];
    t->source_id = 0; // not applicable for input terminals
}

static void parse_output_terminal(const uint8_t *desc, uac2_device_info_t *info)
{
    if (info->num_terminals >= UAC2_MAX_TERMINALS) return;
    uac2_terminal_t *t = &info->terminals[info->num_terminals++];
    t->is_input = false;
    t->terminal_id = desc[3];
    t->terminal_type = read_u16(&desc[4]);
    // bAssocTerminal at desc[6]
    t->source_id = desc[7];
    t->clock_source_id = desc[8];
    t->nr_channels = 0; // output terminals don't declare channels
}

static void parse_feature_unit(const uint8_t *desc, uac2_device_info_t *info)
{
    if (info->num_feature_units >= UAC2_MAX_FEATURE_UNITS) return;
    uac2_feature_unit_t *fu = &info->feature_units[info->num_feature_units++];
    fu->unit_id = desc[3];
    fu->source_id = desc[4];
    // bmaControls are 4 bytes each, starting at offset 5
    // Number of channels = (bLength - 6) / 4 - 1 (subtract master)
    uint8_t bLength = desc[0];
    int controls_bytes = bLength - 6; // 5 bytes header + 1 byte iFeature
    fu->nr_channels = (controls_bytes / 4) - 1; // subtract master control
    if (fu->nr_channels > 32) fu->nr_channels = 0; // sanity check
}

static void parse_ac_entity(const uint8_t *desc, uac2_device_info_t *info)
{
    uint8_t subtype = desc[2];

    switch (subtype) {
    case UAC2_AC_HEADER:
        parse_ac_header(desc, info);
        break;
    case UAC2_AC_CLOCK_SOURCE:
        parse_clock_source(desc, info);
        break;
    case UAC2_AC_CLOCK_SELECTOR:
        parse_clock_selector(desc, info);
        break;
    case UAC2_AC_INPUT_TERMINAL:
        parse_input_terminal(desc, info);
        break;
    case UAC2_AC_OUTPUT_TERMINAL:
        parse_output_terminal(desc, info);
        break;
    case UAC2_AC_FEATURE_UNIT:
        parse_feature_unit(desc, info);
        break;
    default:
        ESP_LOGD(TAG, "Skipping AC subtype 0x%02X", subtype);
        break;
    }
}

// ── AS descriptor parsing ───────────────────────────────────────────

static void parse_as_general(const uint8_t *desc, uac2_as_iface_t *as)
{
    as->terminal_link = desc[3];
    // bmControls at desc[4]
    as->format_type = desc[5];
    // bmFormats at desc[6..9]
    as->nr_channels = desc[10];
    // bmChannelConfig at desc[11..14]
}

static void parse_format_type_i(const uint8_t *desc, uac2_as_iface_t *as)
{
    as->sub_slot_size = desc[4];
    as->bit_resolution = desc[5];
}

// ── Main parser ─────────────────────────────────────────────────────

bool uac2_parse_config_descriptor(const uint8_t *config_desc, uint16_t total_length,
                                  uac2_device_info_t *info)
{
    memset(info, 0, sizeof(*info));

    // State tracking as we walk descriptors
    uint8_t current_iface_class = 0;
    uint8_t current_iface_subclass = 0;
    uint8_t current_iface_num = 0;
    uint8_t current_alt_setting = 0;
    uint8_t current_num_endpoints = 0;

    // Current AS interface being built (when in an AudioStreaming alternate)
    uac2_as_iface_t *current_as = NULL;
    int ep_count_in_current_as = 0;

    int offset = 0;
    while (offset < total_length) {
        uint8_t len = config_desc[offset];
        if (len == 0) break;
        if (offset + len > total_length) break;

        uint8_t type = config_desc[offset + 1];
        const uint8_t *desc = &config_desc[offset];

        if (type == 0x04) { // INTERFACE descriptor
            current_iface_num = desc[2];
            current_alt_setting = desc[3];
            current_num_endpoints = desc[4];
            current_iface_class = desc[5];
            current_iface_subclass = desc[6];

            // Start tracking a new AS interface if it has endpoints
            current_as = NULL;
            ep_count_in_current_as = 0;

            if (current_iface_class == UAC2_CLASS_AUDIO &&
                current_iface_subclass == UAC2_SUBCLASS_AUDIOSTREAMING &&
                current_num_endpoints > 0) {
                // This is an active AS alt setting
                if (info->num_as_ifaces < UAC2_MAX_AS_INTERFACES) {
                    current_as = &info->as_ifaces[info->num_as_ifaces++];
                    memset(current_as, 0, sizeof(*current_as));
                    current_as->interface_num = current_iface_num;
                    current_as->alt_setting = current_alt_setting;
                }
            }
        } else if (type == UAC2_CS_INTERFACE) { // Class-specific INTERFACE
            if (current_iface_class == UAC2_CLASS_AUDIO) {
                if (current_iface_subclass == UAC2_SUBCLASS_AUDIOCONTROL) {
                    parse_ac_entity(desc, info);
                } else if (current_iface_subclass == UAC2_SUBCLASS_AUDIOSTREAMING && current_as) {
                    uint8_t subtype = desc[2];
                    if (subtype == UAC2_AS_GENERAL) {
                        parse_as_general(desc, current_as);
                    } else if (subtype == UAC2_AS_FORMAT_TYPE) {
                        if (desc[3] == UAC2_FORMAT_TYPE_I) {
                            parse_format_type_i(desc, current_as);
                        }
                    }
                }
            }
        } else if (type == 0x05) { // ENDPOINT descriptor
            if (current_as && len >= 7) {
                uint8_t ep_addr = desc[2];
                uint8_t ep_attr = desc[3];
                uint16_t max_pkt = read_u16(&desc[4]);
                uint8_t interval = desc[6];

                uint8_t ep_type = ep_attr & 0x03;
                uint8_t ep_usage = (ep_attr >> 4) & 0x03;

                if (ep_type == 0x01) { // Isochronous
                    if (ep_usage == 0x01) {
                        // Feedback endpoint
                        current_as->fb_ep_addr = ep_addr;
                        current_as->fb_ep_max_packet_size = max_pkt;
                        current_as->fb_ep_interval = interval;
                    } else {
                        // Data endpoint
                        current_as->ep_addr = ep_addr;
                        current_as->ep_attributes = ep_attr;
                        current_as->ep_max_packet_size = max_pkt;
                        current_as->ep_interval = interval;
                    }
                }
                ep_count_in_current_as++;
            }
        } else if (type == UAC2_CS_ENDPOINT) { // Class-specific ENDPOINT
            // CS endpoint general — nothing critical to extract for now
        }

        offset += len;
    }

    return info->is_uac2;
}

// ── Logging ─────────────────────────────────────────────────────────

void uac2_log_device_info(const uac2_device_info_t *info)
{
    ESP_LOGI(TAG, "=== UAC2 Device Info ===");
    ESP_LOGI(TAG, "UAC version: %d.%02d  Category: 0x%02X (%s)",
             info->bcdADC >> 8, info->bcdADC & 0xFF,
             info->category,
             info->category == UAC2_CATEGORY_IO_BOX ? "I/O Box" : "Other");

    // Clock sources
    ESP_LOGI(TAG, "--- Clock Topology ---");
    for (int i = 0; i < info->num_clock_sources; i++) {
        const uac2_clock_source_t *cs = &info->clock_sources[i];
        ESP_LOGI(TAG, "  Clock Source ID=%d: %s, controls=0x%02X",
                 cs->clock_id, clock_type_str(cs->attributes), cs->controls);
    }
    for (int i = 0; i < info->num_clock_selectors; i++) {
        const uac2_clock_selector_t *cx = &info->clock_selectors[i];
        ESP_LOGI(TAG, "  Clock Selector ID=%d: %d input(s)", cx->clock_id, cx->nr_pins);
        for (int j = 0; j < cx->nr_pins; j++) {
            ESP_LOGI(TAG, "    Input %d: Clock ID=%d", j, cx->source_ids[j]);
        }
    }

    // Terminals
    ESP_LOGI(TAG, "--- Terminals ---");
    for (int i = 0; i < info->num_terminals; i++) {
        const uac2_terminal_t *t = &info->terminals[i];
        if (t->is_input) {
            ESP_LOGI(TAG, "  Input Terminal ID=%d: %s (0x%04X), %dch, clock=%d",
                     t->terminal_id, uac2_terminal_type_str(t->terminal_type),
                     t->terminal_type, t->nr_channels, t->clock_source_id);
        } else {
            ESP_LOGI(TAG, "  Output Terminal ID=%d: %s (0x%04X), src=%d, clock=%d",
                     t->terminal_id, uac2_terminal_type_str(t->terminal_type),
                     t->terminal_type, t->source_id, t->clock_source_id);
        }
    }

    // Feature units
    ESP_LOGI(TAG, "--- Feature Units ---");
    for (int i = 0; i < info->num_feature_units; i++) {
        const uac2_feature_unit_t *fu = &info->feature_units[i];
        ESP_LOGI(TAG, "  Feature Unit ID=%d: src=%d, %dch",
                 fu->unit_id, fu->source_id, fu->nr_channels);
    }

    // Audio streaming interfaces
    ESP_LOGI(TAG, "--- Audio Streaming ---");
    for (int i = 0; i < info->num_as_ifaces; i++) {
        const uac2_as_iface_t *as = &info->as_ifaces[i];
        const char *dir = (as->ep_addr & 0x80) ? "IN (capture)" : "OUT (playback)";
        ESP_LOGI(TAG, "  Interface %d Alt %d: %s",
                 as->interface_num, as->alt_setting, dir);
        ESP_LOGI(TAG, "    Terminal link=%d, %dch, %d-bit/%d-byte",
                 as->terminal_link, as->nr_channels,
                 as->bit_resolution, as->sub_slot_size);
        ESP_LOGI(TAG, "    EP 0x%02X: %s, MaxPkt=%d, Interval=%d",
                 as->ep_addr,
                 ((as->ep_attributes >> 2) & 0x03) == 0x01 ? "Async" :
                 ((as->ep_attributes >> 2) & 0x03) == 0x02 ? "Adaptive" :
                 ((as->ep_attributes >> 2) & 0x03) == 0x03 ? "Sync" : "None",
                 as->ep_max_packet_size, as->ep_interval);
        if (as->fb_ep_addr) {
            ESP_LOGI(TAG, "    Feedback EP 0x%02X: MaxPkt=%d, Interval=%d",
                     as->fb_ep_addr, as->fb_ep_max_packet_size, as->fb_ep_interval);
        }
    }

    // Signal chain summary
    ESP_LOGI(TAG, "--- Signal Chains ---");
    for (int i = 0; i < info->num_terminals; i++) {
        const uac2_terminal_t *t = &info->terminals[i];
        if (!t->is_input) continue;
        if (t->terminal_type == UAC2_TERMINAL_USB_STREAMING) {
            // Playback: USB -> FU -> Speaker
            ESP_LOGI(TAG, "  Playback: IT%d(USB) ->", t->terminal_id);
            for (int j = 0; j < info->num_feature_units; j++) {
                if (info->feature_units[j].source_id == t->terminal_id) {
                    ESP_LOGI(TAG, "    FU%d ->", info->feature_units[j].unit_id);
                    for (int k = 0; k < info->num_terminals; k++) {
                        if (!info->terminals[k].is_input &&
                            info->terminals[k].source_id == info->feature_units[j].unit_id) {
                            ESP_LOGI(TAG, "    OT%d(%s)",
                                     info->terminals[k].terminal_id,
                                     uac2_terminal_type_str(info->terminals[k].terminal_type));
                        }
                    }
                }
            }
        } else {
            // Capture: Mic -> FU -> USB
            ESP_LOGI(TAG, "  Capture: IT%d(%s) ->",
                     t->terminal_id, uac2_terminal_type_str(t->terminal_type));
            for (int j = 0; j < info->num_feature_units; j++) {
                if (info->feature_units[j].source_id == t->terminal_id) {
                    ESP_LOGI(TAG, "    FU%d ->", info->feature_units[j].unit_id);
                    for (int k = 0; k < info->num_terminals; k++) {
                        if (!info->terminals[k].is_input &&
                            info->terminals[k].source_id == info->feature_units[j].unit_id) {
                            ESP_LOGI(TAG, "    OT%d(%s)",
                                     info->terminals[k].terminal_id,
                                     uac2_terminal_type_str(info->terminals[k].terminal_type));
                        }
                    }
                }
            }
        }
    }
}
