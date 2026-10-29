/*
 * SPDX-License-Identifier: MIT
 *
 * libFuzzer target for uac2_parse_config_descriptor() and the clock /
 * feature-unit topology walks the driver runs on its result. Built by
 * fuzz.sh, which also compiles in ESP-IDF's descriptor lookup helpers and the
 * topology functions from uac2_host.c.
 *
 * Beyond "no crash, no out-of-bounds read" (address sanitizer, exact-size
 * input buffer), it checks the parser's contract independently: every alt
 * setting it keeps must be one usb_host_interface_claim() can claim from this
 * configuration without walking a malformed descriptor, with exactly the
 * endpoint parameters the parser reported.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "usb/uac2_desc.h"
#include "usb/usb_helpers.h"

// From uac2_host.c (extracted by fuzz.sh)
uint8_t resolve_clock_source(const uac2_device_info_t *info);
const uac2_feature_unit_t *resolve_feature_unit_for_iface(const uac2_device_info_t *info,
                                                          uint8_t iface_num);

#define FUZZ_ASSERT(cond) do { \
    if (!(cond)) { \
        fprintf(stderr, "invariant failed: %s (%s:%d)\n", #cond, __FILE__, __LINE__); \
        abort(); \
    } \
} while (0)

/** Every descriptor well-formed in the sense ESP-IDF's walk needs. */
static bool walk_is_safe(const uint8_t *d, uint16_t len)
{
    for (int off = 0; off < len; off += d[off]) {
        if (off + 1 >= len || d[off] < 2 || off + d[off] > len) {
            return false;
        }
        if ((d[off + 1] == 0x04 && d[off] < 9) || (d[off + 1] == 0x05 && d[off] < 7)) {
            return false;
        }
    }
    return true;
}

static void check_alt_claimable(const uint8_t *d, const uac2_as_iface_t *as)
{
    const usb_config_desc_t *cfg = (const usb_config_desc_t *)d;
    int offset = 0;
    const usb_intf_desc_t *intf = usb_parse_interface_descriptor(cfg, as->interface_num,
                                                                 as->alt_setting, &offset);
    FUZZ_ASSERT(intf != NULL);
    FUZZ_ASSERT(intf->bInterfaceNumber == as->interface_num);
    FUZZ_ASSERT(intf->bAlternateSetting == as->alt_setting);

    bool saw_data = as->ep_addr == 0;
    bool saw_fb = as->fb_ep_addr == 0;
    for (int e = 0; e < intf->bNumEndpoints; e++) {
        int ep_offset = offset;
        const usb_ep_desc_t *ep = usb_parse_endpoint_descriptor_by_index(intf, e, cfg->wTotalLength,
                                                                         &ep_offset);
        FUZZ_ASSERT(ep != NULL);
        FUZZ_ASSERT(ep_offset + ep->bLength <= cfg->wTotalLength);
        if (ep->bEndpointAddress == as->ep_addr && !saw_data) {
            FUZZ_ASSERT(ep->bmAttributes == as->ep_attributes);
            FUZZ_ASSERT(ep->wMaxPacketSize == as->ep_max_packet_size);
            FUZZ_ASSERT(ep->bInterval == as->ep_interval);
            saw_data = true;
        } else if (ep->bEndpointAddress == as->fb_ep_addr && !saw_fb) {
            FUZZ_ASSERT(ep->wMaxPacketSize == as->fb_ep_max_packet_size);
            FUZZ_ASSERT(ep->bInterval == as->fb_ep_interval);
            saw_fb = true;
        }
    }
    FUZZ_ASSERT(saw_data && saw_fb);
}

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    if (size < 9 || size > 1024) {
        return 0;
    }
    // Exact-size copy so any read past wTotalLength is caught
    uint8_t *d = malloc(size);
    memcpy(d, data, size);
    // Keep the configuration header plausible so inputs reach the walk
    d[0] = 9;
    d[1] = 0x02;
    d[2] = size & 0xFF;
    d[3] = (size >> 8) & 0xFF;

    uac2_device_info_t info;
    uac2_parse_config_descriptor(d, (uint16_t)size, &info);

    FUZZ_ASSERT(info.num_clock_sources <= UAC2_MAX_CLOCK_SOURCES);
    FUZZ_ASSERT(info.num_clock_selectors <= UAC2_MAX_CLOCK_SELECTORS);
    FUZZ_ASSERT(info.num_clock_multipliers <= UAC2_MAX_CLOCK_MULTIPLIERS);
    FUZZ_ASSERT(info.num_terminals <= UAC2_MAX_TERMINALS);
    FUZZ_ASSERT(info.num_feature_units <= UAC2_MAX_FEATURE_UNITS);
    FUZZ_ASSERT(info.num_as_ifaces <= UAC2_MAX_AS_INTERFACES);

    if (info.num_as_ifaces > 0) {
        // Kept alts imply a configuration ESP-IDF can walk without looping
        // or reading past a descriptor
        FUZZ_ASSERT(walk_is_safe(d, (uint16_t)size));
        for (int i = 0; i < info.num_as_ifaces; i++) {
            check_alt_claimable(d, &info.as_ifaces[i]);
        }
    }

    // What the driver does with a parsed device
    (void)resolve_clock_source(&info);
    for (int i = 0; i < info.num_as_ifaces; i++) {
        (void)resolve_feature_unit_for_iface(&info, info.as_ifaces[i].interface_num);
    }
    uac2_log_device_info(&info);

    free(d);
    return 0;
}
