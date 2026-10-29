/*
 * SPDX-License-Identifier: MIT
 *
 * Host-side tests for uac2_parse_config_descriptor(), run against the
 * captured miniDSP 2x4 HD descriptor and malformed variants of it. The
 * parser cross-checks alt settings with ESP-IDF's own descriptor lookup
 * helpers, which run.sh compiles in from an ESP-IDF checkout.
 *
 * Every descriptor is copied into an exact-size heap buffer so the address
 * sanitizer catches any read past wTotalLength.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "usb/uac2_desc.h"
#include "minidsp_2x4hd_descriptors.h"

static int s_failures;

#define CHECK(cond) do { \
    if (!(cond)) { \
        printf("    FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        s_failures++; \
    } \
} while (0)

#define REAL      minidsp_2x4hd_config1_desc
#define REAL_LEN  ((uint16_t)sizeof(minidsp_2x4hd_config1_desc))

static bool parse(const char *name, const uint8_t *src, uint16_t len, uac2_device_info_t *info)
{
    printf("[%s]\n", name);
    uint8_t *buf = malloc(len);
    memcpy(buf, src, len);
    bool ok = uac2_parse_config_descriptor(buf, len, info);
    free(buf);
    return ok;
}

static const uac2_as_iface_t *find_alt(const uac2_device_info_t *info, int iface, int alt)
{
    for (int i = 0; i < info->num_as_ifaces; i++) {
        if (info->as_ifaces[i].interface_num == iface && info->as_ifaces[i].alt_setting == alt) {
            return &info->as_ifaces[i];
        }
    }
    return NULL;
}

/** Offset of the interface descriptor for iface/alt in a well-formed config. */
static int intf_offset(const uint8_t *d, int len, int iface, int alt)
{
    for (int off = 0; off < len; off += d[off]) {
        if (d[off + 1] == 0x04 && d[off + 2] == iface && d[off + 3] == alt) {
            return off;
        }
    }
    return -1;
}

/** Offset of the idx-th endpoint descriptor after the descriptor at off. */
static int ep_offset(const uint8_t *d, int len, int off, int idx)
{
    for (off += d[off]; off < len; off += d[off]) {
        if (d[off + 1] == 0x05 && idx-- == 0) {
            return off;
        }
    }
    return -1;
}

static void set_total_length(uint8_t *d, uint16_t len)
{
    d[2] = len & 0xFF;
    d[3] = len >> 8;
}

static void test_real_device(void)
{
    uac2_device_info_t info;
    CHECK(parse("captured miniDSP 2x4 HD", REAL, REAL_LEN, &info));
    CHECK(info.num_as_ifaces == 3);

    const uac2_as_iface_t *pb24 = find_alt(&info, 1, 1);
    CHECK(pb24 && pb24->ep_addr == 0x01 && pb24->ep_max_packet_size == 294 &&
          pb24->ep_interval == 1 && pb24->bit_resolution == 24 && pb24->nr_channels == 2 &&
          pb24->fb_ep_addr == 0x81 && pb24->fb_ep_max_packet_size == 4 && pb24->fb_ep_interval == 4);

    const uac2_as_iface_t *pb16 = find_alt(&info, 1, 2);
    CHECK(pb16 && pb16->ep_addr == 0x01 && pb16->ep_max_packet_size == 196 &&
          pb16->bit_resolution == 16 && pb16->fb_ep_addr == 0x81);

    const uac2_as_iface_t *cap = find_alt(&info, 2, 1);
    CHECK(cap && cap->ep_addr == 0x82 && cap->ep_max_packet_size == 294 && cap->fb_ep_addr == 0);
}

static void test_zero_length_descriptor(void)
{
    // A bLength of 0 makes ESP-IDF's descriptor walk loop forever
    uint8_t d[REAL_LEN];
    memcpy(d, REAL, REAL_LEN);
    d[ep_offset(d, REAL_LEN, intf_offset(d, REAL_LEN, 1, 1), 1)] = 0;
    uac2_device_info_t info;
    CHECK(!parse("zero-length descriptor rejects the configuration", d, REAL_LEN, &info));
    CHECK(info.num_as_ifaces == 0);
}

static void test_short_standard_descriptors(void)
{
    uac2_device_info_t info;

    // Trailing 2-byte ENDPOINT descriptor claimed by an alt declaring one more endpoint
    uint8_t d[REAL_LEN + 2];
    memcpy(d, REAL, REAL_LEN);
    d[intf_offset(d, REAL_LEN, 2, 1) + 4] += 1;
    d[REAL_LEN] = 2;
    d[REAL_LEN + 1] = 0x05;
    set_total_length(d, sizeof(d));
    CHECK(!parse("2-byte endpoint descriptor rejects the configuration", d, sizeof(d), &info));

    // 8-byte INTERFACE descriptor (ESP-IDF reads 9 bytes regardless)
    uint8_t e[REAL_LEN - 1];
    int off = intf_offset(REAL, REAL_LEN, 2, 0);
    memcpy(e, REAL, off + 8);
    memcpy(e + off + 8, REAL + off + 9, REAL_LEN - off - 9);
    e[off] = 8;
    set_total_length(e, sizeof(e));
    CHECK(!parse("8-byte interface descriptor rejects the configuration", e, sizeof(e), &info));
}

static void test_overrunning_descriptor(void)
{
    // The last descriptor runs past wTotalLength
    uac2_device_info_t info;
    CHECK(!parse("descriptor overrunning wTotalLength rejects the configuration",
                 REAL, REAL_LEN - 3, &info));
}

static void test_undeclared_endpoint_ignored(void)
{
    // Alt declares 1 endpoint; the feedback EP after it is not part of the alt
    uint8_t d[REAL_LEN];
    memcpy(d, REAL, REAL_LEN);
    d[intf_offset(d, REAL_LEN, 1, 1) + 4] = 1;
    uac2_device_info_t info;
    CHECK(parse("endpoint beyond bNumEndpoints is ignored", d, REAL_LEN, &info));
    const uac2_as_iface_t *pb24 = find_alt(&info, 1, 1);
    CHECK(pb24 && pb24->ep_addr == 0x01 && pb24->fb_ep_addr == 0);
}

static void test_extra_endpoint_cannot_override_claimed_one(void)
{
    // One declared endpoint (bInterval 7), then an undeclared copy with
    // bInterval 1. ESP-IDF claims the first; the parser must report that one
    // so the stream-start interval check rejects it.
    uint8_t d[REAL_LEN];
    memcpy(d, REAL, REAL_LEN);
    int alt = intf_offset(d, REAL_LEN, 1, 1);
    int ep0 = ep_offset(d, REAL_LEN, alt, 0);
    int ep1 = ep_offset(d, REAL_LEN, alt, 1);
    d[alt + 4] = 1;
    d[ep0 + 6] = 7;
    memcpy(&d[ep1 + 2], &d[ep0 + 2], 4);
    d[ep1 + 6] = 1;
    uac2_device_info_t info;
    CHECK(parse("undeclared endpoint cannot replace the claimed one", d, REAL_LEN, &info));
    const uac2_as_iface_t *pb24 = find_alt(&info, 1, 1);
    CHECK(pb24 && pb24->ep_interval == 7);
}

static void test_duplicate_alt(void)
{
    // Two "iface 1 alt 1" descriptors: ESP-IDF claims the first
    uint8_t d[REAL_LEN];
    memcpy(d, REAL, REAL_LEN);
    d[intf_offset(d, REAL_LEN, 1, 2) + 3] = 1;
    uac2_device_info_t info;
    CHECK(parse("duplicate alt keeps only the copy ESP-IDF claims", d, REAL_LEN, &info));
    CHECK(info.num_as_ifaces == 2);
    const uac2_as_iface_t *pb = find_alt(&info, 1, 1);
    CHECK(pb && pb->bit_resolution == 24 && pb->ep_max_packet_size == 294);
}

static void test_duplicate_alt_first_incomplete(void)
{
    // The first copy declares more endpoints than it has (dropped by the walk),
    // the second is complete. ESP-IDF would still claim the first, so neither
    // may be exposed.
    uint8_t d[REAL_LEN];
    memcpy(d, REAL, REAL_LEN);
    d[intf_offset(d, REAL_LEN, 1, 2) + 3] = 1;
    d[intf_offset(d, REAL_LEN, 1, 1) + 4] = 3;
    uac2_device_info_t info;
    CHECK(parse("duplicate alt whose claimed copy is incomplete is dropped", d, REAL_LEN, &info));
    CHECK(find_alt(&info, 1, 1) == NULL);
    CHECK(find_alt(&info, 2, 1) != NULL);
}

static void test_many_alt_settings(void)
{
    // Copies of the 16-bit playback alt, renumbered 3, 4, ..., inserted right
    // after it (ESP-IDF only finds alts that follow their interface's other
    // alts): with the captured 3 alts, 5 copies fill every slot; with 6, the
    // 9th entry in descriptor order (the capture alt) is skipped
    int blk = intf_offset(REAL, REAL_LEN, 1, 2);
    int blk_end = intf_offset(REAL, REAL_LEN, 2, 0);
    int blk_len = blk_end - blk;
    for (int copies = 5; copies <= 6; copies++) {
        uint16_t len = (uint16_t)(REAL_LEN + copies * blk_len);
        uint8_t *d = malloc(len);
        memcpy(d, REAL, blk_end);
        for (int c = 0; c < copies; c++) {
            uint8_t *p = d + blk_end + c * blk_len;
            memcpy(p, REAL + blk, blk_len);
            p[3] = (uint8_t)(3 + c);
        }
        memcpy(d + blk_end + copies * blk_len, REAL + blk_end, REAL_LEN - blk_end);
        set_total_length(d, len);
        uac2_device_info_t info;
        CHECK(parse(copies == 5 ? "8 alt settings are all kept" : "a 9th alt setting is skipped",
                    d, len, &info));
        bool full = copies == 6;
        CHECK(UAC2_MAX_AS_INTERFACES == 8);
        CHECK(info.num_as_ifaces == UAC2_MAX_AS_INTERFACES);
        if (info.num_as_ifaces != UAC2_MAX_AS_INTERFACES) {
            for (int i = 0; i < info.num_as_ifaces; i++) {
                printf("    kept iface %d alt %d\n", info.as_ifaces[i].interface_num,
                       info.as_ifaces[i].alt_setting);
            }
        }
        const uac2_as_iface_t *last = find_alt(&info, 1, 7);
        CHECK(last && last->bit_resolution == 16 && last->ep_addr == 0x01);
        CHECK((find_alt(&info, 1, 8) != NULL) == full);
        CHECK((find_alt(&info, 2, 1) != NULL) == !full);
        free(d);
    }
}

int main(void)
{
    test_real_device();
    test_zero_length_descriptor();
    test_short_standard_descriptors();
    test_overrunning_descriptor();
    test_undeclared_endpoint_ignored();
    test_extra_endpoint_cannot_override_claimed_one();
    test_duplicate_alt();
    test_duplicate_alt_first_incomplete();
    test_many_alt_settings();

    if (s_failures) {
        printf("%d check(s) FAILED\n", s_failures);
        return 1;
    }
    printf("All descriptor parser checks passed\n");
    return 0;
}
