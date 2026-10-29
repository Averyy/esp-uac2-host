#!/usr/bin/env bash
# Build and run the host-side tests: descriptor parser (test_uac2_desc.c)
# and playback packet sizing (test_tx_sizing.c).
#
# Needs a C compiler and the ESP-IDF USB Host Library sources: by default
# $IDF_PATH/components/usb (ESP-IDF 5.x); set USB_COMPONENT_DIR to use another
# copy, such as the espressif/usb managed component.
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/../.." && pwd)"
usb="${USB_COMPONENT_DIR:-${IDF_PATH:?set IDF_PATH or USB_COMPONENT_DIR}/components/usb}"
helpers="$usb/usb_helpers.c"
[ -f "$helpers" ] || helpers="$usb/src/usb_helpers.c"
cc="${CC:-cc}"

out="$(mktemp -d)"
trap 'rm -rf "$out"' EXIT

# The parser cross-checks against ESP-IDF's lookup helpers; compile those
# functions verbatim (the rest of usb_helpers.c needs FreeRTOS).
{
    echo '#include <stddef.h>'
    echo '#include <assert.h>'
    echo '#include "usb/usb_helpers.h"'
    sed -n \
        -e '/^const usb_standard_desc_t \*usb_parse_next_descriptor(/,/^}/p' \
        -e '/^const usb_standard_desc_t \*usb_parse_next_descriptor_of_type(/,/^}/p' \
        -e '/^const usb_intf_desc_t \*usb_parse_interface_descriptor(/,/^}/p' \
        -e '/^const usb_ep_desc_t \*usb_parse_endpoint_descriptor_by_index(/,/^}/p' \
        "$helpers"
} > "$out/usb_helpers_subset.c"
found=$(grep -c '^const ' "$out/usb_helpers_subset.c" || true)
if [ "$found" -ne 4 ]; then
    echo "Expected 4 helper functions from $helpers, found $found" >&2
    exit 1
fi

"$cc" -std=gnu11 -g -O1 -Wall -Wextra -Werror -Wno-unused-parameter \
    -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all \
    -include stddef.h \
    -I "$here/stub" \
    -I "$root/components/usb_host_uac2/include" \
    -I "$root/ref" \
    -I "$usb/include" \
    "$here/test_uac2_desc.c" \
    "$root/components/usb_host_uac2/uac2_desc.c" \
    "$out/usb_helpers_subset.c" \
    -o "$out/test_uac2_desc"

"$out/test_uac2_desc"

# Playback packet sizing (header-only, no ESP-IDF sources needed)
"$cc" -std=gnu11 -g -O1 -Wall -Wextra -Werror \
    -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all \
    -I "$root/components/usb_host_uac2" \
    "$here/test_tx_sizing.c" \
    -o "$out/test_tx_sizing"

"$out/test_tx_sizing"
