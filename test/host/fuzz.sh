#!/usr/bin/env bash
# Build and run the libFuzzer target for the descriptor parser
# (fuzz_uac2_desc.c). Extra arguments go to libFuzzer, e.g.:
#   test/host/fuzz.sh -max_total_time=600
#
# Needs clang with libFuzzer (Linux clang, or Homebrew llvm on macOS; Apple
# clang has no libFuzzer) and the ESP-IDF USB Host Library sources, found as
# in run.sh. Seeds come from the captured miniDSP descriptor; the corpus and
# any crash inputs are kept under $FUZZ_OUT (default test/host/fuzz-out).
#
# With -fork=N, LeakSanitizer can report a 32-byte leak in libFuzzer's own
# fork driver (fuzzer::FuzzWithFork) at exit and save an empty crash-da39a3ee...
# artifact; that is libFuzzer, not this code.
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/../.." && pwd)"
usb="${USB_COMPONENT_DIR:-${IDF_PATH:?set IDF_PATH or USB_COMPONENT_DIR}/components/usb}"
helpers="$usb/usb_helpers.c"
[ -f "$helpers" ] || helpers="$usb/src/usb_helpers.c"
cc="${CC:-clang}"
out="${FUZZ_OUT:-$here/fuzz-out}"
mkdir -p "$out/corpus" "$out/artifacts" "$out/src"

# ESP-IDF's descriptor lookup helpers, verbatim
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
} > "$out/src/usb_helpers_subset.c"

# The driver's topology walks over parsed descriptors, made non-static
{
    echo '#include <stdbool.h>'
    echo '#include <stddef.h>'
    echo '#include "esp_log.h"'
    echo '#include "usb/uac2_desc.h"'
    echo 'static const char *TAG = "uac2-host";'
    sed -n \
        -e '/^static uint8_t resolve_clock_source(/,/^}/p' \
        -e '/^static const uac2_terminal_t \*find_terminal_by_id(/,/^}/p' \
        -e '/^static const uac2_feature_unit_t \*find_feature_unit_by_id(/,/^}/p' \
        -e '/^static const uac2_feature_unit_t \*resolve_feature_unit_for_iface(/,/^}/p' \
        "$root/components/usb_host_uac2/uac2_host.c" |
        sed -e 's/^static uint8_t resolve_clock_source(/uint8_t resolve_clock_source(/' \
            -e 's/^static const uac2_feature_unit_t \*resolve_feature_unit_for_iface(/const uac2_feature_unit_t *resolve_feature_unit_for_iface(/'
} > "$out/src/uac2_topology_subset.c"
for f in usb_helpers_subset uac2_topology_subset; do
    if [ "$(grep -c '^[a-z].*(.*' "$out/src/$f.c")" -lt 2 ]; then
        echo "Extraction into $f.c failed" >&2
        exit 1
    fi
done

# Seed: the captured miniDSP configuration descriptor
python3 - "$root/ref/minidsp_2x4hd_descriptors.h" "$out/corpus/minidsp_2x4hd" <<'EOF'
import re, sys
src = open(sys.argv[1]).read()
body = src[src.index('minidsp_2x4hd_config1_desc'):]
body = body[body.index('{') + 1:body.index('};')]
body = re.sub(r'//.*', '', body)
body = re.sub(r'/\*.*?\*/', '', body, flags=re.S)
open(sys.argv[2], 'wb').write(bytes(int(x, 16) for x in re.findall(r'0x[0-9A-Fa-f]{2}', body)))
EOF

"$cc" -std=gnu11 -g -O1 -Wall -Wextra -Wno-unused-parameter -Wno-unused-function \
    -fsanitize=fuzzer,address,undefined -fno-sanitize-recover=all \
    -DUAC2_TEST_EVAL_LOGS -include stddef.h \
    -I "$here/stub" \
    -I "$root/components/usb_host_uac2/include" \
    -I "$usb/include" \
    "$here/fuzz_uac2_desc.c" \
    "$root/components/usb_host_uac2/uac2_desc.c" \
    "$out/src/usb_helpers_subset.c" \
    "$out/src/uac2_topology_subset.c" \
    -o "$out/fuzz_uac2_desc"

"$out/fuzz_uac2_desc" -artifact_prefix="$out/artifacts/" -timeout=5 -max_len=1024 "$@" "$out/corpus"
