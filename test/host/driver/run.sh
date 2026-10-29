#!/usr/bin/env bash
# Build the driver-on-Linux test (ESP-IDF Linux target + fake USB Host
# Library) and run it. Needs an activated ESP-IDF 5.x environment (IDF_PATH)
# and a host C compiler; address/UB sanitizers are used when it supports them.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
cd "$here"
# The driver's manifest only lists esp32s3; components come from the project
# and EXTRA_COMPONENT_DIRS, so the component manager is not needed here.
export IDF_COMPONENT_MANAGER=0

probe="$(mktemp -d)"
trap 'rm -rf "$probe"' EXIT
echo 'int main(void) { return 0; }' > "$probe/p.c"
sanitize=0
if "${CC:-cc}" -fsanitize=address,undefined "$probe/p.c" -o "$probe/p" 2>/dev/null; then
    sanitize=1
fi
echo "Sanitizers: $([ $sanitize = 1 ] && echo on || echo off)"

if [ ! -f sdkconfig ]; then
    idf.py -B bout --preview set-target linux
fi
idf.py -B bout -DUAC2_SANITIZE=$sanitize build
./bout/uac2_host_driver_test.elf
