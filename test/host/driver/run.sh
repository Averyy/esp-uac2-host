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

# sdkconfig.defaults only seeds a new sdkconfig, so start over when it or the
# driver's Kconfig changed since (a new option would otherwise stay unset)
if [ ! -f sdkconfig ] || [ sdkconfig.defaults -nt sdkconfig ] ||
   [ ../../../components/usb_host_uac2/Kconfig -nt sdkconfig ]; then
    rm -f sdkconfig
    idf.py -B bout --preview set-target linux
fi
idf.py -B bout -DUAC2_SANITIZE=$sanitize build

# The per-test watchdog is a FreeRTOS task, so a wedged scheduler silences it,
# and the POSIX port blocks signals other than SIGKILL. Bound the whole run
# from outside (a full run takes about 30 s).
./bout/uac2_host_driver_test.elf &
test_pid=$!
(
    # Polls rather than sleeping 900 s, so it ends within a second of the
    # test and leaves no child holding the output open
    i=0
    while [ "$i" -lt 900 ]; do
        kill -0 "$test_pid" 2>/dev/null || exit 0
        sleep 1
        i=$((i + 1))
    done
    kill -9 "$test_pid" 2>/dev/null && echo "HANG: test binary killed after 900 s"
) &
guard_pid=$!
status=0
wait "$test_pid" || status=$?
wait "$guard_pid" || true
exit "$status"
