# esp-uac2-host

USB Audio Class 2.0 host driver for ESP32-S3. ESP-IDF component (C).

Generic UAC2 driver — works with any UAC2 device (DACs, audio interfaces, miniDSP, etc.). Primary use case: playing measurement sweeps through a miniDSP 2x4 HD from an ESP32 for automated sub optimization.

## Status

**v0.1.0 — Complete, hardware-verified.** All 12 automated tests pass against both ESP32-to-ESP32 simulator and real miniDSP 2x4 HD (zero errors, 766-second stability run). Espressif class driver pattern with install/uninstall lifecycle, internal device discovery, reference counting. 4-agent code review with all findings fixed. Builds clean with `-Werror -Wextra`.

See [PROGRESS.md](PROGRESS.md) for detailed history.

## Features

- Espressif class driver pattern (`uac2_host_install`/`uac2_host_uninstall`) with internal device discovery
- UAC2 descriptor parsing (clock sources, selectors, multipliers, terminals, feature units, AS interfaces)
- Clock topology walk (terminal→selector/multiplier→clock source)
- Clock control: get/set sample rate, query supported ranges, check clock validity
- SET_INTERFACE for proper endpoint activation/deactivation
- Isochronous TX (playback) and RX (capture) with ring buffer
- Feedback endpoint handling (16.16 format, confirmed with real XMOS hardware) with adaptive packet sizing
- Volume/mute control via feature unit with bmaControls validation and range caching
- Suspend/resume without URB/ringbuf reallocation
- First-frame timestamp (microsecond precision, for measurement sync)
- Transfer error limiting (auto-stops after consecutive errors, fires STREAM_ERROR event)
- Endpoint halt/flush/clear on stream stop
- Atomic in-flight URB tracking for crash-free disconnect
- Per-stream spinlock-protected state transitions

## Quick Start

```c
#include "usb/uac2_host.h"

// Install the driver (creates internal USB Host client)
uac2_host_config_t config = {
    .create_background_task = true,
    .callback = device_event_cb,
    .callback_arg = NULL,
};
uac2_host_install(&config);

// In the callback, open a discovered interface:
uac2_host_device_handle_t dev;
uac2_host_device_open(&open_config, &dev);

// Start 48kHz/24-bit/stereo playback
uac2_host_device_start(dev, &stream_config);

// Write PCM data (fills ring buffer, blocks if full)
uac2_host_device_write(dev, pcm_data, num_bytes, timeout_ms);

// Stop and close
uac2_host_device_stop(dev);
uac2_host_device_close(dev);

// When done
uac2_host_uninstall();
```

## Project Structure

```
components/uac2_host/       # The driver (ESP-IDF component)
  include/usb/uac2_host.h   #   Public API
  include/usb/uac2_desc.h   #   Descriptor structs
  uac2_host.c               #   Driver implementation
  uac2_desc.c               #   Descriptor parser
  Kconfig                   #   Tunable parameters (menuconfig)
  idf_component.yml         #   Component registry manifest
main/                        # Test harness (12 automated tests)
  main.c                    #   Enumeration + streaming test suite
  tone_gen.c/h              #   Sine wave generator
simulators/
  simple/                   #   Minimal UAC2 simulator (TinyUSB)
  minidsp-2x4hd/            #   Full miniDSP 2x4 HD simulator (audio + HID + fault injection)
ref/                         # Reference code (read-only, not compiled)
docs/                        # Design docs and remaining work
```

## Hardware

- **MCU:** ESP32-S3-DevKitC-1
- **USB:** Full Speed (12 Mbps) — sufficient for 48kHz/24-bit/stereo (25% bus utilization)
- **Simulator:** Second ESP32-S3 running `simulators/minidsp-2x4hd/` firmware
- **Target device:** miniDSP 2x4 HD (XMOS XU216, UAC2, VID 0x2752 PID 0x0011)

## Building

Requires [ESP-IDF v5.4](https://docs.espressif.com/projects/esp-idf/en/v5.4/).

```sh
# Activate ESP-IDF
export PATH="/opt/homebrew/bin:$PATH" && . ~/esp/esp-idf/export.sh

# Build and flash the host
idf.py build
idf.py -p /dev/cu.usbmodem<SERIAL> flash

# Build and flash the simulator
cd simulators/minidsp-2x4hd
idf.py build
idf.py -p /dev/cu.usbmodem<SERIAL> flash
```

### Kconfig

In `sdkconfig.defaults`:
```
CONFIG_USB_HOST_CONTROL_TRANSFER_MAX_SIZE=512
CONFIG_USB_HOST_HW_BUFFER_BIAS_PERIODICOUT=y
```

## Tests

The test suite runs automatically on boot against any connected UAC2 device (12 tests):

1. **48kHz streaming** — 10 seconds, 1kHz tone
2. **Volume/mute control** — set/get during streaming
3. **Stop/restart cycle** — 5s stream → stop → 2s pause → 5s stream
4. **44.1kHz streaming** — 5 seconds, sample rate switch
5. **Start time precision** — microsecond timestamp verification
6. **Feedback convergence** — verify feedback locks to expected rate
7. **16-bit mode** — alternate format streaming
8. **Rapid measurement cycles** — 9x rapid start/stop
9. **Volume range exploration** — full range sweep
10. **Sample rate switch stress** — repeated 48kHz↔44.1kHz switching
11. **Ring buffer starvation/recovery** — underrun and recovery test
12. **Long-running stability** — continuous until disconnect (766s verified)

## Using as a Component

Add to your ESP-IDF project:
```cmake
set(EXTRA_COMPONENT_DIRS "/path/to/esp-uac2-host/components/uac2_host")
```

## License

MIT
