# esp-uac2-host

USB Audio Class 2.0 host driver for ESP32-S3. ESP-IDF component (C).

Generic UAC2 driver — works with any UAC2 device (DACs, audio interfaces, miniDSP, etc.). Primary use case: playing measurement sweeps through a miniDSP 2x4 HD from an ESP32 for automated sub optimization.

## Status

**v1.0.0 — Driver complete, hardened, live-tested.** All 5 automated tests pass against miniDSP 2x4 HD simulator (55k+ frames, zero errors). Full cleanup pass (ESP-IDF conventions, Kconfig, idf_component.yml) + 5-agent code review with all 29 findings fixed. Next: test with real miniDSP 2x4 HD hardware, then integrate with minidsp-open.

See [PROGRESS.md](PROGRESS.md) for detailed history and [docs/TODO.md](docs/TODO.md) for remaining work.

## Features

- UAC2 descriptor parsing (clock sources, selectors, terminals, feature units, AS interfaces)
- Clock control: get/set sample rate, query supported ranges, check clock validity
- SET_INTERFACE for proper endpoint activation/deactivation
- Isochronous TX (playback) and RX (capture) with ring buffer
- Feedback endpoint handling (10.14 and 16.16 formats) with adaptive packet sizing
- Volume/mute control via feature unit
- First-frame timestamp (microsecond precision, for measurement sync)
- Transfer error limiting (auto-stops after 10 consecutive errors)
- Endpoint halt/flush/clear on stream stop
- Atomic in-flight URB tracking for crash-free disconnect
- Spinlock-protected stream state transitions

## Quick Start

```c
#include "usb/uac2_host.h"

// After USB enumeration...
uac2_host_device_handle_t dev;
uac2_host_device_open(client, usb_dev, event_cb, NULL, &dev);

// Start 48kHz/24-bit/stereo playback
uac2_stream_config_t cfg = { .sample_rate = 48000, .bit_resolution = 24, .channels = 2 };
uac2_host_stream_start(dev, UAC2_STREAM_TX, &cfg);

// Write PCM data (fills ring buffer, blocks if full)
uac2_host_stream_write(dev, pcm_data, num_bytes, timeout_ms);

// Get hardware timestamp of first audio frame
int64_t start_us = uac2_host_stream_get_start_time(dev);

// Stop
uac2_host_stream_stop(dev, UAC2_STREAM_TX);
uac2_host_device_close(dev);
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
main/                        # Test harness (5 automated tests)
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

The test suite runs automatically on boot against any connected UAC2 device:

1. **48kHz streaming** — 10 seconds, 1kHz tone
2. **Volume/mute control** — set/get during streaming
3. **Stop/restart cycle** — 5s stream → stop → 2s pause → 5s stream
4. **44.1kHz streaming** — 5 seconds, sample rate switch
5. **Long-running stability** — continuous until disconnect

## Using as a Component

Add to your ESP-IDF project:
```cmake
set(EXTRA_COMPONENT_DIRS "/path/to/esp-uac2-host/components/uac2_host")
```

## License

MIT
