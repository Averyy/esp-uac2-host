# esp-uac2-host

USB Audio Class 2.0 host driver for ESP32-S3. ESP-IDF component (C).

Targets the miniDSP 2x4 HD but should work with any UAC2 device (XMOS-based DACs, audio interfaces, etc.).

## Status

Phase 3 complete. Full driver stack working end-to-end: descriptor parsing, control requests (CUR/RANGE), isochronous streaming with feedback-based adaptive packet sizing, volume/mute, disconnect handling. Tested against an ESP32-to-ESP32 UAC2 simulator: 48kHz and 44.1kHz streaming, stop/restart cycles, volume/mute during streaming, hot disconnect/reconnect — all verified. Next: test with real miniDSP 2x4 HD hardware.

## Features

- UAC2 descriptor parsing (clock sources, selectors, terminals, feature units, AS interfaces)
- Clock control: get/set sample rate, query supported ranges, check clock validity
- Isochronous TX (playback) and RX (capture) with ring buffer
- Feedback endpoint handling with adaptive packet sizing (accumulator pattern)
- Volume/mute control via feature unit
- First-frame timestamp (microsecond precision, for measurement applications)
- Atomic in-flight URB tracking for crash-free disconnect
- Spinlock-protected stream state transitions

## Project Structure

```
components/uac2_host/       # The driver (ESP-IDF component)
  include/uac2_host.h       #   Public API
  include/uac2_desc.h       #   Descriptor structs
  uac2_host.c               #   Driver implementation
  uac2_desc.c               #   Descriptor parser
main/                        # Test harness
  main.c                    #   Enumeration + streaming test suite
  tone_gen.c/h              #   Sine wave generator
simulators/
  simple/                   #   Minimal UAC2 simulator (TinyUSB, separate ESP32-S3)
  minidsp-2x4hd/            #   Full miniDSP 2x4 HD simulator (audio + HID)
ref/                         # Reference code (read-only, not compiled)
docs/                        # Design docs and research
```

## Hardware

- **MCU:** ESP32-S3-DevKitC-1
- **USB:** Full Speed (12 Mbps) — sufficient for 48kHz/24-bit/stereo (25% bus utilization)
- **Primary test device:** miniDSP 2x4 HD (XMOS XU216, UAC2, VID 0x2752)
- **Simulator:** Second ESP32-S3 running `simulators/minidsp-2x4hd/` firmware (TinyUSB, audio + HID)

## Building

Requires [ESP-IDF v5.4](https://docs.espressif.com/projects/esp-idf/en/v5.4/).

```sh
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/cu.usbmodem* flash monitor
```

## License

MIT
