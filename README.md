# esp-uac2-host

USB Audio Class 2.0 host driver for ESP32-S3. Enables ESP32-S3 to send and receive audio to/from UAC2 USB audio devices (DACs, audio interfaces, miniDSP, XMOS-based devices).

**This does not exist anywhere else.** Espressif's official `usb_host_uac` driver only supports UAC1. TinyUSB has no audio host class. CherryUSB's open-source host audio is UAC1-only. This is the first UAC2 host implementation for ESP32.

## Status

Not started. Implementation plan in `PROJECT.md`, research in `docs/ref-research.md`.

## Why

Any modern USB audio device (DAC, ADC, audio interface) uses UAC2. The ESP32-S3 has USB OTG with host mode and supports isochronous transfers — the hardware is capable. Only the driver is missing.

Many UAC2 devices do NOT fall back to UAC1 at Full Speed — they present UAC2 descriptors regardless of bus speed. Espressif's `usb_host_uac` can't talk to them. This driver fills that gap.

## Target

ESP32-S3 (Full Speed USB OTG, 12 Mbps).

## Approach

Fork the architecture of Espressif's `usb_host_uac` v1.3.3 (UAC1). The isochronous transfer plumbing, ring buffers, device enumeration, and streaming API are already proven. The delta for UAC2:

- UAC2 descriptor parsing (clock source/selector/multiplier entities)
- UAC2 control requests (CUR/RANGE vs UAC1's SET_CUR/GET_CUR)
- `bInterfaceProtocol` 0x20 detection
- UAC2 Format Type descriptors
- Clock source management (explicit in UAC2, implicit in UAC1)

Descriptor struct references (MIT/Apache-2.0):
- Eclipse ThreadX USBX `ux_class_audio20.h` — complete UAC2 descriptor structs and control selectors
- CherryUSB `usb_audio.h` — UAC2 descriptor structs alongside UAC1, well-organized for dual-version support

## License

MIT
