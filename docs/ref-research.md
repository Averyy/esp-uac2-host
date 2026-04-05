# Research Reference

Pre-implementation research conducted April 2026. Findings here inform the project plan and confirm feasibility.

## miniDSP 2x4 HD: Confirmed UAC2-Only at Full Speed

**The miniDSP does NOT fall back to UAC1.** This is the critical finding that confirms this project is necessary.

A user on [USB_Host_Shield_2.0#594](https://github.com/felis/USB_Host_Shield_2.0/issues/594) connected a miniDSP 2x4 HD to an Arduino Pro Mini with a USB Host Shield (Full Speed only host). The descriptor dump shows:

- VID `0x2752` / PID `0x0011` -- confirmed
- `bDeviceClass: 0xEF` (IAD composite device)
- `bNumConfigurations: 2`
- `bInterfaceProtocol: 0x20` -- **UAC2, not UAC1**
- Audio Streaming endpoint `wMaxPacketSize: 0x0126` (294 bytes) at Alt Setting 1 -- consistent with 48kHz/24-bit/stereo
- Alt Setting 2 `wMaxPacketSize: 0x00C4` (196 bytes) -- consistent with 48kHz/16-bit/stereo
- `bMaxPacketSize0: 0x40` (64 bytes) -- standard Full Speed

The XMOS `AUDIO_CLASS_FALLBACK` (renamed `XUA_AUDIO_CLASS_FS` in lib_xua v5.1.0) is an opt-in compile-time flag. The XMOS default is `XUA_AUDIO_CLASS_FS=2` (UAC2 at both HS and FS). miniDSP has never documented or offered UAC1 support. A [miniDSP forum post](https://www.minidsp.com/community/threads/minidsp-hd-uac1-support.18916/) asking about UAC1 received no reply.

Additional evidence: AudioScienceReview users wanting to connect miniDSP 2x4 HD to a PS5 (UAC1-only) resort to external bridges, confirming no direct UAC1 support.

## No Open-Source UAC2 Host Driver Exists for ESP32

Exhaustive search (April 2026) across GitHub, Gitee, ESP forums, diyAudio, Hackaday, Reddit, and all major embedded USB stacks:

| What | Status | Detail |
|------|--------|--------|
| Espressif `usb_host_uac` | UAC1 only | Official driver. v1.3.3. Fork target for this project. |
| Espressif `usb_stream` | UAC1 only | Older combined UVC+UAC driver. |
| CherryUSB `usbh_audio` | UAC1 only (open-source) | See [CherryUSB section](#cherryusb-host-audio-investigation) below. |
| TinyUSB | No host audio at all | Device-side UAC1/UAC2 only. No `tuh_audio`. |
| NuttX on ESP32 | No audio host driver | HID/MSC/CDC only. |
| Zephyr on ESP32 | UAC2 device only | No host audio class. |
| USBX (Eclipse ThreadX) | Has UAC2 host | MIT structs reusable. Driver requires ThreadX RTOS. |
| PANAPROD (ESP32-P4) | Private, not published | Confirmed working via [esp-idf#18235](https://github.com/espressif/esp-idf/issues/18235). |
| pszsh (RP2040) | Likely device-side | See [pszsh section](#pszsh-rp2040-investigation) below. |

Espressif acknowledged the need on [esp-usb#173](https://github.com/espressif/esp-usb/issues/173) but has no timeline.

## UAC2 at Full Speed: Valid Per Spec

UAC2 does NOT require High Speed. XMOS explicitly supports UAC2 at Full Speed by default (`XUA_AUDIO_CLASS_FS=2`). The miniDSP descriptor dump proves this works in practice.

XMOS FS bandwidth limits (regardless of UAC version):
- Max 48 kHz if both input and output enabled
- Max 96 kHz if only one direction
- Max 2 channels per direction

Windows may reject UAC2 devices at Full Speed (Windows driver limitation, not spec). Irrelevant for this project since ESP32 is the host.

Source: [lib_xua docs](https://github.com/xmos/lib_xua/blob/develop/doc/rst/opt_audio_class.rst), [USB-IF whitepaper](https://www.usb.org/sites/default/files/bwpaper2.pdf)

## ESP32-S3 USB Bandwidth Analysis

| Metric | Value |
|--------|-------|
| Raw audio (48kHz/24-bit/stereo) | 288,000 bytes/sec = 2.3 Mbps |
| Per-frame (1ms) payload | 288 bytes |
| USB overhead per frame | ~10 bytes protocol |
| With bit stuffing (worst case) | ~348 bytes |
| Available per frame (after bus overhead) | ~1,308 bytes |
| **Bus utilization** | **~25%** |

96kHz/24-bit/stereo = 576 bytes/frame -- still fits. Bidirectional 48kHz/24-bit/stereo = ~597 bytes/frame (~50% utilization, feasible but tight).

## ESP32 Variant Comparison for USB Audio

| Feature | ESP32-S2 | ESP32-S3 | ESP32-P4 |
|---------|----------|----------|----------|
| USB speed | Full Speed (12 Mbps) | Full Speed (12 Mbps) | **High Speed (480 Mbps)** + FS |
| USB Host mode | Yes | Yes | Yes |
| Host channels | 8 | 8 | 16 |
| FIFO depth | 1024 bytes | 1024 bytes | Larger |
| CPU | 1x 240 MHz LX7 | 2x 240 MHz LX7 | 2x 400 MHz RISC-V |
| WiFi | Yes | Yes | **No** (external required) |

ESP32-S3 is the practical target (WiFi + adequate FS bandwidth). P4 is overkill and lacks wireless.

## ESP-IDF Isochronous Transfer: Proven Working

- `esp32-rtp` ([GitHub](https://github.com/netham45/esp32-rtp), 78 stars) -- continuous isochronous audio streaming on ESP32-S3 via `usb_host_uac`. 48kHz/16-bit/stereo, dual-core architecture.
- ESP-IDF USB Host Library supports all four transfer types including isochronous.
- DWC_OTG uses Scatter/Gather DMA with per-frame QTDs for isochronous.

**Known issues:**
- `HCCHAR.ChDis` does not work for isochronous channels during sudden disconnect. Full controller soft reset required.
- ESP32-P4 pre-v3 silicon has DMA corruption when USB isoc + Ethernet run simultaneously ([esp-idf#18235](https://github.com/espressif/esp-idf/issues/18235)). **Does not affect ESP32-S3.**
- 8 host channels total on S2/S3; each active endpoint consumes a channel.

## Memory Estimate

| Resource | Estimate |
|----------|----------|
| USB host + UAC driver | ~15-20 KB |
| Audio ring buffer (50ms @ 48kHz/24-bit/stereo) | ~14 KB |
| Isochronous transfer buffers (3 URBs x 3 packets) | ~4.5 KB |
| Task stacks | ~8 KB |
| **Total** | **~40-50 KB internal SRAM** |

ESP32-S3 has ~200-280 KB free after WiFi. USB DMA buffers must be in internal SRAM (not PSRAM).

## UAC2 Protocol: Key Differences from UAC1

| Feature | UAC1 | UAC2 |
|---------|------|------|
| `bInterfaceProtocol` | `0x00` | `0x20` |
| Clock management | Implicit (rate list in Format Type descriptor, SET_CUR on endpoint) | Explicit clock entities (source/selector/multiplier), rate via CUR/RANGE on clock source |
| Control requests | 7 codes: SET_CUR/GET_CUR/GET_MIN/GET_MAX/GET_RES + SET_MEM/GET_MEM | 2 codes: CUR/RANGE (direction via bmRequestType bit) |
| Sample rate location | Embedded in Format Type I descriptor (`tSamFreq[]`) | Queried from clock source via RANGE request |
| Sample rate setting | SET_CUR to endpoint (3 bytes) | CUR to clock source entity on AC interface (4 bytes) |
| AC Header | Lists streaming interface numbers (`baInterfaceNr[]`) | No interface list; has `bCategory` instead |
| Feature Unit controls | 1-2 byte bitmask per channel | 4-byte bitmask per channel |
| Format Type I | Includes inline rates | Only `bSubslotSize` + `bBitResolution` |

### UAC2 Clock Entities (new in UAC2)

- **Clock Source (subtype 0x0A)**: The oscillator. `bmAttributes` = internal/external, fixed/variable/programmable. `bmControls` for frequency and validity.
- **Clock Selector (subtype 0x0B)**: MUX between clock sources. `bNrInPins`, `baCSourceID[]`.
- **Clock Multiplier (subtype 0x0C)**: PLL/divider. `bCSourceID` upstream clock, numerator/denominator controls.

Terminals reference clocks explicitly: `bCSourceID` field in both Input Terminal and Output Terminal descriptors. Host must walk the clock topology to find the leaf clock source.

### UAC2 Control Request Layout

```
SET CUR (sample rate):
  bmRequestType = 0x21 (CLASS | INTERFACE | OUT)
  bRequest      = 0x01 (CUR)
  wValue        = (SAM_FREQ_CONTROL << 8) | 0x00 = 0x0100
  wIndex        = (clock_source_id << 8) | ac_interface_number
  wLength       = 4
  Data          = 32-bit LE frequency in Hz

GET RANGE (supported rates):
  bmRequestType = 0xA1 (CLASS | INTERFACE | IN)
  bRequest      = 0x02 (RANGE)
  wValue        = (SAM_FREQ_CONTROL << 8) | 0x00 = 0x0100
  wIndex        = (clock_source_id << 8) | ac_interface_number
  Response      = wNumSubRanges (2 bytes) + N * (dMIN, dMAX, dRES) (12 bytes each)
```

## Descriptor Struct References (MIT / Apache-2.0)

Two high-quality, permissively-licensed sources for UAC2 descriptor structs:

1. **USBX `ux_class_audio20.h`** (MIT) -- [GitHub](https://github.com/eclipse-threadx/usbx/blob/master/common/core/inc/ux_class_audio20.h). 1694 lines. Complete UAC2 descriptor structs, control selectors, format codes, RANGE response structs.

2. **CherryUSB `usb_audio.h`** (Apache-2.0) -- [GitHub](https://github.com/cherry-embedded/CherryUSB/blob/master/class/audio/usb_audio.h). Complete UAC2 descriptor structs alongside UAC1 structs. Includes clock source/selector/multiplier, RANGE parameter blocks, all control selectors.

Both can be used directly. CherryUSB's structs are arguably better organized for a driver that needs to support both UAC1 and UAC2.

## Code References (Ordered by Usefulness)

1. **Espressif `usb_host_uac` v1.3.3** -- Fork architecture. ESP-IDF native, proven isochronous streaming, ring buffers, state machine. ([GitHub](https://github.com/espressif/esp-usb/tree/master/host/class/uac/usb_host_uac))
2. **USBX `ux_class_audio20.h`** -- MIT descriptor structs. ([GitHub](https://github.com/eclipse-threadx/usbx/blob/master/common/core/inc/ux_class_audio20.h))
3. **CherryUSB `usb_audio.h` + `usbh_audio.c`** -- Apache-2.0 descriptor structs + UAC1 host driver pattern. ([GitHub](https://github.com/cherry-embedded/CherryUSB/tree/master/class/audio))
4. **Linux `sound/usb/clock.c`** -- Clock topology walking. GPL, read-only reference. ([GitHub](https://github.com/torvalds/linux/blob/master/sound/usb/clock.c))
5. **Linux `sound/usb/stream.c`** -- UAC2 descriptor parsing. GPL, read-only reference.
6. **esp32-rtp** -- Proves isochronous audio works on S3. ([GitHub](https://github.com/netham45/esp32-rtp))
7. **USB Audio Class 2.0 spec** -- [USB-IF](https://www.usb.org/sites/default/files/Audio2_with_Errata_and_ECN_through_Apr_2_2025.pdf)
8. **XMOS lib_xua docs** -- Clock/fallback behavior. ([GitHub](https://github.com/xmos/lib_xua/blob/develop/doc/rst/opt_audio_class.rst))

## pszsh RP2040 Investigation

User `pszsh` (Jess, jess@else-if.org) commented on [esp-usb#173](https://github.com/espressif/esp-usb/issues/173) claiming a "full and mostly working UAC2 implementation" on RP2xxx (Mar 13, 2026). As of April 2026, the repo has not been published.

**Assessment: Almost certainly a DEVICE driver, not HOST.**

Evidence:
- Their entire project history is building USB audio interfaces (RP2040/Teensy/STM32/Daisy act as USB audio devices to a host PC)
- "Audio recording at 48kHz 16-bit" = device-mode use case (MCU acts as microphone to computer)
- Every repo they forked/starred is device-side audio: `pschatzmann/Adafruit_TinyUSB_Arduino` (TinyUSB audio device wrapper), `alex6679/teensy-4-usbAudio` (Teensy UAC2 device), `electro-smith/libDaisy` (Daisy audio platform)
- TinyUSB has NO audio host class driver (`tuh_audio` does not exist). Only `tud_audio` (device).
- Their STM32 forum posts describe building a USB microphone (device-side)
- The "can't select the audio input device more than once" bug sounds like a device enumeration issue seen from the host OS side

Their `arduino-pico-master` repo (snapshot of earlephilhower/arduino-pico, Aug 2025) may contain their customizations, but has only a single commit tagged "yeeet" with no obvious audio additions.

**Not useful for this project** (we need host-side, not device-side). Worth replying on esp-usb#173 to ask them to clarify host vs device.

## CherryUSB Host Audio Investigation

CherryUSB's `usbh_audio.c` is **open-source (Apache-2.0) and UAC1-only**. No UAC2 host code exists anywhere in the repo, forks, vendor SDKs, or mirrors.

**What is commercially licensed** is not the class driver -- it's the **isochronous transfer layer** inside specific HCD port drivers (EHCI, DWC2, MUSB). The class-level protocol code is open.

Key findings:
- `usbh_audio.c` (~550 lines): Uses UAC1 request codes (SET_CUR/GET_CUR/etc.), 3-byte frequencies, UAC1 descriptor types. Registers with `bInterfaceProtocol=0x00`.
- `usb_audio.h`: Has complete UAC2 descriptor structs (used by device-side code), useful for our project.
- `usbd_audio.c`: Device-side handles both UAC1/UAC2. Shows CUR/RANGE and clock source handling -- useful as a reference for implementing the host-side equivalent.
- No host audio demos exist anywhere (demo/usb_host.c only tests CDC/HID/MSC).
- Bouffalo SDK, Beken SDK, all Gitee mirrors contain identical UAC1-only host audio code.

**Useful for this project:** The `usb_audio.h` descriptor structs (Apache-2.0), the `usbh_audio.c` driver pattern, and the `usbd_audio.c` UAC2 request handling (flip direction for host).

## Espressif UAC1 Host Driver Architecture

Source: [usb_host_uac v1.3.3](https://github.com/espressif/esp-usb/tree/master/host/class/uac/usb_host_uac)

**Core structures:**
- `uac_device_t` -- physical USB device (handle, address, control transfer, AC descriptor blob)
- `uac_iface_t` -- one logical audio interface (speaker OR mic, not both). Per-device with both = two instances.

**State machine:** NOT_INITIALIZED -> IDLE -> READY -> ACTIVE -> SUSPENDING

**Isochronous transfer management:**
- Default: 3 URBs (`UAC_NUM_ISOC_URBS`), 3 packets per URB (`UAC_NUM_PACKETS_PER_URB`)
- TX path: ring buffer -> fill transfer -> submit -> completion callback -> refill
- RX path: submit -> completion callback -> copy to ring buffer -> resubmit

**API:** `uac_host_install()`, `uac_host_device_open()`, `uac_host_device_start()`, `uac_host_device_write()`, `uac_host_device_read()`, `uac_host_device_set_volume()`, `uac_host_device_close()`.

This is the direct architectural predecessor. The isochronous plumbing, ring buffers, and state machine are reusable. The delta is descriptor parsing and control request format.
