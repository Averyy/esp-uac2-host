# Design

ESP-IDF component layered on the ESP-IDF USB Host Library: it discovers UAC2 streaming interfaces, manages the clock, and streams audio over isochronous transfers. API reference and internals: `llms.txt`. Pre-implementation research (April 2026: miniDSP descriptor dumps, bandwidth analysis, driver survey): `docs/ref-research.md`.

```
+---------------------------+
|     Application           |  (e.g. minidsp-open sweep playback)
+---------------------------+
|     esp-uac2-host         |  descriptor parsing, clock management, streaming API
+---------------------------+
|  ESP-IDF USB Host Lib     |  in-tree on IDF 5.x, espressif/usb on 6.x
+---------------------------+
|  DWC_OTG HCD              |  scatter/gather DMA
+---------------------------+
|  ESP32-S3 USB OTG HW      |  Full Speed 12 Mbps, 8 host channels, 1024 B FIFO
+---------------------------+
```

## Design goals

1. **Composite-device safe:** claims only the audio streaming interface, so it coexists with other class drivers (e.g. HID on the miniDSP).
2. **Both directions:** isochronous OUT (playback) and IN (capture).
3. **Whatever format the device offers:** rates, bit depths and channel counts come from the device; the caller picks.
4. **Blocking read/write** over a ring buffer, as in Espressif's UAC1 driver.
5. **Start timestamp:** `uac2_host_device_get_start_time()` returns the `esp_timer_get_time()` value at the first isochronous submit, for measurement sync.
6. **Opens by address, not VID/PID:** the driver reports `addr` + `iface_num` for every UAC2 streaming interface (including devices attached before install); the caller decides what to open.
7. **Lightweight:** isochronous buffers must be internal DMA memory; ring buffer storage comes from `malloc` and can live in PSRAM.

## UAC2 vs UAC1

| Feature | UAC1 | UAC2 |
|---------|------|------|
| `bInterfaceProtocol` | 0x00 | 0x20 (IP_VERSION_02_00) |
| Clock management | Implicit (SET_CUR sample rate on the endpoint, 3-byte LE) | Explicit clock source/selector/multiplier entities; CUR on the clock source (4-byte LE) |
| Control requests | SET_CUR/GET_CUR/GET_MIN/GET_MAX/GET_RES/SET_MEM/GET_MEM | CUR (0x01) / RANGE (0x02), direction from bmRequestType |
| Sample rates | In the Format Type I descriptor (`bSamFreqType`, `tSamFreq[]`) | Queried from the clock source with RANGE |
| Format Type I | `bSamFreqType` + rate list or range | `bSubslotSize` + `bBitResolution` |
| AC header | Lists streaming interfaces, `bcdADC=0x0100` | No interface list, `bCategory`, `bcdADC=0x0200` |
| Feature unit controls | 1-2 byte bitmask per channel | 4-byte bitmask per channel |
| Feedback | 3-byte 10.14 | 4-byte 16.16 at FS on XMOS (10.14 also seen) |

Some XMOS devices offer two configurations (UAC2 and UAC1); at Full Speed, firmware built with `XUA_AUDIO_CLASS_FS=1` presents UAC1. The miniDSP 2x4 HD presents UAC2 at Full Speed. This driver is UAC2-only; see `docs/TODO-uac1-coexistence.md`.

## Protocol notes

Clock: walk terminal `bCSourceID` -> optional selectors/multipliers -> clock source (as Linux `sound/usb/clock.c` `__uac_clock_find_source()` does), query RANGE on its SAM_FREQ_CONTROL, then SET CUR:

```
bmRequestType = 0x21 (CLASS | INTERFACE | OUT)   // SET CUR
bRequest      = 0x01 (CUR)
wValue        = SAM_FREQ_CONTROL << 8 = 0x0100
wIndex        = (clock_source_id << 8) | ac_interface_number
wLength       = 4
Data          = sample rate, uint32 LE
```

Start order: claim the interface, set the clock, then SET_INTERFACE(alt), then submit URBs (the order Linux snd-usb-audio uses for UAC2).

Full Speed isochronous: one packet per 1 ms frame. 48 kHz stereo 24-bit = 288 B/frame, 16-bit = 192 B/frame, 96 kHz 24-bit = 576 B/frame. The ESP32-S3 FIFO limits packet sizes further (see `llms.txt`, Hardware Constraints). Isochronous transactions are never retried; check every packet's status.

## Key references

- Espressif UAC1 driver (`esp-usb/host/class/uac/usb_host_uac`): class driver pattern, state machine, ring buffers, isochronous transfer management. Source in `ref/espressif-uac1/`.
- USBX `ux_class_audio20.h` (MIT) and CherryUSB `usb_audio.h` (Apache-2.0): UAC2 descriptor structs.
- Linux `sound/usb/clock.c`, `stream.c`, `endpoint.c`, `quirks-table.h` (GPL, read-only reference): clock walking, descriptor parsing, start sequence, device quirks.
- USB Audio Class 2.0 spec with errata (usb.org); XMOS USB Audio design guide and `lib_xua`.
- Arduino USB Host Shield issue #594: an early miniDSP 2x4 HD descriptor dump at Full Speed.
