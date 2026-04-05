# Design

## Research

See `docs/ref-research.md` for the full pre-implementation research (April 2026): miniDSP descriptor dumps, protocol details, bandwidth analysis, existing driver survey, and code references.

## Architecture

ESP-IDF component that layers on top of the ESP-IDF USB Host Library. Provides a clean API for opening UAC2 audio devices, configuring sample rate/format, and streaming audio in/out via isochronous transfers.

```
+---------------------------+
|     Application           |  (e.g., minidsp-open sweep playback)
+---------------------------+
|     esp-uac2-host         |  <- this project
|  (descriptor parsing,     |
|   clock management,       |
|   streaming API)          |
+---------------------------+
|  ESP-IDF USB Host Lib     |  (isochronous transfers, device enumeration)
+---------------------------+
|  DWC_OTG HCD              |  (hardware controller driver, Scatter/Gather DMA)
+---------------------------+
|  ESP32-S3 USB OTG HW      |  (Full Speed 12 Mbps, 8 host channels, 1024B FIFO)
+---------------------------+
```

## UAC2 vs UAC1: What's Different

| Feature | UAC1 | UAC2 |
|---------|------|------|
| `bInterfaceProtocol` | 0x00 | 0x20 (IP_VERSION_02_00) |
| Clock management | Implicit (SET_CUR sample rate on endpoint, 3-byte LE) | Explicit clock source/selector/multiplier entities, CUR on clock source entity (4-byte LE) |
| Control requests | 7 codes: SET_CUR/GET_CUR/GET_MIN/GET_MAX/GET_RES/SET_MEM/GET_MEM | 2 codes: CUR (0x01) / RANGE (0x02), direction via bmRequestType bit |
| Sample rate location | Inline in Format Type I descriptor (`bSamFreqType`, `tSamFreq[]`) | Not in descriptor; queried from clock source via RANGE request |
| Format Type I descriptor | `bSamFreqType` + rate list or continuous range | `bSubslotSize` + `bBitResolution` only |
| AC Header descriptor | Lists streaming interfaces (`bInCollection`, `baInterfaceNr[]`), `bcdADC=0x0100` | No interface list, has `bCategory`, `bcdADC=0x0200` |
| Feature Unit controls | 1-2 byte bitmask per channel (`bControlSize`-parameterized) | 4-byte bitmask per channel (fixed) |
| Feedback | Implicit (3-byte feedback endpoint) | Explicit (can use clock source for async mode) |
| Connector descriptors | No | Yes (optional) |

## Implementation Plan

### Phase 0: Hardware Verification (1 day)

Even though research confirms the miniDSP presents UAC2 at FS, set up the dev environment and verify firsthand:
1. ESP32-S3 dev board + USB OTG cable + miniDSP 2x4 HD
2. Enumerate device, dump all descriptors to serial
3. Confirm `bInterfaceProtocol=0x20`, capture exact clock topology
4. Log all alternate settings, endpoint addresses, max packet sizes
5. Also test with JDS Labs Atom DAC+ — should present UAC1 at FS (fallback)

### Phase 1: Descriptor Parsing + Enumeration

Parse UAC2 device descriptors to extract:
- Audio Control interface with clock source/selector entities
- Audio Streaming interfaces with format type descriptors
- Isochronous endpoints (IN for mic, OUT for speaker)
- Supported sample rates (via RANGE request on clock source)
- Supported bit depths and channel counts

Detect UAC2 via `bInterfaceProtocol == 0x20`. Fall back to UAC1 path if `0x00`.

Descriptor struct sources (both permissively licensed):
- USBX `ux_class_audio20.h` (MIT): all UAC2 entity descriptors, control selectors, RANGE response structs
- CherryUSB `usb_audio.h` (Apache-2.0): UAC2 structs alongside UAC1, init macros, control selectors

### Phase 2: Clock Configuration

UAC2 has explicit clock entities:
- **Clock Source (subtype 0x0A)**: generates a clock (internal oscillator, SOF-synced, external). `bmAttributes` = internal/external + fixed/variable/programmable. `bmControls` for frequency and validity.
- **Clock Selector (subtype 0x0B)**: selects between multiple clock sources. `bNrInPins`, `baCSourceID[]`.
- **Clock Multiplier (subtype 0x0C)**: multiplies a clock frequency. `bCSourceID` upstream clock.

To set sample rate:
1. Find the clock source entity: walk terminal `bCSourceID` -> (optional selector/multiplier) -> clock source. Reference: Linux `sound/usb/clock.c` `__uac_clock_find_source()`.
2. Query supported rates via GET RANGE on clock source's SAM_FREQ_CONTROL
3. Set sample rate via SET CUR on clock source's SAM_FREQ_CONTROL (4-byte LE frequency)

Control request layout:
```
bmRequestType = 0x21 (CLASS | INTERFACE | OUT)   // SET CUR
bRequest      = 0x01 (CUR)
wValue        = (SAM_FREQ_CONTROL << 8) = 0x0100
wIndex        = (clock_source_id << 8) | ac_interface_number
wLength       = 4
Data          = sample_rate as uint32_t LE
```

### Phase 3: Audio Streaming

Open the streaming interface:
1. Select the correct alternate setting (matching desired format)
2. Configure the isochronous OUT endpoint
3. Submit isochronous OUT transfers with PCM data

At Full Speed (ESP32-S3):
- 1ms frame period
- Max 1023 bytes per isochronous packet (spec), 512 bytes effective (DWC_OTG single-transaction-per-frame)
- 48kHz stereo 24-bit = 288 bytes/frame (fits easily, ~25% bus utilization)
- 48kHz stereo 16-bit = 192 bytes/frame
- 96kHz stereo 24-bit = 576 bytes/frame (fits, but approaches limits)

Ring buffer for smooth streaming (same pattern as Espressif's UAC1 driver):
- Default: 3 URBs, 3 packets per URB
- TX: ring buffer -> fill transfer -> submit -> completion callback -> refill
- RX: submit -> completion callback -> copy to ring buffer -> resubmit

DMA note: isochronous transfer buffers must be in internal SRAM (not PSRAM). DWC_OTG Scatter/Gather DMA allocates one QTD per frame. Isochronous transactions are NOT retried on failure — each QTD status must be checked individually.

### Phase 4: UAC1 Fallback

Some XMOS devices present `bNumConfigurations: 2` — one for UAC2, one for UAC1. When connected at Full Speed, devices with `XUA_AUDIO_CLASS_FS=1` (e.g., JDS Labs Atom DAC+) present UAC1 descriptors automatically.

The miniDSP 2x4 HD does NOT fall back (confirmed). But for broader device compatibility:

1. Check `bInterfaceProtocol` after enumeration
2. If 0x20 -> UAC2 path
3. If 0x00 -> delegate to Espressif's `usb_host_uac` (UAC1) or built-in UAC1 handling
4. Log which path was taken

## Design Goals

General-purpose ESP-IDF component. Not tied to any specific device or application.

1. **Composite device safe.** Claims only the audio interface — works alongside other class drivers (HID, CDC, etc.) on the same composite device.
2. **Bidirectional audio.** Both isochronous OUT (playback) and IN (capture). Same transfer pattern, direction-dependent branching.
3. **Any format the device supports.** Sample rate, bit depth, and channel count are queried from the device via RANGE and selected by the caller. Not hardcoded.
4. **Blocking read/write API.** Same pattern as Espressif's `uac_host_device_write()` / `uac_host_device_read()`. Ring buffer decouples caller from USB frame timing.
5. **Start timestamp.** `uac2_host_device_start()` returns `first_frame_us` — the `esp_timer_get_time()` value when the first isochronous transfer is submitted. Useful for any latency-sensitive application.
6. **Opens by address, not VID/PID.** The caller is responsible for device discovery. The driver opens whatever UAC2 device it's pointed at.
7. **Lightweight.** Target ~40-50 KB internal SRAM (driver + ring buffers + URBs). DMA buffers in internal SRAM; application may place its own buffers in PSRAM.

**Nice-to-have (not MVP):**
- Volume/mute control via Feature Unit
- UAC1 fallback detection/delegation

## Primary Test Target

minidsp-open (Rust on ESP-IDF) is the first consumer, calling this C component via FFI. Testing against miniDSP hardware (XMOS XU216, UAC2, VID `0x2752`):

| Device | PID |
|--------|-----|
| 2x4 HD / Flex / Flex DL | 0x0011 / 0x0044 |
| Flex Eight / Eight DL | 0x0012 / 0x004A |
| Flex HT / HTx | 0x004B |
| SHD / Studio / Power | 0x0045 |
| DDRC-24 | 0x0044 |

Key test scenario: composite device (HID already claimed), 48kHz/24-bit/stereo OUT, sustained streaming.

## API (Implemented)

See `components/uac2_host/include/uac2_host.h` for the full API. Summary:

```c
// Device management
esp_err_t uac2_host_device_open(client, usb_dev, cb, cb_arg, &handle);
esp_err_t uac2_host_device_close(handle);
esp_err_t uac2_host_device_get_info(handle, &info);

// Clock control
esp_err_t uac2_host_get_sample_rate(handle, &rate);
esp_err_t uac2_host_set_sample_rate(handle, rate);
esp_err_t uac2_host_get_sample_rate_range(handle, ranges, &count);
esp_err_t uac2_host_get_clock_valid(handle, &valid);

// Streaming
esp_err_t uac2_host_stream_start(handle, dir, &config);
esp_err_t uac2_host_stream_stop(handle, dir);
esp_err_t uac2_host_stream_write(handle, data, size, timeout_ms);
esp_err_t uac2_host_stream_read(handle, data, size, &bytes_read, timeout_ms);
int64_t   uac2_host_stream_get_start_time(handle);  // microsecond timestamp

// Volume/mute (raw 1/256 dB units)
esp_err_t uac2_host_set_volume(handle, channel, volume_db256);
esp_err_t uac2_host_get_volume(handle, channel, &volume_db256);
esp_err_t uac2_host_set_mute(handle, channel, mute);
esp_err_t uac2_host_get_mute(handle, channel, &mute);
```

Note: The current API requires the caller to manage the USB Host client and device enumeration. A future release will add `uac2_host_install()`/`uac2_host_uninstall()` matching the Espressif driver pattern. See `docs/TODO-prep-driver-for-release.md`.

## Key References

### Descriptor struct sources (copy/adapt directly)
- **USBX UAC2 structs** (MIT): `github.com/eclipse-threadx/usbx` — `ux_class_audio20.h` has every UAC2 descriptor struct, control selector, and format code
- **CherryUSB UAC2 structs** (Apache-2.0): `github.com/cherry-embedded/CherryUSB` — `class/audio/usb_audio.h` has UAC2 structs alongside UAC1

### Architecture to fork
- **Espressif UAC1 driver**: `github.com/espressif/esp-usb/tree/master/host/class/uac/usb_host_uac` — `uac_host.c` (~1400 lines), `uac_descriptors.c`, state machine, ring buffers, isochronous transfer management
- **CherryUSB UAC1 host**: `github.com/cherry-embedded/CherryUSB` — `class/audio/usbh_audio.c` (~550 lines), clean UAC1 host pattern

### Protocol reference (read-only, GPL)
- **Linux `sound/usb/clock.c`**: Clock topology walking (`__uac_clock_find_source`)
- **Linux `sound/usb/stream.c`**: UAC2 descriptor parsing
- **Linux `sound/usb/quirks-table.h`**: Known device-specific workarounds

### Proves isochronous audio works on ESP32-S3
- **esp32-rtp** (`github.com/netham45/esp32-rtp`): ESP32-S3 USB host -> UAC1 DAC audio output. Continuous isochronous streaming, dual-core, 48kHz/16-bit/stereo.

### USB Audio Class 2.0 spec
- USB Audio Class 2.0 with errata (usb.org)
- XMOS USB Audio Design Guide: `xmos.com/file/sw_usb_audio-design-guide`
- XMOS lib_xua clock/fallback docs: `github.com/xmos/lib_xua/blob/develop/doc/rst/opt_audio_class.rst`

### miniDSP descriptor dump
- Arduino USB Host Shield descriptor dump: `github.com/felis/USB_Host_Shield_2.0/issues/594` — actual miniDSP 2x4 HD descriptors at Full Speed

## Testing

### Test devices
- miniDSP 2x4 HD (XMOS XU216, UAC2, VID 0x2752 PID 0x0011) — primary target, confirmed UAC2-only at FS
- JDS Labs Atom DAC+ (XMOS, UAC2 at HS, UAC1 fallback at FS) — UAC1 baseline/comparison
- Any cheap USB sound card (~$10, UAC1) — sanity check
- Apple USB-C to 3.5mm dongle (~$9, UAC1) — another UAC1 baseline

### Test plan
1. Enumerate device, log all descriptors (UAC1 vs UAC2 detection)
2. Set sample rate via clock source control (CUR request)
3. Stream 1kHz sine tone, verify audio output with oscilloscope or ears
4. Stream 48kHz/24-bit/stereo, verify no glitches over 30 seconds
5. Test with composite device (miniDSP: HID + Audio simultaneously)
6. Test disconnect/reconnect handling (DWC_OTG isochronous channel cleanup)
7. Test UAC1 fallback path with JDS Labs DAC+

## Resolved Questions

- **miniDSP Full Speed enumeration**: Confirmed — presents UAC2 descriptors at Full Speed (Arduino USB Host Shield descriptor dump). Does NOT fall back to UAC1.
- **Bandwidth**: 48kHz/24-bit/stereo = 288 bytes/frame = ~25% bus utilization. Comfortable margin.
- **Memory**: ~40-50 KB internal SRAM. Fits easily on ESP32-S3 alongside WiFi.
- **Isochronous transfers**: Proven working on ESP32-S3 by esp32-rtp and usb_host_uac.

## Resolved Questions

- **Composite device**: ESP-IDF supports multiple USB Host clients. The UAC2 driver can coexist with HID on the same composite device. Needs final verification on real miniDSP.
- **Async mode feedback**: Implemented. Feedback endpoint handling works at Full Speed. 3-byte (10.14) and 4-byte (16.16) formats both supported. Adaptive packet sizing via accumulator pattern.
- **miniDSP clock topology**: Known from descriptor dump: Clock Source ID=41 (internal programmable), Clock Selector ID=40 (1 input from source 41). Single clock path.
- **DWC_OTG disconnect recovery**: Tested. Disconnect during streaming handled cleanly via atomic in-flight URB tracking + spinlock-protected state transitions. No crashes observed.

## Remaining Open Questions

- **Real miniDSP hardware**: All testing so far against simulator. Need to verify with actual miniDSP 2x4 HD (descriptor match, feedback format, POST-SET_INTERFACE timing).
- **Full config descriptor**: Simulator provides a subset. Real miniDSP has 373-byte config descriptor with HID + DFU interfaces we haven't tested parsing against.
