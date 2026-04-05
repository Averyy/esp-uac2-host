# Progress

## Status: Phase 2 complete + live UAC2 device tested via ESP32-to-ESP32 simulator. Control requests verified end-to-end.

## Completed

### Phase 0A — Scaffold + Compile (April 5, 2026)
- [x] ESP-IDF project scaffolded (CMake, sdkconfig.defaults, Kconfig overrides)
- [x] Builds and flashes to ESP32-S3 (zero warnings)
- [x] USB Host Library initializes, logs "Waiting for USB device..." on serial
- [x] Two-task FreeRTOS pattern: USB Host Library task + Class Driver task

### Phase 1 — UAC2 Descriptor Parsing (April 5, 2026)
- [x] UAC2 descriptor struct definitions in `uac2_desc.h` (clock source, selector, terminals, feature unit, AS general, format type I, CS endpoint)
- [x] Configuration descriptor walker in `uac2_desc.c` — iterates AC and AS class-specific descriptors
- [x] Clock topology extraction (source -> selector chain)
- [x] AS interface parsing (alt settings, format types, endpoints, MPS, feedback EP)
- [x] UAC2 detection via `bInterfaceProtocol == 0x20`
- [x] Self-test against static miniDSP 2x4 HD descriptor dump — all assertions pass on hardware
- [x] Signal chain tracing and full logging

### Phase 2 — UAC2 Host Driver (April 5, 2026)
- [x] Public API header `uac2_host.h` (device open/close, stream start/stop/read/write, clock control, volume/mute)
- [x] Control request layer with CUR/RANGE support (SET_CUR, GET_CUR, GET_RANGE)
- [x] Mutex-protected control transfers with proper actual_num_bytes handling
- [x] Isochronous TX path: ringbuf -> fill URB -> submit -> callback -> refill (with threshold-based TX_DONE events)
- [x] Isochronous RX path: submit -> callback -> copy to ringbuf -> resubmit (with threshold-based RX_DONE events)
- [x] Async feedback endpoint handling (reads 10.14 or 16.16 format, converts to 16.16)
- [x] Stream state machine (idle -> ready -> active)
- [x] Ring buffer using ESP-IDF `xRingbufferCreate` (BYTEBUF type)
- [x] ESP32-S3 duplex guard (FIFO too small for simultaneous TX+RX)
- [x] Interface release retry for ESP-IDF bug #17707
- [x] Clock queries: get/set sample rate, get range, get clock validity
- [x] Volume/mute control via feature unit (set/get volume, set/get mute)
- [x] Compiles clean, self-test passes on hardware
- [x] main.c updated to use driver API (opens UAC2 devices, queries clock info)

### Live UAC2 Testing via ESP32-to-ESP32 Simulator (April 5, 2026)
- [x] UAC2 test device firmware built (TinyUSB, miniDSP 2x4 HD descriptor layout)
- [x] macOS enumerates simulator as "2x4HD UAC2 Simulator" audio device
- [x] macOS sends all expected control requests (clock, mute, volume) — all handled
- [x] Host driver enumerates simulator: `uac2_host_device_open()` succeeds
- [x] GET_CUR sample rate returns 48000 Hz
- [x] GET_RANGE returns 2 ranges (44.1kHz, 48kHz)
- [x] GET clock validity returns valid
- [x] Control transfer deadlock found and fixed (was blocking event loop; now runs in separate task)
- [x] Clock selector SET_CUR support added to simulator

### Known Issues (need hardware to resolve)
- [ ] Feedback-based adaptive packet sizing (currently fixed at nominal)
- [ ] Stream shutdown race condition (50ms delay instead of URB tracking)
- [ ] Clock topology walk for multi-clock devices (works for miniDSP's single clock)
- [ ] POST-SET_INTERFACE delay may be needed on XMOS
- [ ] Disconnect during streaming untested
- [ ] Feedback format confirmation (expect 3 bytes / 10.14 from XMOS at FS)

See `TODO(hardware)` markers in source code for details.

## Next: Phase 0B + 3 — Live Device Testing (needs powered USB hub + miniDSP)

### Phase 0B — Hardware Verification
- [x] ESP32-S3 enumerates USB device (verified with CDC + UAC2 simulator)
- [x] `uac2_host_device_open()` succeeds on UAC2 device
- [x] Clock queries return valid data (GET_CUR, GET_RANGE, clock validity)
- [ ] Full 373-byte config descriptor captured from real miniDSP (vs simulator)
- [ ] Capture AS interface descriptors documented from real device

### Phase 3 — Live Audio Streaming
- [ ] SET_CUR sample rate 48kHz accepted
- [ ] Interface claim + alt setting works
- [ ] Isochronous OUT transfers submit without errors
- [ ] Feedback endpoint receives data (log format and values)
- [ ] Implement feedback-based packet size adjustment
- [ ] Test tone plays through miniDSP
- [ ] 48kHz/24-bit/stereo sustained for 30+ seconds
- [ ] Disconnect/reconnect recovery

### Phase 4 — Polish + Integration
- [ ] Proper URB in-flight tracking for clean shutdown
- [ ] Error recovery (transfer errors, stalls)
- [ ] Integration with minidsp-open Rust FFI

## Hardware

- **MCU:** ESP32-S3-DevKitC-1 (Full Speed USB OTG, 1024B FIFO)
- **Test device:** miniDSP 2x4 HD (XMOS XU216, UAC2, VID 0x2752 PID 0x0011)
- **VBUS:** External power required (DevKitC-1 has Schottky diodes blocking VBUS out). Powered USB hub on order.
- **Limitation:** No simultaneous TX+RX on ESP32-S3 (PERIODIC_OUT FIFO bias gives RX only 128 bytes, audio packets are ~294 bytes)

## Key Research Findings

- XMOS sends 3-byte feedback at FS (10.14 format), despite wMaxPacketSize=4
- Feedback arrives every 8ms (bInterval=4 -> 2^3 frames)
- XMOS averages feedback over 128 SOFs (128ms)
- ESP-IDF USB Host interface release bug (#17707) — retry logic added
- miniDSP is self-powered (bmAttributes=0xC0) — still needs VBUS present to enumerate
