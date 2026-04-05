# Progress

## Status: Phase 3 complete. Isochronous streaming verified end-to-end (48kHz/24-bit/stereo, 40+ seconds sustained, zero errors).

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

### Phase 3 — Isochronous Streaming (April 5, 2026)
- [x] main.c updated with stream_start, stream_write, tone generator (1kHz sine)
- [x] SET_CUR sample rate 48kHz accepted by simulator
- [x] Interface claim + alt setting works (iface 1 alt 1)
- [x] Isochronous OUT transfers submit without errors
- [x] Feedback endpoint submitted (EP 0x81)
- [x] Feedback-based adaptive packet sizing implemented (accumulator pattern)
- [x] 48kHz/24-bit/stereo sustained for 40+ seconds (zero errors, zero glitches)
- [x] Ring buffer backpressure working correctly (consistent 1s timing between logs)
- [x] Pre-fill strategy (50ms buffer) prevents initial underrun
- [x] Control transfer timeout recovery (semaphore drain on timeout)
- [x] Device task lifecycle: proper disconnect wait + cleanup in handle_device_gone

### Code Review + Stability Hardening (April 5, 2026)
- [x] Full 6-agent code review (security, performance, architecture, quality, UX, regression)
- [x] Control transfer mutex race fixed (mutex held through response data copy)
- [x] Descriptor parser: bLength validation on all parse functions (defense against malformed USB)
- [x] Control transfer buffer overflow guard (w_length vs UAC2_CTRL_XFER_MAX_SIZE)
- [x] `volatile` qualifiers on cross-task shared state (dev_connected, dev_task_hdl)
- [x] URB resubmit return values checked in RX/feedback callbacks
- [x] Feedback 4-byte path: explicit little-endian deserialization (portable)
- [x] Feedback packet size calc: uint32 intermediate prevents overflow
- [x] ESP_ERROR_CHECK replaced with proper error handling in device task
- [x] Relative include path fixed (ref/ added to PRIV_INCLUDE_DIRS)
- [x] Component CMakeLists: esp_ringbuf moved to PRIV_REQUIRES
- [x] Atomic in-flight URB counter (replaces 50ms delay in stream_stop)
- [x] Spinlock (portMUX) for stream state transitions
- [x] Ring buffer flush on stream stop
- [x] First-frame timestamp API (uac2_host_stream_get_start_time)
- [x] Interface release retry loop (5 attempts, for ESP-IDF bug #17707)
- [x] ESP-IDF component restructure (main/ -> components/uac2_host/)
- [x] Test suite: 5 automated tests (48kHz, volume/mute, stop/restart, 44.1kHz, long-running)
- [x] Disconnect/reconnect tested and verified clean
- [x] All tests pass after all fixes applied

### Known Issues (need real miniDSP hardware to resolve)
- [ ] Clock topology walk for multi-clock devices (works for miniDSP's single clock)
- [ ] POST-SET_INTERFACE delay may be needed on real XMOS hardware
- [ ] Feedback format confirmation (expect 3 bytes / 10.14 from XMOS at FS)
- [ ] Full 373-byte config descriptor capture from real miniDSP (vs simulator)

See `TODO(hardware)` markers in source code for details.

## Next: Phase 4 — Real Hardware + Integration

### Phase 0B — Real miniDSP Verification (needs powered USB hub + miniDSP)
- [ ] Full config descriptor captured from real miniDSP 2x4 HD
- [ ] Enumeration and clock queries succeed on real device
- [ ] Streaming works on real device (may need POST-SET_INTERFACE delay)
- [ ] Feedback format confirmed (3-byte 10.14 vs 4-byte 16.16)

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
