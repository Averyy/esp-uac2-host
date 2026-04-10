# Progress

## Status: v0.1.2 — validated on current hardware, still under active testing. UAC2 host driver with Espressif class driver pattern, internal device discovery, linked lists, and reference counting. The 12-test boot harness passes against both ESP32-to-ESP32 simulator and real miniDSP 2x4 HD hardware, and recent live miniDSP reruns also pass suspend/resume, duplex guard, and active unplug/replug recovery. Driver is feature-complete for single-clock UAC2 devices, but validation coverage is still expanding.

## Completed

### Multi-Packet URB Fix for Audio Dropout (April 10, 2026)
- [x] Version bumped to 0.1.2
- [x] `UAC2_NUM_PACKETS_PER_URB` made configurable via Kconfig (default 3, range 1-8) — was hardcoded to 1
- [x] `stream_tx_xfer_submit()` rewritten to fill multiple isochronous packets per URB with contiguous data packing (matching DWC_OTG `_buffer_fill_isoc` layout, confirmed by reading `hcd_dwc.c`)
- [x] Per-packet feedback-adjusted sizing preserved inside the multi-packet loop
- [x] Per-packet low-watermark TX_DONE signaling preserved (review finding: URB-granular signaling would delay refill)
- [x] HCD pipeline increased from 3ms (3 URBs × 1 pkt) to 9ms (3 URBs × 3 pkts), giving 6ms resubmission margin
- [x] Fixes periodic audio dropouts (~1/sec blips) when running under system load (WiFi + HTTP + HID), caused by WiFi beacon processing stalling USB callbacks beyond the previous 2ms margin
- [x] Verified on real miniDSP 2x4 HD: standalone test harness plays clean, and minidsp-open playback (200 Hz tone via `POST /play`) is blip-free with WiFi active
- [x] Dobby 3-agent review (correctness, performance, regression): no bugs found; TX_DONE signaling regression fixed before commit

### Live miniDSP Cleanup + Validation (April 9, 2026)
- [x] Version bumped to 0.1.1
- [x] `docs/TODO-multiclock.md` now tracks only the remaining multi-clock support work
- [x] Close-path interface release retry logging quieted; ESP-IDF bug `#17707` retry handling retained without repeated normal-shutdown warning spam
- [x] Attached-device close heap reporting clarified so retained host bookkeeping is reported accurately instead of as a leak-style warning
- [x] `uac2_host_device_resume()` fixed to reclaim the interface and resume an active playback stream on real miniDSP hardware
- [x] Harness live checks added and validated on real hardware: suspend/resume and ESP32-S3 duplex guard rejection
- [x] Full 12-test harness rerun passes on the real miniDSP 2x4 HD after the teardown/logging and suspend/resume changes
- [x] Active playback unplug/replug rerun passes after the latest changes: `USB_HOST_LIB_EVENT_FLAGS_ALL_FREE` observed, re-enumeration clean, next-cycle heap baseline restored to delta 0
- [x] Device task stack increased 12 KB -> 16 KB for the expanded harness; final watermark 4360 bytes free of 16384

### Phase 5 — Install/Uninstall Lifecycle Refactor (April 6, 2026)
- [x] Refactored to Espressif USB host class driver pattern (matching CDC-ACM, HID, MSC, UAC1)
- [x] `uac2_host_install(config)` / `uac2_host_uninstall()` — singleton driver, internal USB Host client
- [x] `uac2_host_handle_events(timeout)` — manual or background task event pumping
- [x] Internal device discovery: parses config descriptor on NEW_DEV, fires driver callbacks per AS interface
- [x] Internal disconnect handling: iterates interfaces, stops streams, fires per-interface DISCONNECTED event
- [x] Two-level struct split: `uac2_device_t` (physical, shared) + `uac2_iface_t` (per-interface, user handle)
- [x] `uac2_stream_t` fields folded into `uac2_iface_t` — no more separate allocation
- [x] Singleton `uac2_driver_t` with STAILQ linked lists (devices + interfaces)
- [x] Lazy physical device creation with reference counting (opened_cnt)
- [x] New `uac2_host_device_open(config, &handle)` — takes addr+iface_num from driver callback
- [x] All public API renamed: `stream_*` → `device_*`, direction implicit from interface
- [x] Device handle validation: `is_interface_in_list()` on all public functions
- [x] `uac2_host_get_device_alt_param()` convenience function
- [x] TX silence behavior documented in header
- [x] `FLAG_STREAM_SUSPEND_AFTER_START` support in `uac2_host_device_start()`
- [x] `main.c` rewritten: no manual USB client, driver callback creates device task
- [x] LICENSE copied into `components/uac2_host/`
- [x] Initial pre-release version tagged
- [x] All battle-tested logic preserved: URB callbacks, feedback, state machine, error handling
- [x] Both host and simulator build clean with zero warnings (`-Werror -Wextra`)
- [x] `uac2_host_device_set_volume_all_channels()` — iterates bmaControls bitmap
- [x] 4-agent code review: all 12 findings from that review round fixed (handle validation, disconnect race, gone flag, URB leak state, refcount safety, 5s timeout fix)
- [x] Device discovery close-before-callback fix (USB device must be released before firing connect callback)
- [x] All 12 tests pass against ESP32-to-ESP32 simulator (zero errors, zero disconnects)
- [x] All 12 tests pass against real miniDSP 2x4 HD (zero errors, real feedback 48.0000 Hz)
- [x] Repeated miniDSP hot-unplug/replug cycles verified on hardware: 3-second unplug/replug and immediate unplug/replug both recover cleanly, with `USB_HOST_LIB_EVENT_FLAGS_ALL_FREE` observed before reconnect and no heap baseline drift across cycles
- [x] Stack watermark: 4004 bytes free of 12288 (67% headroom)
- [x] Fixed `atomic_init` UB on reused interface struct (was `atomic_init`, now `atomic_store`)
- [x] Fixed stream resource leak on disconnect (stream_stop_internal now frees resources when state is IDLE but resources exist)
- [x] Fixed driver teardown for active hot-unplug: release interface even after `DEV_GONE`, wait for host `ALL_FREE` before heap accounting, and verified clean reconnect on real miniDSP hardware
- [x] TODO-part2.md fully completed and deleted

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
- [x] First-frame timestamp API (`uac2_host_device_get_start_time()`)
- [x] Interface release retry loop (5 attempts, for ESP-IDF bug #17707)
- [x] ESP-IDF component restructure (main/ -> components/uac2_host/)
- [x] Test suite: 5 automated tests (48kHz, volume/mute, stop/restart, 44.1kHz, long-running)
- [x] Disconnect/reconnect tested and verified clean
- [x] All tests pass after all fixes applied

### miniDSP 2x4 HD Simulator — Full Composite Device (April 5, 2026)
- [x] Restructured `test_device/` → `simulators/simple/` + `simulators/minidsp-2x4hd/`
- [x] Composite TinyUSB device: UAC2 audio (playback IF1, alt 1 24-bit + alt 2 16-bit) + HID (IF2, 64-byte vendor reports)
- [x] Full AC topology in descriptor: playback path (IT2→FU10→OT20) + capture path (IT1→FU11→OT22) — matches real miniDSP
- [x] Config descriptor: 300 bytes (all AC entities, playback AS with 2 alt settings, HID — no DFU/capture AS due to TinyUSB constraints)
- [x] PID changed to 0x0011 (real miniDSP PID), VID 0x2752, bcdDevice 0x06F2
- [x] HID command protocol: all 20 commands implemented (ReadHardwareId, ReadFlash, ReadFloats, SetVolume, SetMute, SetConfig, SetSource, WriteDSP, WriteBiquad, etc.)
- [x] EEPROM state: preset, source, volume, mute, serial, mod tokens, DSP ID
- [x] DSP parameter storage: WriteDSP/WriteBiquad store float values, ReadFloats reads them back
- [x] Fault injection: config switch delay, empty HID response, HID NAK, command timeout (serial console toggle)
- [x] USB lifecycle callbacks: mount/unmount/suspend/resume logging
- [x] Fixed TinyUSB callback name: `tud_audio_rx_done_pre_read_cb` → `tud_audio_rx_done_isr` (both simulators)
- [x] 6-agent code review completed: all security, performance, and code quality fixes applied
- [x] All 5 host driver tests pass against new simulator (48kHz, volume/mute, stop/restart, 44.1kHz, long-running)

### SET_INTERFACE Fix + Code Hardening (April 5, 2026)
- [x] **SET_INTERFACE crash root cause found and fixed**: `tud_audio_rx_done_isr` was calling `ESP_LOGI` (which needs locks) from USB ISR context. Replaced with ISR-safe `ESP_DRAM_LOGI`. Crash was never in DWC2 — the ISR callback worked fine once logging was fixed.
- [x] Host driver now sends proper SET_INTERFACE to device: `ctrl_request_no_data()` auto-releases mutex
- [x] SET_INTERFACE(alt=0) sent on stream stop (before halt/flush/clear, matching Espressif UAC1 pattern)
- [x] SET_INTERFACE(alt=0) sent in stream_start error cleanup path
- [x] Endpoint halt/flush/clear on stream stop
- [x] Transfer error limiting: stops URB re-submission after 10 consecutive errors
- [x] Fixed RX URB leak: resubmit failure now decrements `urbs_in_flight` counter
- [x] Fixed feedback resubmit race: state re-checked under spinlock before resubmit
- [x] Simulator receives audio frames end-to-end (487,000+ frames verified, zero errors)
- [x] Fixed uint32_t cast UB in WriteDSP/WriteBiquad shift operations
- [x] Fixed device descriptor: iProduct=3, iSerialNumber=0 (matches real miniDSP)
- [x] Added WriteFlash page 0xFF handler with bounds checking
- [x] Added HID frame decode buffer size validation
- [x] All 5 host driver tests pass with SET_INTERFACE fully working

### ESP-IDF Convention Cleanup + Code Review (April 6, 2026)
- [x] Headers moved to `include/usb/` (matches Espressif convention)
- [x] SPDX copyright headers on all source files
- [x] Espressif naming conventions: `s_` prefix for statics, no `_` prefix on file-scope functions
- [x] `heap_caps_calloc`/`heap_caps_free` for explicit memory control
- [x] `<inttypes.h>` format specifiers (`PRIu32` replaces `(unsigned long)` casts)
- [x] `esp_check.h` macros (`ESP_RETURN_ON_FALSE`) on all public API functions
- [x] Control transfer debug hex dump (`ESP_LOG_BUFFER_HEXDUMP` at DEBUG level)
- [x] EP0 halt/flush/clear recovery after control transfer timeout
- [x] Safe ringbuffer deletion (unblock waiting tasks before delete, conditional delay)
- [x] VID/PID and string descriptors exposed in `uac2_device_info_t`
- [x] Version macros (`UAC2_HOST_VER_MAJOR/MINOR/PATCH`)
- [x] bInterval fixup for Full Speed data endpoints (warn + patch, skip feedback EPs)
- [x] Low-speed device rejection
- [x] Kconfig for tunable parameters (URB count, timeout, max size, error limit)
- [x] `idf_component.yml` manifest for ESP Component Registry
- [x] 5-agent code review: all critical/high/medium/low findings fixed
- [x] Fixed swapped ringbuffer unblock logic (TX drains for space, RX sends dummy for data)
- [x] Fixed feedback URB `urbs_in_flight` leak on state transition
- [x] Set stream state to ACTIVE before URB submission (prevents callback race)
- [x] Safe stream stop groundwork: initially switched from unsafe free to resource retention when URBs were still in-flight
- [x] Protected `device_close` from concurrent control requests (`closing` flag)
- [x] Per-stream spinlock (replaces global `s_uac2_stream_lock`)
- [x] State recheck in `stream_write`/`stream_read` after blocking ringbuf call
- [x] TX_DONE single-shot flag (prevents event spam during underrun)
- [x] Feedback callback consecutive error limit check
- [x] Atomic `first_frame_us` for 32-bit safety
- [x] Event callback context documented (must not block or call driver APIs)
- [x] `device_close` contract documented (doesn't close underlying USB device)
- [x] Interface descriptor minimum length check in parser
- [x] URB drain wait in `stream_start` error path
- [x] Stale semaphore drain after EP0 timeout recovery
- [x] Reduced stack usage in `device_open` (parse directly into heap)
- [x] All builds pass with zero errors, zero warnings

### Live Test Verification (April 6, 2026)
- [x] Flashed host + simulator on two ESP32-S3-DevKitC-1 boards
- [x] Test 1 PASS: 48kHz/24-bit/stereo, 10 sec sustained
- [x] Test 2 PASS: Volume/mute control during streaming (set/get mute, set/get volume -12dB)
- [x] Test 3 PASS: Stop/restart cycle (5s 440Hz → stop → 2s pause → 5s 880Hz)
- [x] Test 4 PASS: 44.1kHz switch smoke test, 5 sec sustained stream
- [x] Test 5 PASS: Start time precision stays within the expected before/after window across repeated starts
- [x] Simulator confirms: all SET_INTERFACE transitions, sample rate switching, volume/mute, zero faults
- [x] Harness feed math verified: 48kHz uses fixed 288-byte / 10 ms chunks, 44.1kHz uses fractional 10 ms chunking instead of integer truncation
- [x] Identical behavior to pre-cleanup code — zero regressions
- [x] April 9 rerun after control/harness hardening: reflashed host + simulator, `TEST 1` through `TEST 11` passed again on the ESP32-to-ESP32 setup
- [x] April 9 simulator coverage expansion: added reduced no-feedback and playback channel-only feature-unit build variants, profile-specific product-string markers/assertions, and a configurable Test 12 duration for short smoke reruns

### Known Limitations (hardware/framework — not fixable in driver code)
- [x] No simultaneous TX+RX on ESP32-S3 — PERIODIC_OUT FIFO bias gives RX only 128 bytes, audio packets are ~294 bytes. The driver now rejects opposite-direction stream activation at runtime. Requires ESP32-P4 (4KB FIFO) for true duplex.

### Clock Topology Fixes (April 6, 2026)
- [x] `resolve_clock_source()` rewritten: proper topology walk from terminal→selector/multiplier→clock source (was: blindly pick first clock source)
- [x] Clock multiplier parsing added (`parse_clock_multiplier()`, subtype 0x0C) — was silently skipped
- [x] Clock multiplier struct + storage in `uac2_device_info_t`
- [x] Debug print includes clock multipliers
- [x] Topology walk handles selectors (follows first input pin) and multipliers (follows upstream source)
- [x] Max 8 hops with fallback prevents infinite loops from malformed descriptors

### Real miniDSP 2x4 HD Testing (April 6, 2026)
- [x] Full 373-byte config descriptor captured from real device (replaces partial reconstruction)
- [x] Enumeration and clock queries succeed (sample rates: 44100 + 48000 Hz discrete)
- [x] Feedback format confirmed: **4-byte 16.16** (not 3-byte 10.14 as assumed). Value: 0x00300000 (48.0000). Locks immediately.
- [x] Volume range confirmed: -127 to 0 dB, 1 dB resolution
- [x] VBUS fix: bridge `USB-OTG` solder pads on DevKitC-1 back (no powered hub needed)
- [x] 12-test comprehensive suite: basic streaming, volume/mute, stop/restart, 44.1kHz switch smoke test, start time precision, stable feedback + clock validity, 16-bit mode, 9x rapid measurement cycles, per-channel volume/mute readback + restore, sample rate switch stress, ring buffer starvation/recovery, long-running stability
- [x] 766-second stability run, zero errors
- [x] Disconnect/reconnect: clean teardown, automatic re-enumeration and test restart
- [x] New driver APIs: `uac2_host_device_get_volume_range()`, `uac2_host_device_get_feedback()`

### Hot-Unplug Validation Updates (April 7, 2026)
- [x] Heap accounting moved to after `USB_HOST_LIB_EVENT_FLAGS_ALL_FREE` so the harness no longer reports false leak warnings from asynchronous host cleanup
- [x] Active unplug teardown fixed: release interface claims even after `DEV_GONE`, keep resources owned until callbacks are quiesced, and fail close instead of freeing live callback context
- [x] Close-path lifetime hardening: normal APIs reject `closing` handles while `device_close()` performs privileged teardown
- [x] Real miniDSP validation: repeated 3-second unplug/replug cycles and immediate unplug/replug both complete cleanly with no crash, `ALL_FREE` observed, and next-cycle heap baseline restored

### Simulator Update to Match Real Device (April 6, 2026)
- [x] Config descriptor: 300 → 373 bytes (matches real device exactly)
- [x] 5 interfaces: AC, AS playback (24/16-bit), AS capture, DFU (stub driver), HID
- [x] HID report descriptor: 28 bytes captured from real device via hidapi
- [x] Feedback: 4-byte 16.16 (disabled TinyUSB format correction)
- [x] HID endpoint: EP 0x83 IN (was 0x82, shifted by capture interface)
- [x] bcdDevice=0x0185, iProduct=11, all string indices, terminal bmControls — byte-matched
- [x] 3-byte HID address parsing fixed: ReadFloats, WriteDSP, WriteBiquad, BypassFilter
- [x] DFU stub driver (`usbd_app_driver_get_cb`) prevents TinyUSB SET_CONFIGURATION assert
- [x] Builds clean, static asserts verify 373-byte descriptor size

## Downstream Integration (not in this repo)

Phase 4 items (Rust FFI integration, sweep generator, `POST /play` endpoint) are the responsibility of **minidsp-open** (`~/code/minidsp-open`). This project is the standalone UAC2 host driver component only. See `~/code/minidsp-open/docs/TODO-esp32-usb-audio.md`.

### Code Review Findings — ALL FIXED (April 6, 2026)
- [x] 4 HIGH: device_close SET_INTERFACE fix, usb_string_to_ascii underflow guard, DISCONNECTED event firing, stream_free use-after-free (semaphore handshake)
- [x] 9 MED: nr_pins OOB, calc_packet_size overflow, feedback validation, tx_done_pending atomic, closing atomic, TOCTOU fixes, AS truncation logging, ctrl timeout gen counter, fb_accumulator documented
- [x] 6 LOW: descriptor parser bounds checks, ep_interval no-mutate, duplicate vid/pid removed, Kconfig text, tone_gen assert
- [x] Review fixes: stream_write/read signal-before-access ordering, semaphore leak in error paths, calc_packet_size==0 guard, ctrl_xfer_submitted_gen made atomic

### TODO Part 2 Features — IMPLEMENTED (April 6, 2026)
- [x] Suspend/resume (`uac2_host_device_suspend()` / `uac2_host_device_resume()`) — no URB/ringbuf realloc
- [x] API mutex on all public functions (except stream_write/stream_read which use ringbuf sync)
- [x] Stream dead notification: `UAC2_STREAM_STATE_ERROR` + `UAC2_HOST_EVENT_STREAM_ERROR` on max errors
- [x] Volume/mute hardening: bmaControls parsed, capability checks, range caching+validation, percent API
- [x] Debug print (`uac2_host_device_print_info`) — topology, runtime state, stream info
- [x] Sample rate validation at stream_start (warns if rate not in advertised ranges)

### Future (optional, not blocking)
- [ ] Component registry packaging: `examples/` directory and Doxygen docs (compiler flags and `idf_component.yml` already done)

## Hardware

- **MCU:** ESP32-S3-DevKitC-1 (Full Speed USB OTG, 1024B FIFO)
- **Test device:** miniDSP 2x4 HD (XMOS XU216, UAC2, VID 0x2752 PID 0x0011)
- **VBUS:** Bridge `USB-OTG` solder pads on DevKitC-1 back to supply 5V on OTG port.
- **Limitation:** No simultaneous TX+RX on ESP32-S3 (PERIODIC_OUT FIFO bias gives RX only 128 bytes, audio packets are ~294 bytes)

## Key Research Findings

- XMOS sends **4-byte feedback at FS (16.16 format)** — confirmed by real miniDSP testing (was assumed 3-byte 10.14)
- Feedback value at 48kHz: 0x00300000 (48.0000 samples/frame), locks immediately, drift <0.002%
- Feedback arrives every 8ms (bInterval=4 → 2^3 frames)
- ESP-IDF USB Host interface release bug (#17707) — retry logic added
- miniDSP is self-powered (bmAttributes=0xC0) — still needs VBUS present to enumerate
- DevKitC-1 USB-OTG solder pads on back of board enable VBUS output (no powered hub needed)
