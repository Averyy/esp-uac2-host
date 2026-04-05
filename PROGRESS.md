# Progress

## Status: Research complete, prep done. Ready for Phase 0A. miniDSP arriving in a few days.

## Prep Completed (April 5, 2026)

- [x] ESP-IDF v5.4 verified building for ESP32-S3 (USB host example compiles clean)
- [x] Reference code downloaded to `ref/` (~7,100 lines — Espressif UAC1 driver, USBX + CherryUSB UAC2 structs)
- [x] miniDSP 2x4 HD descriptor dump reconstructed as C byte array (`ref/minidsp_2x4hd_descriptors.h`)
- [x] Serial MCP tools configured for ESP32 interaction (19 tools auto-granted)
- [x] Board confirmed: ESP32-S3-DevKitC-1 (dual USB-C: UART + OTG)

## Plan of Approach

### Phase 0A — Scaffold + Compile (no device needed)
Set up ESP-IDF component, get it compiling for ESP32-S3. Verify USB host mode initializes on the bare board.

Serial workflow: After flashing, open the ESP32 console port with `serial_open` and `serial_wait_for` to confirm "waiting for device" appears in the boot log.

- [ ] ESP-IDF project scaffolded (CMake, component structure, Kconfig)
- [ ] Builds and flashes to ESP32-S3
- [ ] USB Host Library initializes, logs "waiting for device" on serial

### Phase 1 — UAC2 Descriptor Parsing (no device needed)
Port UAC2 descriptor structs from USBX (MIT) or CherryUSB (Apache-2.0). Write parser that walks a configuration descriptor and extracts AC entities and AS interfaces. Test against the known miniDSP descriptor dump as a static byte array.

- [ ] UAC2 descriptor struct definitions (clock source, selector, multiplier, input/output terminal, feature unit, AS general, format type I)
- [ ] Configuration descriptor walker — iterate class-specific descriptors within AC and AS interfaces
- [ ] Clock topology extraction — map terminal -> clock source chain
- [ ] AS interface parsing — alternate settings, format types, endpoint addresses, max packet sizes
- [ ] UAC2 detection (`bInterfaceProtocol == 0x20`) with UAC1 fallback detection
- [ ] Tested against static byte array of miniDSP descriptor dump (from USB_Host_Shield_2.0#594)

### Phase 2 — Clock + Streaming Code (no device needed)
Write CUR/RANGE request builders, clock topology walker, ring buffer, isochronous transfer state machine, and public API. All compiles but can't be tested until a device is connected.

- [ ] Clock topology walker (terminal `bCSourceID` -> optional selector -> clock source)
- [ ] CUR/RANGE control request builder functions
- [ ] Ring buffer TX path: write() -> ring buffer -> fill URB -> submit -> callback -> refill
- [ ] Ring buffer RX path: submit -> callback -> copy to ring buffer -> resubmit -> read()
- [ ] Streaming state machine (idle -> ready -> active -> suspending)
- [ ] Public API header (`uac2_host.h`)
- [ ] `uac2_host_device_open()` / `start()` / `write()` / `read()` / `close()`
- [ ] `first_frame_us` timestamp via `esp_timer_get_time()` at first URB submit
- [ ] ESP-IDF component packaging (CMakeLists.txt, Kconfig, idf_component.yml)
- [ ] Everything compiles clean for ESP32-S3

### Phase 0B — Hardware Verification (needs miniDSP)
Plug miniDSP into ESP32-S3. Enumerate, dump all descriptors, confirm UAC2, capture clock topology. This retroactively validates the parsing code from Phase 1.

Serial workflow: User plugs in the miniDSP, then Claude reads the descriptor dump directly via `serial_read` / `serial_wait_for`. Log the full dump with `serial_log_start` to a file in `docs/` for reference. Use `serial_read_since` to re-examine output without re-triggering enumeration.

- [ ] ESP32-S3 enumerates miniDSP via USB Host Library
- [ ] Full descriptor dump logged to serial (all configs, interfaces, endpoints)
- [ ] UAC2 confirmed (`bInterfaceProtocol=0x20`)
- [ ] Clock source entity IDs and topology documented
- [ ] Alternate settings and endpoint max packet sizes documented
- [ ] HID + Audio interfaces both visible on the composite device
- [ ] Descriptor parser output matches actual device descriptors

### Phase 3 — Live Device Testing (needs miniDSP)
Run the code from Phase 2 against the real device. Debug whatever breaks.

Serial workflow: Keep `serial_open` active throughout. Use `serial_wait_for` to detect specific events (e.g., `pattern="RANGE"` after sending clock query, `pattern="Guru Meditation"` to catch crashes). After a crash, `serial_read_since` captures the full backtrace without losing it. Use `serial_set_signals` to reset the ESP32 between test runs instead of asking the user to press the reset button.

- [ ] Clock config: GET RANGE returns supported rates, SET CUR accepts 48kHz
- [ ] Claim audio streaming interface (alongside already-claimed HID)
- [ ] SET_INTERFACE activates the correct alternate setting
- [ ] Isochronous OUT endpoint opened, URBs allocated
- [ ] 1kHz sine tone plays through miniDSP — audible confirmation
- [ ] 48kHz/24-bit/stereo OUT sustained for 30 seconds without glitches
- [ ] Composite device verified — HID commands work while audio streams

### Phase 4 — Polish + Integration (needs miniDSP)
Error handling, disconnect recovery, final API cleanup. Ready for minidsp-open FFI integration.

- [ ] Clean shutdown on device disconnect (DWC_OTG soft reset for isoc channels)
- [ ] `uac2_host_get_sample_rates()` tested against real RANGE response
- [ ] Integration test: minidsp-open Rust FFI calls this driver, plays a sweep

### Future (not MVP)
- [ ] Volume/mute control via Feature Unit
- [ ] UAC1 fallback delegation to `usb_host_uac`
- [ ] Async feedback endpoint handling
- [ ] Multi-device support

## Blocking Questions

| Question | Status | Answer |
|----------|--------|--------|
| Does miniDSP fall back to UAC1 at FS? | **Resolved** | No. Presents UAC2 (protocol 0x20) at Full Speed. |
| Is UAC2 at Full Speed valid? | **Resolved** | Yes. XMOS default, spec allows it. |
| Is FS bandwidth enough? | **Resolved** | Yes. 288 bytes/frame = ~25% bus utilization. |
| Does any UAC2 ESP32 host driver exist? | **Resolved** | No. This is the first. |
| Can HID + Audio coexist on composite device? | **Open** | ESP-IDF supports it in principle. Needs Phase 0B/3 verification. |
| What is the miniDSP's clock topology? | **Open** | Partially known from Arduino dump. Full details in Phase 0B. |
| Async feedback at Full Speed? | **Open** | May need tuning. Deferrable for MVP. |

## Dependencies

```
open-sub-optimizer (Phase 7: network measurement)
  --> minidsp-open (POST /play endpoint)
    --> esp-uac2-host (this project)
      --> ESP-IDF USB Host Library (proven, no work needed)
```

## Research

Full research findings in `docs/ref-research.md`. Key references:
- Espressif `usb_host_uac` v1.3.3 — architecture to fork
- USBX `ux_class_audio20.h` (MIT) — UAC2 descriptor structs
- CherryUSB `usb_audio.h` (Apache-2.0) — UAC2 descriptor structs
- Linux `sound/usb/clock.c` — clock topology walking (GPL, read-only ref)
- miniDSP descriptor dump — github.com/felis/USB_Host_Shield_2.0/issues/594
