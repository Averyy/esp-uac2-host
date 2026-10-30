# esp-uac2-host

USB Audio Class 2.0 host driver for ESP32-S3. ESP-IDF component (C). Streams audio to/from UAC2 devices (DACs, audio interfaces, miniDSP).

## Project State

**v0.2.0 (2026-10-30): verified on the miniDSP 2x4 HD; on the ESP Component Registry as `averyy/usb_host_uac2` (component folder `components/usb_host_uac2`; first release 0.1.3).** Full harness (every tone audible on a UMIK-1), hot-unplug and duplex pass under ESP-IDF 5.4.0 and 6.1; minidsp-open (IDF 6.0.3) built from the tree passes its sweeps and a 10-minute soak. An ESP32 hardware reset during playback sometimes leaves the miniDSP silent until it reboots (intermittent, see Key Facts). Duplex works on the 2x4 HD (2026-10-30, IDF 6.1 duplex profile: 30 s at full rate both ways, audible throughout, normal playback afterwards). Open items: `PROGRESS.md`. Architecture: Espressif class driver pattern (`uac2_host_install()`/`uac2_host_uninstall()`, internal discovery, refcounting); `uac2_device_t` (physical) + `uac2_iface_t` (per interface, the handle). API reference and internals: `llms.txt`.

## Related Projects

- **minidsp-open** (`~/code/minidsp-open`): primary consumer, via Rust FFI (`app-esp32/src/usb_audio.rs`). Since minidsp-open 96d0538 (2026-10-29) it takes the driver from the registry, `averyy/usb_host_uac2` pinned to exactly 0.1.3 in `app-esp32/Cargo.toml`; a release reaches it by bumping that pin (refreshes `app-esp32/components_esp32s3.lock`). To build it against this working copy: `ESP_UAC2_HOST_PATH="$HOME/code/esp-uac2-host/components/usb_host_uac2" tools/esp-cargo build --release` (`tools/esp-cargo` also carries the rebuild-tracking globs). Its FFI declarations are hand-written and must match `uac2_host.h` (checked 2026-10-29: in sync; it never calls `get_info`). It plays 48 kHz only, never captures, keeps the PERIODIC_OUT FIFO bias, and reads miniDSP level meters over HID (`GET /api/1/devices/master/levels`, refused during playback). Docs there: `docs/ref-playback.md`, `docs/TODO1-esp32-usb-audio.md`, `docs/TODO2-usb-audio-consumer-bugs.md`.
- **open-sub-optimizer** (`~/code/open-sub-optimizer`): calls minidsp-open's `POST /play`; doesn't touch this driver.
- **Espressif `usb_host_uac`** (esp-usb, Apache-2.0): UAC1 driver whose architecture this forks. `ref/espressif-uac1/` is its v1.4.0 source (mislabelled v1.3.3); upstream v1.5.0 fixes were reviewed for v0.1.3.
- **USBX** (MIT) and **CherryUSB** (Apache-2.0): UAC2 struct references used during development.

## Environment

- **ESP-IDF 5.4:** `~/esp/esp-idf` (exactly v5.4.0). Activate: `export PATH="/opt/homebrew/bin:$PATH" && . ~/esp/esp-idf/export.sh` (system python3 is Apple 3.9; IDF needs Homebrew Python), or the `esp` wrapper. 5.4.0 lacks the 5.4.2-5.4.4 USB Host fixes (#17707 release delay, device-close mutex leak, heap-poisoning transfer sizes, S3 host channel undercount, external-hub teardown); the root-port disconnect/recycle abort is fixed only in `espressif/usb` >= 1.4.1. The driver works around what it can (release retry, explicit feedback length, close delay).
- **ESP-IDF 6.x:** `~/esp/esp-idf-v6.1` installed; 6.0.3 (minidsp-open's planned version) builds in Docker `espressif/idf:v6.0.3`. IDF 6 removed the in-tree `usb` component; the manifest adds `espressif/usb ^1.6.0` for `idf_version >=6.0`. Earlier `espressif/usb` releases and every IDF 5.x USBH notify control completions before decrementing the in-flight count `usbh_dev_close()` asserts on, hence the driver's 10 ms pre-close delay on IDF < 6. Both 5.4 and `espressif/usb` reject halt/flush on EP0 and ignore `usb_transfer_t.timeout_ms`.
- **Board:** ESP32-S3-DevKitC-1 (test board MAC 3c:0f:02:d7:95:24, port `/dev/cu.usbmodem5B7B0529311`). Right USB-C = CH340 UART (VID 0x1A86): flashing + serial. Left USB-C = USB-Serial/JTAG on GPIO 19/20 = the OTG port, unavailable for serial in host mode. **Bridge the `USB-OTG` solder pads** on the back or the OTG port has no VBUS and nothing enumerates.
- **Flashing:** never let a serial capture run during a flash (it grabbed the port mid-flash and corrupted one); stop it, flash, then restart it. If the UART link is flaky after a replug, `esptool.py --no-stub` at 115200 is the reliable path.
- **Hook:** the Bash hook blocks commands containing `build/`; use other build dir names (`-B out`, `-B bsmoke`).

## Key Facts

- Full Speed (12 Mbps) is enough: 48 kHz/24-bit stereo = 288 B per 1 ms frame, ~25% of the bus.
- **XMOS feedback at FS:** 4-byte 16.16 (driver also accepts 3-byte 10.14). 0x00300000 at 48 kHz (the miniDSP also reports 0x00300040), every 8 ms (bInterval 4).
- **ESP32-S3 FIFO:** 1024 B physical, 800 usable (200 lines). PERIODIC_OUT bias: iso OUT <= 600 B, IN <= 128 B, control <= 64 B, so capture (~294 B packets) can't run next to playback. ESP-IDF 5.5+/6.x `usb_host_config_t.fifo_settings_custom` with `UAC2_HOST_DUPLEX_FIFO_*` (16/90/94 lines) plus `CONFIG_UAC2_ALLOW_DUPLEX` runs both; not available on 5.4.0-5.4.2 (IDF 5.4.3 backported it, checked 2026-10-29; the harness guard and docs still say 5.5+). Verified on the miniDSP 2x4 HD 2026-10-30 (miniDSP's forum line "playback or recording, not both" is about its single input source, i.e. recording the analog input; its USB capture stream carries the processed outputs).
- **Kconfig:** `USB_HOST_CONTROL_TRANSFER_MAX_SIZE` >= 512 (miniDSP config descriptor is 373 B; repo harness uses 1024). `USB_HOST_HW_BUFFER_BIAS_PERIODIC_OUT=y` for playback.
- **Memory per stream:** the ring buffer storage comes from plain `malloc` (lands in PSRAM when SPIRAM malloc is enabled and it's above the always-internal threshold; only the small control struct uses the internal FreeRTOS heap); URBs are internal DMA memory. minidsp-open's 46 KB ring buffer costs ~6 KB internal RAM during playback.
- **ESP-IDF #17707:** `usb_host_interface_release()` can fail with ESP_ERR_INVALID_STATE while URBs complete; the driver retries.
- **miniDSP 2x4 HD:** VID 0x2752 PID 0x0011, XMOS firmware 1.85 (`bcdDevice 0x0185`, the latest). Self-powered (bmAttributes 0xC0) but needs VBUS to enumerate; because of its own supply, USB replug or ESP32 reset never resets it.
- **miniDSP silent-output state:** an ESP32 hardware reset mid-stream with VBUS up (flashing, the RST button: rst:0x1 over EN) sometimes leaves the miniDSP accepting USB audio normally (48 kHz, clock valid, feedback 48.0, 0 errors, no driver warnings) while it outputs nothing. Intermittent: 2 of 8 controlled EN cuts on 2026-10-30 (minidsp-open on both sides, recovery off; this tree 1 of 5, registry 0.1.3 1 of 3), 0 of 3 mixed-host cuts, plus one confirmed that morning (cut about 8-13 s in). In the stuck state, FU mute/volume, HID source/mute/preset changes, alt and format changes and HID memory 0xF000-0xFFFF all look normal, so the host can't detect it. Not triggers: `esp_restart()` mid-stream (2 of 2 audible, 2026-10-30; it doesn't reset the USB OTG controller), a reset with no stream running, a cable unplug during playback (the miniDSP sees VBUS drop), duplex (30 s, then normal playback), and the 2026-10-29 bisect suspects (capture, rate switching, driver reinstall). New streams, SET_INTERFACE cycles and re-enumeration don't clear it. A miniDSP power cycle does, and so does its HID reboot: minidsp-open's `POST /devices/0/reboot` (3 of 3). USB success does not prove audible output.
- **Listening setup:** the miniDSP drives an SVS PB3000 subwoofer; 1 kHz/440/880 Hz harness tones are inaudible through it. Build the harness with `CONFIG_UAC2_TEST_TONES_SUBWOOFER=y` (30-110 Hz, -12 dBFS). A UMIK-1 measurement mic on this Mac (1 m from the sub) checks audio objectively; minidsp-open's session has a recording pipeline for it. Reboot the miniDSP before listening tests, and whenever a test plays but the mic hears nothing (`POST /devices/0/reboot` through minidsp-open; a power cycle if that fails). The miniDSP is the only UAC2 device available.

## Reference Code (`ref/`, read-only, see `ref/README.md`)

- `espressif-uac1/`: Espressif UAC1 host driver (v1.4.0 source), the architecture forked here.
- `usbx-uac2/ux_class_audio20.h` (MIT) and `cherryusb-uac2/usb_audio.h` (Apache-2.0): UAC2 descriptor structs.
- `minidsp_2x4hd_descriptors.h`: real miniDSP 2x4 HD descriptors (373-byte config, 18-byte device), captured 2026-04-06.

## Serial

Read the ESP32 yourself instead of asking the user to paste output: `serial-mcp` (`list_serial_ports` -> VID 0x1A86, `serial_open` at 115200, `serial_read`/`serial_wait_for`/`serial_read_since`; always `serial_close`), or the `capture-esp32` skill (background logger to `logs/`). Don't toggle DTR/RTS to reset (it lands in download mode); reflash or ask the user to press RST. Flash with Bash (esptool owns the port). Ask the user only for physical actions (replug, power-cycling the miniDSP). Log patterns: boot `rst:0x1`, `I (ms) TAG: msg`, crashes `Guru Meditation Error` (capture the whole backtrace).

## Testing

- `IDF_PATH=~/esp/esp-idf test/host/run.sh`: descriptor parser + packet sizing (ASan/UBSan). `test/host/fuzz.sh`: parser fuzzer (needs libFuzzer: Linux clang or Docker).
- `test/host/driver/run.sh` (ESP-IDF activated): the real driver on the ESP-IDF Linux target against a fake USB Host Library (53 Unity tests, ASan/UBSan, per-test hang watchdog, payload checks, USB contract checks in `tearDown()`; descriptor variants via `fake_usb_*_config()`). On macOS, Unity's `longjmp` would unblock SIGALRM in every thread after a failed assertion and corrupt the POSIX port's scheduler, so the project's CMakeLists switches Unity to `_setjmp`/`_longjmp` on Apple hosts. Time windows that count bus frames must use ticks, not `esp_timer` (a loaded host drops ticks). Runs at CONFIG_FREERTOS_HZ=1000 (the fake's 1 ms bus), so tick-rounding bugs (`pdMS_TO_TICKS(5)` = 0 at 100 Hz) only show on the hardware harness (100 Hz, device task above the driver task). The fake's lock is a binary semaphore on purpose (priority inheritance through a shared mutex tripped FreeRTOS POSIX-port asserts). LeakSanitizer can't find leaks here (task stacks live in heap blocks and keep leaks reachable): use the hardware harness with `sdkconfig.heaptrace.defaults`, which dumps every allocation each connect cycle still holds after teardown.
- CI runs all but the fuzzer.
- Hardware harness (`main/`): profiles `sdkconfig.smoke.defaults` (Test 12 at 20 s), `sdkconfig.duplex.defaults` (IDF 5.5+/6.x FIFO split), `sdkconfig.heaptrace.defaults`; options `CONFIG_UAC2_TEST_TONES_SUBWOOFER`, `CONFIG_UAC2_TEST_RESTART_SEC` (esp_restart() mid-Test 12, once per power-on; add the driver's `CONFIG_UAC2_STOP_STREAMS_ON_RESTART=y` to test that hook). Flash only while no stream runs (a hardware reset mid-stream can leave the miniDSP silent); with the harness flashed there's no HID reboot, so recovery means flashing minidsp-open back.
- Simulator profiles (`simulators/minidsp-2x4hd/sdkconfig.*.defaults`, need a second ESP32-S3): no-feedback (adaptive endpoints; 44.1 kHz must play), channel-only, ep0-hang (control timeout / orphan reclaim), fb-drift (feedback +2 Hz).

## License

MIT
