# esp-uac2-host

USB Audio Class 2.0 host driver for ESP32. ESP-IDF component (C). Enables ESP32 to stream audio to/from UAC2 devices like DACs, audio interfaces, and miniDSP hardware.

## Project State

**v0.1.1 — validated on current hardware, still under active testing.** Espressif class driver pattern: `uac2_host_install()`/`uac2_host_uninstall()` with internal device discovery, linked lists, reference counting. Two-level struct split: `uac2_device_t` (physical) + `uac2_iface_t` (per-interface). The 12-test boot harness passes against both the ESP32-to-ESP32 simulator and the real miniDSP 2x4 HD; recent live miniDSP reruns also pass suspend/resume, duplex-guard, and active hot-unplug/replug recovery checks. Builds clean with `-Werror -Wextra`. Downstream integration (sweep generator, `POST /play`) lives in minidsp-open. See `PROGRESS.md` for detailed status.

## Related Projects

- **minidsp-open** (`~/code/minidsp-open`) — primary consumer. Will integrate this driver via Rust FFI for measurement sweeps. See `docs/TODO-esp32-usb-audio.md`.
- **open-sub-optimizer** (`~/code/open-sub-optimizer`) — calls minidsp-open's `POST /play` API. Doesn't interact with this driver directly.
- **Espressif `usb_host_uac`** (`github.com/espressif/esp-usb`) — UAC1 host driver v1.3.3 (Apache-2.0). Architecture was forked for this UAC2 driver. Reference copy in `ref/espressif-uac1/`.
- **USBX** (`github.com/eclipse-threadx/usbx`) — MIT. `ux_class_audio20.h` has UAC2 descriptor structs. Used as protocol reference during development.
- **CherryUSB** (`github.com/cherry-embedded/CherryUSB`) — Apache-2.0. `usb_audio.h` has UAC1+UAC2 structs. Used as protocol reference during development.

## Tech Stack

C (ESP-IDF component). Targets ESP32-S3. Built on top of ESP-IDF USB Host Library. No external dependencies beyond ESP-IDF.

- **ESP-IDF:** v5.4 (`~/esp/esp-idf`). System `python3` is 3.9 (Apple) but ESP-IDF needs Homebrew Python. Always activate with: `export PATH="/opt/homebrew/bin:$PATH" && . ~/esp/esp-idf/export.sh`
- **Board:** ESP32-S3-DevKitC-1. Two USB-C ports:
  - **Right (UART):** CH340 USB-to-UART bridge (VID 0x1A86). Use for programming + serial monitor. Shows as `/dev/cu.usbmodem*`.
  - **Left (USB):** Built-in USB-Serial/JTAG on GPIO 19/20 (VID 0x303A). **This is also the USB OTG port** — when using USB Host mode (miniDSP), this port is unavailable for serial.
- **VBUS:** The DevKitC-1 does not supply 5V to OTG port by default. **Bridge the `USB-OTG` solder pads** on the back of the board to route 5V from the power rail to the OTG port VBUS pin. Without this, USB devices won't enumerate even if self-powered.

## Key Facts

- ESP32-S3 USB OTG is Full Speed (12 Mbps). Bandwidth is fine for audio — 48kHz/24-bit/stereo uses ~25% of available bus capacity (288 bytes per 1ms frame).
- UAC2 at Full Speed is valid per spec. XMOS default is `XUA_AUDIO_CLASS_FS=2` (UAC2 at FS). The miniDSP 2x4 HD presents UAC2 descriptors to a Full Speed host (proven via Arduino USB Host Shield descriptor dump).
- UAC2 differs from UAC1 in: descriptor format, clock management (explicit clock source entities), control request layout (CUR/RANGE vs SET_CUR/GET_CUR), and `bInterfaceProtocol` (0x20 vs 0x00).
- The isochronous transfer layer in ESP-IDF works for audio (proven by `usb_host_uac` UAC1 driver and `esp32-rtp` community project, 78 stars).
- Memory footprint estimate: ~40-50 KB internal SRAM (driver + ring buffers + URBs). ESP32-S3 has ~200-280 KB free after WiFi.
- The JDS Labs Atom DAC+ has confirmed UAC1 fallback at Full Speed (`XUA_AUDIO_CLASS_FS=1`). Useful as a UAC1 comparison/baseline device.
- **XMOS feedback at FS**: 4 bytes, 16.16 format (confirmed by real miniDSP 2x4 HD testing, 2026-04-06). Value at 48kHz: 0x00300000 (48.0000 samples/frame). Locks immediately, drift <0.002%. Arrives every 8ms (bInterval=4). Previous assumption of 3-byte 10.14 was incorrect — driver handles both formats.
- **ESP32-S3 FIFO limitation**: Total 1024 bytes. With PERIODIC_OUT bias: PTX=600 (iso OUT), RX=128 (iso IN), NPTX=64. **Cannot do simultaneous TX+RX** — audio capture packets (~294 bytes) exceed the 128-byte RX FIFO.
- **ESP-IDF bug #17707**: `usb_host_interface_release()` can fail with ESP_ERR_INVALID_STATE when URBs are in-flight. Driver has retry logic.
- **miniDSP hot-unplug status (2026-04-07)**: Active playback unplug/replug is verified working on ESP-IDF v5.4 with the current driver teardown path. Repeated 3-second unplug/replug cycles and an immediate unplug/replug stress test completed without crash, with `ALL_FREE` observed before reconnect.
- miniDSP is self-powered (bmAttributes=0xC0, bMaxPower=0) but still needs VBUS present on the bus to enumerate.

## Kconfig Notes

- `USB_HOST_CONTROL_TRANSFER_MAX_SIZE`: default 256, but miniDSP config descriptor is 373 bytes. **Must increase to at least 512.**
- `USB_HOST_HW_BUFFER_BIAS`: set to `PERIODIC_OUT` for audio playback (biases DWC_OTG FIFO toward isochronous OUT). Cannot use for capture — RX FIFO only 128 bytes.

## Reference Code

Local copies in `ref/` (read-only, not compiled). See `ref/README.md` for sources.

- `ref/espressif-uac1/` — Espressif UAC1 host driver v1.3.3 (Apache-2.0). Architecture to fork. ~4,000 lines.
- `ref/usbx-uac2/ux_class_audio20.h` — USBX UAC2 descriptor structs (MIT). The gold standard. 1694 lines.
- `ref/cherryusb-uac2/usb_audio.h` — CherryUSB UAC1+UAC2 structs (Apache-2.0). Standard C types. 1347 lines.
- `ref/minidsp_2x4hd_descriptors.h` — Real miniDSP 2x4 HD descriptor bytes captured via ESP32-S3 USB Host (2026-04-06). Full 373-byte config descriptor + 18-byte device descriptor.

## ESP32 Serial (serial-mcp)

Claude Code has direct serial access to the ESP32 via `serial-mcp` tools. Use these instead of asking the user to copy-paste terminal output.

**Typical workflow:**
1. `serial_open` — connect to the UART port (right side, CH340). Look for VID `0x1A86` in `list_serial_ports`. 115200 baud.
2. `serial_command` — send commands, read responses (e.g., trigger a descriptor dump, restart the app)
3. `serial_read` / `serial_wait_for` — monitor boot logs, wait for specific output like "waiting for device" or error messages
4. `serial_read_since` — non-destructive replay of recent output (useful after a crash or unexpected behavior)
5. `serial_set_signals` — toggle DTR/RTS to reset the ESP32 or enter bootloader mode

**When to use serial vs asking the user:**
- Reading logs, descriptor dumps, error output → use serial tools directly
- Flashing firmware (`idf.py flash`) → use Bash (esptool handles its own serial)
- Physically connecting/disconnecting USB devices → ask the user

**ESP32 monitor output patterns:**
- Boot log starts with `rst:0x1` or `rst:0x3` lines
- ESP-IDF log format: `I (timestamp) TAG: message` (I=info, W=warn, E=error)
- USB Host Library events: look for `USB_HOST` or `HCD` tags
- Crash dumps start with `Guru Meditation Error` — capture the full backtrace

**Port selection:** Always use the UART port (right, CH340, VID `0x1A86`) for serial-mcp. The USB port (left, VID `0x303A`) shares GPIO 19/20 with USB OTG — it won't be available when the miniDSP is connected.

## Testing

Primary test device: miniDSP 2x4 HD (XMOS XU216, UAC2, VID 0x2752 PID 0x0011).
Secondary: JDS Labs Atom DAC+ (XMOS, UAC2 + confirmed UAC1 fallback at FS).
Baseline: any cheap USB sound card (UAC1).

## License

MIT
