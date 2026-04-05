# TODO — Remaining Work

## Real Hardware Verification

Everything below requires the real miniDSP 2x4 HD connected to the ESP32-S3 via a powered USB hub.

### Must Do

- [ ] **Full descriptor capture** — capture all 373 bytes of the real config descriptor. Compare against our reconstructed descriptor (257 captured + 116 reconstructed). Fix any mismatches in the simulator.
- [ ] **Streaming verification** — confirm isochronous OUT works, run all 5 tests against real device.
- [ ] **Feedback format** — confirm 3 bytes / 10.14 format at Full Speed (vs 4 bytes / 16.16). Log `actual_num_bytes` from the first few feedback callbacks. The driver handles both formats already.
- [ ] **Simultaneous HID + Audio** — verify that claiming the audio interface doesn't interfere with existing HID control. ESP-IDF supports this on composite devices but hasn't been tested with real XMOS firmware.
- [ ] **ASRC lock time** — measure how long the miniDSP takes to lock onto USB audio. Verify the 2-second preamble is sufficient. Adjust if needed.

### Should Do

- [ ] **Update simulator to match** — once real descriptors are captured, update `simulators/minidsp-2x4hd/main/main.c` descriptor arrays to match exactly. The simulator should be a byte-perfect replica.
- [ ] **Feedback endpoint timing** — verify bInterval=4 (8ms) and that the host correctly adapts packet sizing. Log feedback values for the first few seconds to confirm they converge to the expected nominal (48.0 or 44.1 kHz in 16.16 format).
- [ ] **Sample rate switch on real device** — verify SET_CUR for sample rate works, and that the device's feedback adjusts accordingly.

### Nice to Have

- [ ] **Error recovery testing** — hot-unplug during streaming, cable glitches, rapid start/stop cycles.
- [ ] **JDS Labs Atom DAC+ testing** — secondary UAC2 device to verify the driver is truly generic (note: may fall back to UAC1 at Full Speed).
- [ ] **Capture direction** — not needed for measurement sweeps, but verify the driver's RX path works if we ever need it. Blocked by ESP32-S3 FIFO limitation (128-byte RX FIFO vs 294-byte audio packets).

## minidsp-open Integration

After real hardware is verified:

- [ ] **Add esp-uac2-host as component** — git submodule or local path in minidsp-open's ESP-IDF project.
- [ ] **Sweep generator** — real-time Farina ESS generation on ESP32 (see `~/code/minidsp-open/docs/TODO-esp32-usb-audio.md` for spec).
- [ ] **POST /play endpoint** — HTTP handler that generates sweep and streams via UAC2 driver.
- [ ] **POST /sync endpoint** — NTP-like clock sync for timing alignment.
- [ ] **HID + Audio coexistence** — manage both interfaces from the same USB host client.

## Driver Improvements (Low Priority)

These were identified during code review but are not blocking:

- [ ] **Suspend/resume API** — `uac2_host_stream_suspend()` / `resume()` to avoid URB reallocation during sample rate changes. Currently every stop/start reallocates.
- [ ] **Normalized volume API** — 0-100% convenience function that queries device volume range and maps linearly.
- [ ] **UAC2_HOST_EVENT_DISCONNECTED** — fire the event on device removal (currently only surfaced via USB Host Library client events, not via the UAC2 event callback).
- [ ] **Stream dead notification** — when consecutive error limit is hit and URBs stop, notify the caller so they can restart.
