# miniDSP 2x4 HD Simulator

Full-fidelity ESP32-S3 TinyUSB device that replicates the real miniDSP 2x4 HD on USB. Composite device: UAC2 audio + HID vendor control.

## Features

- **UAC2 Audio**: Playback IF1 (alt 1 = 24-bit, alt 2 = 16-bit), explicit feedback EP 0x81
- **HID Control**: IF4 with 64-byte vendor reports, full miniDSP command protocol
- **AC Topology**: Matches real device — playback path (IT2→FU10→OT20) + capture path (IT1→FU11→OT22)
- **HID Commands**: ReadHardwareId, ReadFlash, ReadFloats, SetVolume, SetMute, SetConfig, SetSource, WriteDSP, WriteBiquad, and 11 more
- **EEPROM State**: Preset, source, volume, mute, serial number, mod tokens, DSP ID
- **DSP Parameters**: WriteDSP/WriteBiquad store values, ReadFloats reads them back
- **Fault Injection**: Config switch delay, empty HID response, dropped HID commands, command timeout (toggle via serial console). These act at the application level: packets are still ACKed on the bus, so none of them produces a USB NAK
- **No-feedback test build**: optional build variant omits feedback endpoints on both playback alts, switches playback OUT endpoints to adaptive sync, drops the capture streaming alt, and exposes product string `miniDSP 2x4HD [sim no-fb]`. The host must play 44.1 kHz on it (adaptive endpoints need no feedback). The harness's 44.1 kHz streams are too short for the `PACKET SIZING CHECK` below, so on this rig only the 48 kHz streams get checked (nominal 48.0000)
- **Channel-only FU test build**: optional build variant keeps the normal endpoints but removes playback master-channel mute/volume support so only channels 1 and 2 advertise those controls, and exposes product string `miniDSP 2x4HD [sim ch-only]`
- **Control-hang test build** (`[sim ep0-hang]`): default descriptors, but the first GET CUR clock-valid request after boot is never answered and the simulator soft-disconnects 12 s later (one-shot per boot). Drives the host's control-timeout / orphaned-URB path
- **Feedback drift test build** (`[sim fb-drift]`): default descriptors, but feedback reports 2 Hz above nominal (~42 ppm fast clock)
- **Packet sizing check** (all builds): every 10 s of playback the simulator logs `PACKET SIZING CHECK PASS/FAIL`, comparing the host's average samples per packet (over 5000+ packets, ±0.0005) with the rate it asks for: the reported feedback, or the nominal rate without a feedback endpoint

## Differences from Real miniDSP

| Feature | Simulator | Real miniDSP |
|---------|-----------|-------------|
| PID | 0x0011 (same) | 0x0011 |
| Config descriptor | 373 bytes, byte-identical (checked at boot) | 373 bytes |
| bNumConfigurations | 1 | 2 (second configuration not captured; ESP-IDF enumerates configuration 1) |
| Interfaces | AC, AS playback, AS capture, DFU stub, HID | Same |
| Feedback | 4-byte 16.16 explicit feedback, driven from active sample rate | 4-byte 16.16 (hardware PLL) |
| Audio data reception | Functional | Functional |
| HID report descriptor | 28 bytes (captured from real device) | 28 bytes |

## Building

```sh
cd simulators/minidsp-2x4hd
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/cu.usbmodem* flash monitor
```

For the reduced no-feedback validation build:

```sh
cd simulators/minidsp-2x4hd
rm -rf build-no-feedback sdkconfig.no-feedback
idf.py -B build-no-feedback -DSDKCONFIG=sdkconfig.no-feedback \
  -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.no-feedback.defaults" build flash monitor
```

That variant intentionally advertises no playback feedback endpoints and only capture alt 0, so it is not byte-identical to the real miniDSP. It validates host playback on adaptive endpoints without feedback: Tests 4 and 10 must stream 44.1 kHz on it and Test 7 must be able to start it at 16 bits (before driver v0.1.3 they expected `ESP_ERR_NOT_SUPPORTED`, which the driver now returns only for asynchronous endpoints without feedback; the Linux-target driver tests cover that case). The product string marker lets the host harness assert that this profile really enumerated before trusting the result.

For the playback channel-only feature-unit validation build:

```sh
cd simulators/minidsp-2x4hd
rm -rf build-channel-only sdkconfig.channel-only
idf.py -B build-channel-only -DSDKCONFIG=sdkconfig.channel-only \
  -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.channel-only.defaults" build flash monitor
```

That variant keeps the normal feedback-capable endpoints but changes the playback feature-unit descriptor and control handlers so master channel 0 is unsupported while channels 1 and 2 still support mute/volume. The product string marker lets the host harness assert the expected `mute_ch_map` / `volume_ch_map` shape before running control tests.

For the control-hang and feedback-drift builds, use the same pattern with `ep0-hang` or `fb-drift`:

```sh
cd simulators/minidsp-2x4hd
idf.py -B build-ep0-hang -DSDKCONFIG=sdkconfig.ep0-hang \
  -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.ep0-hang.defaults" build flash monitor
```

With `[sim ep0-hang]` the host harness runs its control-timeout live check on the first connection: the clock-valid request must time out (5 s), the next request must be refused while it is still pending, close must succeed, reopening must be refused, and after the simulator's soft disconnect the host must free the device (ALL_FREE) and see it re-enumerate. The fault is then disarmed, so the reconnection runs the full suite. With `[sim fb-drift]` the host runs the normal suite and the simulator's `PACKET SIZING CHECK` lines must read about 48.0020 samples/packet during the 48 kHz streams, not the nominal 48.0000.

## Wiring

Same three-cable setup as `simulators/simple/` — see its README for the diagram.

## Serial Console

Press `h` during operation for fault injection commands:
- `1`: Toggle config switch delay (discard received audio for 4.5 s after a HID SetConfig command; the host harness never sends one)
- `2`: Toggle empty HID response
- `3`: Drop the next 3 HID commands (received, but never answered)
- `4`: Toggle command timeout
- `s`: Show simulator state
- `h`: Show help
