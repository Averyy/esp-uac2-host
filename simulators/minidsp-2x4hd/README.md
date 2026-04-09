# miniDSP 2x4 HD Simulator

Full-fidelity ESP32-S3 TinyUSB device that replicates the real miniDSP 2x4 HD on USB. Composite device: UAC2 audio + HID vendor control.

## Features

- **UAC2 Audio**: Playback IF1 (alt 1 = 24-bit, alt 2 = 16-bit), explicit feedback EP 0x81
- **HID Control**: IF4 with 64-byte vendor reports, full miniDSP command protocol
- **AC Topology**: Matches real device — playback path (IT2→FU10→OT20) + capture path (IT1→FU11→OT22)
- **HID Commands**: ReadHardwareId, ReadFlash, ReadFloats, SetVolume, SetMute, SetConfig, SetSource, WriteDSP, WriteBiquad, and 11 more
- **EEPROM State**: Preset, source, volume, mute, serial number, mod tokens, DSP ID
- **DSP Parameters**: WriteDSP/WriteBiquad store values, ReadFloats reads them back
- **Fault Injection**: Config switch delay, empty HID response, HID NAK, command timeout (toggle via serial console)
- **No-feedback test build**: optional build variant omits feedback endpoints on both playback alts, switches playback OUT endpoints to adaptive sync, drops the capture streaming alt so the ESP32-S3 simulator can exercise the host's rejection path reliably, and exposes product string `miniDSP 2x4HD [sim no-fb]`
- **Channel-only FU test build**: optional build variant keeps the normal endpoints but removes playback master-channel mute/volume support so only channels 1 and 2 advertise those controls, and exposes product string `miniDSP 2x4HD [sim ch-only]`

## Differences from Real miniDSP

| Feature | Simulator | Real miniDSP |
|---------|-----------|-------------|
| PID | 0x0011 (same) | 0x0011 |
| Config descriptor | 373 bytes (byte-matched) | 373 bytes |
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

That variant intentionally advertises no playback feedback endpoints and only capture alt 0, so it is not byte-identical to the real miniDSP. It exists only to validate the host-side `ESP_ERR_NOT_SUPPORTED` path for fractional-rate playback without feedback. The product string marker lets the host harness assert that this profile really enumerated before trusting the result.

For the playback channel-only feature-unit validation build:

```sh
cd simulators/minidsp-2x4hd
rm -rf build-channel-only sdkconfig.channel-only
idf.py -B build-channel-only -DSDKCONFIG=sdkconfig.channel-only \
  -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.channel-only.defaults" build flash monitor
```

That variant keeps the normal feedback-capable endpoints but changes the playback feature-unit descriptor and control handlers so master channel 0 is unsupported while channels 1 and 2 still support mute/volume. The product string marker lets the host harness assert the expected `mute_ch_map` / `volume_ch_map` shape before running control tests.

## Wiring

Same three-cable setup as `simulators/simple/` — see its README for the diagram.

## Serial Console

Press `h` during operation for fault injection commands:
- `1`: Toggle config switch delay (4.5s audio NAK after SetConfig)
- `2`: Toggle empty HID response
- `3`: NAK next 3 HID writes
- `4`: Toggle command timeout
- `s`: Show simulator state
- `h`: Show help
