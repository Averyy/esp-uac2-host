# miniDSP 2x4 HD Simulator

Full-fidelity ESP32-S3 TinyUSB device that replicates the real miniDSP 2x4 HD on USB. Composite device: UAC2 audio + HID vendor control.

## Features

- **UAC2 Audio**: Playback IF1 (alt 1 = 24-bit, alt 2 = 16-bit), feedback EP 0x81
- **HID Control**: IF2 with 64-byte vendor reports, full miniDSP command protocol
- **AC Topology**: Matches real device — playback path (IT2→FU10→OT20) + capture path (IT1→FU11→OT22)
- **HID Commands**: ReadHardwareId, ReadFlash, ReadFloats, SetVolume, SetMute, SetConfig, SetSource, WriteDSP, WriteBiquad, and 11 more
- **EEPROM State**: Preset, source, volume, mute, serial number, mod tokens, DSP ID
- **DSP Parameters**: WriteDSP/WriteBiquad store values, ReadFloats reads them back
- **Fault Injection**: Config switch delay, empty HID response, HID NAK, command timeout (toggle via serial console)

## Differences from Real miniDSP

| Feature | Simulator | Real miniDSP |
|---------|-----------|-------------|
| PID | 0x0011 (same) | 0x0011 |
| Capture AS interface | Not present (DWC2 FIFO limit) | IF2, EP 0x82 IN |
| Capture AC entities | Present (IT1, FU11, OT22) | Present |
| DFU interface | Not present | IF4 |
| Config descriptor | 300 bytes | 373 bytes |
| Audio data reception | Not functional (SET_INTERFACE crash) | Functional |

## Building

```sh
cd simulators/minidsp-2x4hd
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/cu.usbmodem* flash monitor
```

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

## Known Issues

- **SET_INTERFACE crash**: When the host sends a SET_INTERFACE standard request, TinyUSB's DWC2 driver aborts during isochronous endpoint activation. This is currently worked around by not sending SET_INTERFACE from the host driver. The host tests pass because isochronous OUT transfers on Full Speed USB are fire-and-forget.
