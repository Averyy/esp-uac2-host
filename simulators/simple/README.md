# UAC2 Test Device — miniDSP 2x4 HD Simulator

ESP32-S3 firmware that presents as a UAC2 audio device, mimicking the miniDSP 2x4 HD's descriptor layout and control request behavior. Used to test the `esp-uac2-host` driver without needing the real hardware.

## What it does

- Presents UAC2 descriptors matching the miniDSP 2x4 HD topology:
  - Clock Source ID=41 (internal programmable)
  - Clock Selector ID=40 (1 input from source 41)
  - Input Terminal ID=2 (USB Streaming, 2ch stereo)
  - Feature Unit ID=10 (mute + volume, master + 2ch)
  - Output Terminal ID=20 (Speaker)
- Playback endpoint: EP 0x01 OUT, async isochronous, MPS=294, 24-bit/2ch
- Feedback endpoint: EP 0x81 IN, MPS=4, interval=4 (every 8ms)
- Responds to UAC2 CUR/RANGE requests:
  - Sample rate GET/SET on Clock Source (supports 44.1kHz, 48kHz)
  - Clock validity
  - Clock selector input selection
  - Mute GET/SET on all channels
  - Volume GET/SET with range (-90dB to 0dB, 1dB steps)
- Accepts isochronous audio data and logs frame count/byte totals
- Sends feedback at the configured sample rate

## Hardware

Requires an ESP32-S3 board (same as the host). Uses the left USB-C port (OTG) for the USB device connection and the right USB-C port (UART) for power and serial monitoring.

## Building

```sh
cd simulators/simple
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/cu.usbmodem* flash monitor
```

## Wiring Diagram

Three USB-C cables connect two ESP32-S3-DevKitC-1 boards to each other and to a Mac/PC.

```
                        Mac / PC
                 ┌─────────────────────┐
                 │                     │
                 │  USB-A    USB-A     │
                 └───┬─────────┬───────┘
          Cable 1    │         │    Cable 2
       (flash/serial)│         │(flash/serial)
                     │         │
                     ▼         ▼
              RIGHT port    RIGHT port
              (UART/CH340)  (UART/CH340)
            ┌──────────┐  ┌──────────┐
            │          │  │          │
            │   HOST   │  │SIMULATOR │
            │  ESP32   │  │  ESP32   │
            │          │  │          │
            └──────────┘  └──────────┘
              LEFT port     LEFT port
              (USB OTG)     (USB OTG)
                  │             │
                  └──────┬──────┘
                         │
                      Cable 3
                   (USB audio data)
```

**Cable 1:** Mac/PC USB port  -->  Host ESP32 **RIGHT** port (UART)
**Cable 2:** Mac/PC USB port  -->  Simulator ESP32 **RIGHT** port (UART)
**Cable 3:** Host ESP32 **LEFT** port (OTG)  <-->  Simulator ESP32 **LEFT** port (OTG)

**Port identification (each board has two USB-C ports):**
- **RIGHT port** = CH340 UART bridge (VID `0x1A86`). For flashing firmware and serial monitor.
- **LEFT port** = ESP32-S3 native USB on GPIO 19/20. For USB OTG data (host ↔ device).

## Testing with the host driver

1. Flash this firmware to the "device" ESP32-S3
2. Flash the main `esp-uac2-host` firmware to the "host" ESP32-S3
3. Connect both boards' UART ports to the Mac (for power + serial)
4. Connect a USB-C cable between both boards' OTG ports (left side)
5. Monitor the host board's serial output — it should enumerate the device, query clocks, start streaming, and log "Streamed N sec" every second

## Differences from real miniDSP 2x4 HD

| Feature | Simulator | Real miniDSP |
|---------|-----------|-------------|
| PID | 0x9999 | 0x0011 |
| VID | 0x2752 (same) | 0x2752 |
| Sample rates | 44.1k, 48k | 44.1k, 48k, 88.2k, 96k (FS) |
| Alt settings | Alt 1 (24-bit) | Alt 1 (24-bit), Alt 2 (16-bit) |
| Capture interface | Not present | IF2, 2ch 24-bit, EP 0x82 IN |
| HID interface | Not present | IF3, 64-byte reports |
| DFU interface | Not present | IF4, runtime mode |
| Feedback | Software-generated at configured rate | Hardware PLL-based |
