# esp-uac2-host

USB Audio Class 2.0 host driver for ESP32-S3. ESP-IDF component (C).

Targets the miniDSP 2x4 HD but should work with any UAC2 device (XMOS-based DACs, audio interfaces, etc.).

## Status

Work in progress. USB Host init, UAC2 descriptor parsing, control requests, and streaming infrastructure built and verified on ESP32-S3. UAC2 control requests (sample rate, clock validity, volume, mute) tested end-to-end against a UAC2 simulator device. Isochronous audio streaming pending live hardware test.

## Hardware

- **MCU:** ESP32-S3-DevKitC-1
- **USB:** Full Speed (12 Mbps) — sufficient for 48kHz/24-bit/stereo
- **Test device:** miniDSP 2x4 HD (XMOS XU216, UAC2)

## Building

Requires [ESP-IDF v5.4](https://docs.espressif.com/projects/esp-idf/en/v5.4/).

```sh
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/cu.usbmodem* flash monitor
```

## License

MIT
