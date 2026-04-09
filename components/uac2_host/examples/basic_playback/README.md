| Supported Targets | ESP32-S3 |
| ----------------- | -------- |

# UAC2 Basic Playback

Minimal example for `usb_host_uac2`. It waits for a UAC2 playback interface, opens it, starts a 48 kHz stereo stream, and continuously writes a 1 kHz sine wave.

## What it demonstrates

- Installing the ESP-IDF USB Host Library
- Installing `usb_host_uac2` with background event handling
- Reacting to `TX_CONNECTED` driver events
- Deferring interface open/start work to a playback task outside the driver callback
- Starting a 48 kHz / 24-bit / stereo stream
- Streaming PCM data with `uac2_host_device_write()`
- Cleaning up after disconnect

## Hardware Required

- ESP32-S3 board with USB OTG host support
- UAC2 playback device
- USB cable/adapter for the target device

### Notes

- This example targets ESP32-S3 only.
- The driver requires `CONFIG_USB_HOST_HW_BUFFER_BIAS_PERIODICOUT=y`.
- `CONFIG_USB_HOST_CONTROL_TRANSFER_MAX_SIZE` must be at least `512` for devices with larger descriptors such as miniDSP.
- On ESP32-S3, this example is playback-only. Simultaneous TX+RX is not supported by the hardware FIFO limits.

## Build and Flash

From this directory:

```sh
idf.py set-target esp32s3
idf.py build
idf.py -p PORT flash monitor
```

The example uses a local `path` dependency in `main/idf_component.yml` so it builds directly from this repository checkout. After the component is published, this can be switched to a registry dependency.

## Expected Behavior

After boot:

- The ESP32-S3 waits for a UAC2 device.
- When a playback interface appears, the example signals a playback task, which opens it and starts streaming.
- A 1 kHz tone continues until the device is unplugged.
- Reconnect the device to start again.
