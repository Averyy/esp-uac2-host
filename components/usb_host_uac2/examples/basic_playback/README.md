| Supported Targets | ESP32-S3 |
| ----------------- | -------- |

# UAC2 basic playback example

## How to use this example

This example shows the basic use of the USB Host UAC2 driver: it waits for a UAC2 playback interface, opens it, starts a 48 kHz / 24-bit stereo stream and writes a continuous 1 kHz sine wave until the device is unplugged. Plug the device back in to start again.

It demonstrates:

- installing the USB Host Library and the UAC2 driver with its background event task
- reacting to `UAC2_HOST_DRIVER_EVENT_TX_CONNECTED` by handing the interface to a playback task, outside the driver callback (the latest device plugged in while the previous one is still closing is played next)
- streaming PCM with `uac2_host_device_write()`
- closing the interface after a disconnect, retrying until the close succeeds, so the device can be reopened when it comes back

### Hardware Required

- An ESP32-S3 board with USB OTG host support. On the ESP32-S3-DevKitC-1, bridge the `USB-OTG` solder pads on the back so the OTG port supplies 5 V; without it nothing enumerates, even self-powered devices.
- A UAC 2.0 playback device with a 48 kHz / 24-bit stereo format carried in 3-byte samples (USB DAC, audio interface, miniDSP processor)
- A USB cable or adapter for the device

### Configure the project

`sdkconfig.defaults` sets what the driver needs:

- `CONFIG_USB_HOST_HW_BUFFER_BIAS_PERIODIC_OUT=y`
- `CONFIG_USB_HOST_CONTROL_TRANSFER_MAX_SIZE=512`: raise it to 1024 for composite headsets such as Apple USB-C EarPods, and enable `CONFIG_USB_HOST_HUBS_SUPPORTED` for devices behind a hub

`main/idf_component.yml` depends on `averyy/usb_host_uac2` with an `override_path`, so a build inside the driver's repository uses the local copy, while a project created from the registry fetches the published component:

```sh
idf.py create-project-from-example "averyy/usb_host_uac2^0.1.3:basic_playback"
```

### Build and Flash

```sh
idf.py set-target esp32s3
idf.py build
idf.py -p PORT flash monitor
```

(To exit the serial monitor, type `Ctrl-]`.)

Builds with ESP-IDF 5.4 or newer. On ESP-IDF 6.x the USB Host Library comes from the `espressif/usb` component, which the driver's manifest adds automatically.

## Example Output

```
I (318) uac2_basic_playback: Installing USB Host Library
I (348) uac2_basic_playback: UAC2 basic playback example ready, waiting for device
I (938) uac2_basic_playback: TX interface connected: addr=1 iface=1
I (948) uac2_basic_playback: Opening playback interface addr=1 iface=1
I (1168) uac2_basic_playback: Playback started: 48000 Hz, 24-bit, 2 channels
...
I (15302) uac2_basic_playback: Stopping playback
I (15312) uac2_basic_playback: All USB devices freed
```

## Troubleshooting

- **`device_start failed: ESP_ERR_NOT_FOUND`**: the device has no 48 kHz / 24-bit stereo playback format. Change `PLAYBACK_BIT_RESOLUTION` (and the sine writer) or `PLAYBACK_SAMPLE_RATE` in `main/main.c` to a format the device lists; `uac2_host_device_print_info()` shows them.
- **`device_start failed: ESP_ERR_NOT_SUPPORTED`**: the device doesn't advertise 48 kHz, or its endpoints don't fit the ESP32-S3 host.
- **`Unsupported sample layout: 24-bit samples in 4-byte slots`**: the device carries 24-bit audio in 4-byte slots, and this example only writes packed 3-byte samples. `uac2_host_device_get_stream_info()` reports the layout to write.
- **No `TX interface connected` line**: check VBUS (above), and that the device is UAC 2.0. UAC 1.0 devices need Espressif's `usb_host_uac` driver.
