# USB Host UAC2 Driver

[![Component Registry](https://components.espressif.com/components/averyy/usb_host_uac2/badge.svg)](https://components.espressif.com/components/averyy/usb_host_uac2) ![maintenance-status](https://img.shields.io/badge/maintenance-actively--developed-brightgreen.svg) ![changelog](https://img.shields.io/badge/Keep_a_Changelog-blue?logo=keepachangelog&logoColor=E05735)

This directory contains an implementation of a USB Audio Class 2.0 (UAC2) host driver implemented on top of the [USB Host Library](https://components.espressif.com/components/espressif/usb).

The UAC2 driver streams playback and capture audio to and from UAC 2.0 devices such as USB DACs, audio interfaces and miniDSP processors, on the ESP32-S3's Full Speed USB OTG port. It handles device discovery, descriptor parsing, clock control, async feedback, alt-setting selection and feature-unit volume/mute.

## Usage

The following steps outline the typical API call pattern of the UAC2 Class Driver:

01. Install the USB Host Library via `usb_host_install()` and keep calling `usb_host_lib_handle_events()` in a task
02. Install the UAC2 driver via `uac2_host_install()`
03. When a UAC2 streaming interface is connected (or was already attached when the driver was installed), the driver event callback is called with the USB device address, interface number and event:
    - `UAC2_HOST_DRIVER_EVENT_TX_CONNECTED`
    - `UAC2_HOST_DRIVER_EVENT_RX_CONNECTED`
04. To open/close an interface by device address and interface number:
    - `uac2_host_device_open()`
    - `uac2_host_device_close()`
05. To get the formats and sample rates the device supports:
    - `uac2_host_device_get_info()`
    - `uac2_host_get_device_alt_param()`
    - `uac2_host_device_get_sample_rate_range()`
06. To start/stop streaming with a specific format (the driver picks the best matching alt setting and sets the clock):
    - `uac2_host_device_start()`
    - `uac2_host_device_stop()`
    - `uac2_host_device_get_stream_info()` reports the format in use
07. To write playback data or read captured data (blocking, through a ring buffer):
    - `uac2_host_device_write()`
    - `uac2_host_device_read()`
    - `uac2_host_device_get_buffer_status()` reports queued and in-flight audio
08. To suspend/resume streaming:
    - `uac2_host_device_suspend()`
    - `uac2_host_device_resume()`
09. To control volume and mute:
    - `uac2_host_device_set_volume()` / `uac2_host_device_set_volume_percent()` / `uac2_host_device_set_volume_all_channels()`
    - `uac2_host_device_set_mute()` / `uac2_host_device_set_mute_all_channels()`
10. After an interface is opened, the device event callback is called with the following events:
    - `UAC2_HOST_DEVICE_EVENT_TX_DONE`
    - `UAC2_HOST_DEVICE_EVENT_RX_DONE`
    - `UAC2_HOST_DEVICE_EVENT_TRANSFER_ERROR`
    - `UAC2_HOST_DEVICE_EVENT_STREAM_ERROR`
    - `UAC2_HOST_DEVICE_EVENT_DISCONNECTED`
11. When `UAC2_HOST_DEVICE_EVENT_DISCONNECTED` is called, close the interface via `uac2_host_device_close()` (retry if it returns an error)
12. The UAC2 driver can be uninstalled via `uac2_host_uninstall()`

> Note: Both callbacks run on the driver's event task and must not block or call driver APIs: hand the address and interface number to another task.

> A device with playback and capture reports each streaming interface separately, and each is opened as its own handle.

See [uac2_host.h](include/usb/uac2_host.h) for the full API.

### Configuration

- `CONFIG_USB_HOST_HW_BUFFER_BIAS_PERIODIC_OUT=y` for reliable playback on ESP32-S3
- `CONFIG_USB_HOST_CONTROL_TRANSFER_MAX_SIZE=512` or more (the miniDSP 2x4 HD's configuration descriptor is 373 bytes); 1024 for composite headsets such as Apple USB-C EarPods
- `CONFIG_USB_HOST_HUBS_SUPPORTED=y` for devices behind a hub, including internal ones
- `CONFIG_UAC2_ALLOW_UNADVERTISED_RATES` (default off): send sample rates outside the device's advertised ranges with a warning instead of refusing them
- `CONFIG_UAC2_ALLOW_DUPLEX` (default off): allow playback and capture at the same time; needs a custom FIFO split at `usb_host_install()` (`usb_host_config_t.fifo_settings_custom` with `UAC2_HOST_DUPLEX_FIFO_*`, ESP-IDF 5.5+/6.x)

### Example code

```c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "usb/usb_host.h"
#include "usb/uac2_host.h"

static uint8_t s_addr;
static uint8_t s_iface_num;
static TaskHandle_t s_playback_task;

static void playback_task(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uac2_host_device_handle_t dev = NULL;
        uac2_host_device_config_t open_cfg = {
            .addr = s_addr,
            .iface_num = s_iface_num,
            .buffer_size = 0,
            .buffer_threshold = 0,
            .callback = NULL,
            .callback_arg = NULL,
        };

        if (uac2_host_device_open(&open_cfg, &dev) != ESP_OK) {
            continue;
        }

        uac2_host_stream_config_t stream_cfg = {
            .channels = 2,
            .bit_resolution = 24,
            .sample_freq = 48000,
            .flags = 0,
        };

        if (uac2_host_device_start(dev, &stream_cfg) != ESP_OK) {
            uac2_host_device_close(dev);
            continue;
        }

        // Application can now call uac2_host_device_write()
    }
}

static void driver_event_cb(uint8_t addr, uint8_t iface_num,
                            const uac2_host_driver_event_t event, void *arg)
{
    if (event != UAC2_HOST_DRIVER_EVENT_TX_CONNECTED) {
        return;
    }

    s_addr = addr;
    s_iface_num = iface_num;
    xTaskNotifyGive(s_playback_task);
}

static void usb_host_lib_task(void *arg)
{
    usb_host_config_t host_cfg = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_cfg));
    xTaskNotifyGive((TaskHandle_t)arg);

    // The USB Host Library needs its own event loop; the UAC2 driver's
    // background task only handles its client events.
    for (;;) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
    }
}

void app_main(void)
{
    xTaskCreate(usb_host_lib_task, "usb_host", 4096, xTaskGetCurrentTaskHandle(), 21, NULL);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Create the task the driver callback notifies before installing the
    // driver: an already-attached device is reported as soon as it starts.
    xTaskCreate(playback_task, "uac2_playback", 4096, NULL, 19, &s_playback_task);

    uac2_host_driver_config_t driver_cfg = {
        .create_background_task = true,
        .task_priority = 20,
        .stack_size = 6144,
        .core_id = 0,
        .callback = driver_event_cb,
        .callback_arg = NULL,
    };
    ESP_ERROR_CHECK(uac2_host_install(&driver_cfg));
}
```

## Installation

```sh
idf.py add-dependency "averyy/usb_host_uac2^0.1.3"
```

To track unreleased changes, depend on the repository instead:

```yaml
dependencies:
  averyy/usb_host_uac2:
    git: https://github.com/Averyy/esp-uac2-host.git
    path: components/usb_host_uac2
```

## Examples

- [basic_playback](examples/basic_playback): waits for a UAC2 playback interface, opens it and streams a 48 kHz / 24-bit stereo sine wave.

## Supported Devices

- UAC 2.0 devices with a single clock domain, on ESP32-S3 at Full Speed. ESP-IDF 5.4 or newer (tested on 5.4.0, 6.0.3 and 6.1).
- Tested with a miniDSP 2x4 HD (XMOS). Reports from other devices are welcome, working or not: [open an issue](https://github.com/Averyy/esp-uac2-host/issues) with the device's VID:PID, your ESP-IDF version and the driver's log.
- UAC 1.0 devices: use Espressif's [`usb_host_uac`](https://components.espressif.com/components/espressif/usb_host_uac) alongside this driver. Its connect callback also reports UAC2 interfaces, so open only UAC1 devices from it.

## Known issues

- Playback and capture at the same time needs `CONFIG_UAC2_ALLOW_DUPLEX` plus a custom FIFO split; without it the driver refuses a stream while one in the other direction runs (ESP-IDF's FIFO presets leave 128 bytes for incoming packets). The split lowers the largest playback packet from 600 to 360 bytes.
- Playback and capture together is unverified on the miniDSP 2x4 HD: once, after a test session that included it, the device's output stayed silent until it was power-cycled. That didn't reproduce, including with capture tested on its own.
- `uac2_host_device_set_sample_rate()` returns `ESP_ERR_INVALID_STATE` while any stream on the device is started or suspended (the clock is shared; `uac2_host_device_start()` applies the stream's rate).
- 44.1 kHz on an asynchronous playback endpoint needs its feedback endpoint; implicit feedback isn't supported.
- Full-speed audio data endpoints must use `bInterval = 1`, and only a device's first 8 streaming alt settings are used.
- With `create_background_task = false`, keep calling `uac2_host_handle_events()` while the driver is in use.

## Acknowledgements

The driver's structure (install/uninstall lifecycle, device discovery, reference-counted interfaces, ring-buffered isochronous streaming) follows Espressif's UAC1 driver, [`usb_host_uac`](https://github.com/espressif/esp-usb/tree/master/host/class/uac/usb_host_uac) (Apache-2.0).
