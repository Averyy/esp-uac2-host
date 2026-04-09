| Supported Targets | ESP32-S3 |
| ----------------- | -------- |

# USB Host UAC2

USB Audio Class 2.0 host driver for ESP-IDF.

This component provides a UAC2 host driver for ESP32-S3. It handles device discovery, descriptor parsing, clock control, isochronous playback/capture streams, async feedback, and feature-unit volume/mute controls. Current public support is scoped to single-clock UAC2 devices. Real miniDSP validation currently includes the 12-test harness, live suspend/resume, duplex-guard rejection, and active hot-unplug/replug recovery.

## Features

- Espressif-style class driver lifecycle: `uac2_host_install()` / `uac2_host_uninstall()`
- Internal UAC2 device discovery with per-interface connect callbacks
- UAC2 descriptor parsing for clock sources, selectors, multipliers, terminals, feature units, and audio streaming interfaces
- Sample-rate query/set through UAC2 clock entities
- Isochronous TX and RX streaming with ring buffers
- Async feedback endpoint handling for adaptive playback packet sizing
- Feature-unit mute and volume control, including range queries and percent helpers
- Suspend/resume without reallocating stream resources
- Disconnect-aware teardown and interface reference counting

## Requirements

- ESP-IDF `>=5.4`
- Target: `esp32s3`
- USB Host Library installed by the application before `uac2_host_install()`
- `CONFIG_USB_HOST_CONTROL_TRANSFER_MAX_SIZE >= 512` for larger configuration descriptors
- `CONFIG_USB_HOST_HW_BUFFER_BIAS_PERIODIC_OUT=y` for reliable playback on ESP32-S3

## Install

After publication to the registry:

```sh
idf.py add-dependency "averyy/usb_host_uac2^0.1.1"
```

For local development before publication, use a `path` dependency in `idf_component.yml`:

```yaml
dependencies:
  idf: ">=5.4"
  uac2_host:
    path: ../../components/uac2_host
```

## Quick Start

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

void app_main(void)
{
    usb_host_config_t host_cfg = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    usb_host_install(&host_cfg);

    uac2_host_driver_config_t driver_cfg = {
        .create_background_task = true,
        .task_priority = 20,
        .stack_size = 6144,
        .core_id = 0,
        .callback = driver_event_cb,
        .callback_arg = NULL,
    };
    uac2_host_install(&driver_cfg);

    xTaskCreate(playback_task, "uac2_playback", 4096, NULL, 21, &s_playback_task);
}
```

## Main API

- Driver lifecycle: `uac2_host_install()`, `uac2_host_uninstall()`, `uac2_host_handle_events()`
- Device lifecycle: `uac2_host_device_open()`, `uac2_host_device_close()`, `uac2_host_device_get_info()`
- Streaming: `uac2_host_device_start()`, `uac2_host_device_stop()`, `uac2_host_device_write()`, `uac2_host_device_read()`, `uac2_host_device_suspend()`, `uac2_host_device_resume()`
- Clock control: `uac2_host_device_get_sample_rate()`, `uac2_host_device_set_sample_rate()`, `uac2_host_device_get_sample_rate_range()`, `uac2_host_device_get_clock_valid()`
- Feature unit: `uac2_host_device_set_volume()`, `uac2_host_device_get_volume()`, `uac2_host_device_set_mute()`, `uac2_host_device_get_mute()`

See [uac2_host.h](include/usb/uac2_host.h) for the full API.

## Example

- [basic_playback](examples/basic_playback) streams a 48 kHz / 24-bit stereo sine wave to a connected UAC2 playback device.

## Known Limitations

- ESP32-S3 cannot do simultaneous playback and capture for typical UAC2 packet sizes because of USB FIFO limits. The driver rejects opposite-direction stream activation at runtime.
- Public support is currently limited to single-clock UAC2 devices.
- Active miniDSP hot-unplug/replug is verified working on ESP-IDF v5.4 with the current teardown path.
- This component is UAC2-only. It does not fall back to UAC1.

## Tested Hardware

- ESP32-S3-DevKitC-1
- miniDSP 2x4 HD
- ESP32-S3 UAC2 simulator in this repository
- Real-device reruns on the miniDSP 2x4 HD also validate suspend/resume, duplex-guard rejection, and active hot-unplug/replug recovery

## License

MIT
