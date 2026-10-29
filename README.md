# esp-uac2-host

> [!NOTE]
> Verified on an ESP32-S3 with a miniDSP 2x4 HD, the UAC2 device I have on hand. Other UAC2 devices should work too. If yours doesn't, or you'd like a feature or a change, please [open an issue](https://github.com/Averyy/esp-uac2-host/issues). Pull requests are welcome.

USB Audio Class 2.0 host driver for ESP32-S3, packaged as an ESP-IDF component (C). It streams audio to and from UAC2 devices such as DACs, audio interfaces and miniDSP processors over the S3's USB OTG port.

I built it to play measurement sweeps through a miniDSP 2x4 HD from an ESP32 for automated subwoofer optimization, so that's the path with the most testing behind it.

## Status

**v0.1.3** is verified on a miniDSP 2x4 HD under ESP-IDF 5.4, 6.0.3 and 6.1: the full test harness and hot-unplug pass, and a downstream app's measurement sweeps and a 10-minute playback run were checked with a measurement microphone. Once, the miniDSP's output went silent until it was power-cycled; that didn't reproduce (see [Known limitations](#known-limitations)).

New in v0.1.3:

- ESP-IDF 6.x support
- 44.1 kHz on adaptive and synchronous endpoints without a feedback endpoint
- Falls back to the next matching alt setting when one can't be used
- Sets the clock before enabling the stream, as Linux does, and refuses sample rates the device doesn't advertise
- Queries for the active stream format and for buffered audio
- Playback and capture at the same time (opt-in, ESP-IDF 5.5+/6.x)
- Packet sizes follow the device's feedback at every sample rate
- Stricter teardown, concurrency and descriptor validation

**v0.1.2** is the last committed version, verified on the miniDSP and on the ESP32-to-ESP32 simulator and checked by ear.

Upgrading from 0.1.2: the [CHANGELOG](components/usb_host_uac2/CHANGELOG.md) lists the calls that now behave differently (unadvertised sample rates are refused, for example). [PROGRESS.md](PROGRESS.md) has the full history and the hardware test plan. v0.1.3 is also the first release on the ESP Component Registry, as `averyy/usb_host_uac2`.

## Features

- Espressif class driver pattern (`uac2_host_install()` / `uac2_host_uninstall()`) with device discovery, including devices already attached at install
- UAC2 descriptor parsing and clock topology walk (clock sources, selectors, multipliers); malformed descriptors are rejected
- Playback and capture streams with a ring buffer and blocking `write()` / `read()`
- Async feedback (16.16 or 10.14) drives packet sizes at every sample rate; adaptive and synchronous endpoints without feedback get the exact 44/45-frame pattern at 44.1 kHz
- Format selection that tries every matching alt setting, best first
- Clock control: get and set the rate, query ranges, check validity
- Volume and mute through the feature unit, with percent helpers and all-channel helpers that fall back to per-channel controls when a master control fails
- Suspend and resume without reallocating
- Stream info, buffer status (queued and in-flight audio) and a first-frame timestamp for measurement sync
- Teardown that survives hot-unplug; a stream that keeps failing is stopped and reported (`STREAM_ERROR`)

## Quick start

```c
#include "usb/uac2_host.h"

// Install the driver (creates internal USB Host client). The USB Host Library
// must already be installed, with a task running usb_host_lib_handle_events().
uac2_host_driver_config_t config = {
    .create_background_task = true,
    .task_priority = 20,
    .stack_size = 6144,
    .core_id = 0,
    .callback = device_event_cb,
    .callback_arg = NULL,
};
uac2_host_install(&config);

// In the driver callback, record addr/iface_num and notify a worker task.
// The callback runs on the USB Host event task and must not block.
enqueue_open_request(addr, iface_num);

// In a normal task, open the interface and start playback.
uac2_host_device_handle_t dev;
uac2_host_device_open(&open_config, &dev);
uac2_host_device_start(dev, &stream_config);

// Write PCM data (fills ring buffer, blocks if full)
uac2_host_device_write(dev, pcm_data, num_bytes, timeout_ms);

// Stop and close
uac2_host_device_stop(dev);
uac2_host_device_close(dev);

// When done
uac2_host_uninstall();
```

[components/usb_host_uac2/examples/basic_playback](components/usb_host_uac2/examples/basic_playback) is a complete example: it waits for a UAC2 playback interface, opens it and streams a 48 kHz / 24-bit stereo sine wave.

## Using the component

From the ESP Component Registry:

```sh
idf.py add-dependency "averyy/usb_host_uac2^0.1.3"
```

To track unreleased changes, depend on this repository instead:

```yaml
dependencies:
  averyy/usb_host_uac2:
    git: https://github.com/Averyy/esp-uac2-host.git
    path: components/usb_host_uac2
```

For local development, a `path:` dependency on `components/usb_host_uac2` or `set(EXTRA_COMPONENT_DIRS "/path/to/esp-uac2-host/components/usb_host_uac2")` also works.

## Building

Requires ESP-IDF 5.4 or newer; hardware-tested on 5.4.0, 6.0.3 and 6.1.

- On 5.4, use the latest 5.4.x: 5.4.2-5.4.4 fix USB Host Library bugs that affect audio (interface release while transfers complete, a device-close mutex leak, transfer sizes under heap poisoning, the S3 host channel count, external-hub teardown).
- On 6.x the USB Host Library is the [`espressif/usb`](https://components.espressif.com/components/espressif/usb) component, and the manifest adds it automatically (1.6.0 or newer). It also fixes a root-port disconnect race that 5.x still has.

```sh
export PATH="/opt/homebrew/bin:$PATH" && . ~/esp/esp-idf/export.sh
idf.py build                                    # test harness (repo root)
idf.py -p /dev/cu.usbmodem<SERIAL> flash
cd simulators/minidsp-2x4hd && idf.py build     # simulator for a second board
```

### Configuration

Required USB Host settings:

```
CONFIG_USB_HOST_CONTROL_TRANSFER_MAX_SIZE=512   # at least 512: the miniDSP config descriptor is 373 bytes
CONFIG_USB_HOST_HW_BUFFER_BIAS_PERIODIC_OUT=y
```

For other devices, prefer `CONFIG_USB_HOST_CONTROL_TRANSFER_MAX_SIZE=1024`: composite headsets (audio + HID, several configurations, e.g. Apple USB-C EarPods) have larger configuration descriptors and fail to enumerate with 256 or 512. Devices behind a hub, including an internal one (e.g. some Bose headsets), also need `CONFIG_USB_HOST_HUBS_SUPPORTED=y`.

Component options:

- `CONFIG_UAC2_ALLOW_UNADVERTISED_RATES` (default off): send sample rates outside the clock's advertised ranges with a warning instead of refusing them, for devices whose ranges are incomplete.
- `CONFIG_UAC2_ALLOW_DUPLEX` (default off): allow playback and capture at the same time. Set it only together with a custom FIFO split at `usb_host_install()` (ESP-IDF 5.5+/6.x):

```c
usb_host_config_t host_config = {
    .intr_flags = ESP_INTR_FLAG_LEVEL1,
    .fifo_settings_custom = {
        .nptx_fifo_lines = UAC2_HOST_DUPLEX_FIFO_NPTX_LINES,   // 64 B: control
        .ptx_fifo_lines = UAC2_HOST_DUPLEX_FIFO_PTX_LINES,     // playback endpoints up to 360 B
        .rx_fifo_lines = UAC2_HOST_DUPLEX_FIFO_RX_LINES,       // capture endpoints up to 368 B
    },
};
```

### Hardware

- ESP32-S3-DevKitC-1. Its OTG port isn't powered by default: bridge the `USB-OTG` solder pads on the back, or devices won't enumerate, even self-powered ones.
- Full Speed USB (12 Mbps) is plenty: 48 kHz / 24-bit stereo uses about 25% of the bus.
- Test device: miniDSP 2x4 HD (XMOS XU216, VID 0x2752 PID 0x0011). The simulator runs on a second ESP32-S3.

## Repository layout

```
components/usb_host_uac2/   The driver (ESP-IDF component): public API in include/usb/,
                            Kconfig, manifest, examples/
main/                       Hardware test harness
simulators/                 ESP32-S3 UAC2 device simulators (simple, full miniDSP 2x4 HD)
test/host/                  Host-side tests, parser fuzzer, driver on the ESP-IDF Linux target
ref/                        Read-only reference code and captured descriptors
docs/                       Design notes and remaining work
```

## Tests

The harness in `main/` runs on boot against any connected UAC2 device:

1. 48 kHz streaming, 10 s
2. Volume and mute during streaming, with readback
3. Stop, pause, restart
4. 44.1 kHz streaming
5. Start-time precision
6. Feedback stabilizes near the expected value and the clock reports valid
7. 16-bit streaming
8. Nine rapid start/stop cycles
9. Per-channel volume and mute
10. Repeated 48 / 44.1 kHz switches
11. Ring buffer starvation and recovery
12. Long-running stability (`CONFIG_UAC2_TEST12_DURATION_SEC`, default 1 hour; `sdkconfig.smoke.defaults` sets 20 s)

Then come live checks: suspend/resume, the duplex guard (or, in `sdkconfig.duplex.defaults` builds, a playback + capture streaming check, skipped on the miniDSP 2x4 HD for now), a sample-rate guard (another handle can't change the shared clock under a stream) and a driver reinstall with the device attached. Tones fade in and out and drain before each stop, so they don't click. For listening through a subwoofer, set `CONFIG_UAC2_TEST_TONES_SUBWOOFER=y` to move every tone to 30-110 Hz.

Harness build variants: add `sdkconfig.smoke.defaults` (short run), `sdkconfig.duplex.defaults` (playback + capture, ESP-IDF 5.5+/6.x) or `sdkconfig.heaptrace.defaults` (per-cycle leak trace) after `sdkconfig.defaults`. Give each variant its own `sdkconfig` as well as its own build directory, or an existing root `sdkconfig` overrides the defaults:

```sh
idf.py -B build-smoke -DSDKCONFIG=build-smoke/sdkconfig \
  -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.smoke.defaults" build flash
```

You don't need audio hardware for the other tests, and CI runs them on every pull request:

- `IDF_PATH=~/esp/esp-idf test/host/run.sh`: the descriptor parser against the captured miniDSP descriptor and malformed variants, and hour-long simulated playback streams that must send exactly the samples the rate asks for.
- `test/host/driver/run.sh` (with ESP-IDF activated): the real driver on the ESP-IDF Linux target against a fake USB Host Library that simulates unplugs, control timeouts, endpoint failures and other descriptors. Its 38 tests check what goes over the bus (lost or repeated audio bytes, inserted silence) and every USB Host rule the real library enforces.
- `test/host/fuzz.sh`: a libFuzzer target over the parser (needs clang with libFuzzer).

## Known limitations

- Playback and capture at the same time is opt-in (see [Configuration](#configuration)). Without the custom FIFO split, ESP-IDF's presets leave only 128 bytes for incoming packets, so the driver refuses a stream while one in the other direction runs. The split lowers the largest playback packet from 600 to 360 bytes. On ESP-IDF 6.x, stopping one stream while the other keeps running can take up to 10 RTOS ticks inside `usb_host_interface_release()`.
- Playback and capture together is unverified on the miniDSP 2x4 HD. Both streams run at full rate, but once, after a session that included them, the miniDSP's output stayed silent until it was power-cycled (it has its own power supply, so a USB replug or host reset doesn't clear it). Tested on its own afterwards, capture didn't trigger it.
- Single-clock UAC2 devices only.
- `uac2_host_device_set_sample_rate()` is refused while any stream on the device is started or suspended; `uac2_host_device_start()` applies the stream's own rate.
- Full-speed audio data endpoints must use `bInterval = 1`.
- 44.1 kHz on an asynchronous playback endpoint needs its feedback endpoint; implicit feedback isn't supported.
- Only a device's first 8 streaming alt settings are used.
- UAC2 only. For UAC1 devices, run Espressif's [`usb_host_uac`](https://components.espressif.com/components/espressif/usb_host_uac) alongside it. Its connect callback also reports UAC2 interfaces, so open only UAC1 devices from it (check `bInterfaceProtocol` or VID/PID).

If one of these gets in your way, or you hit something that isn't listed, please [open an issue](https://github.com/Averyy/esp-uac2-host/issues).

## Contributing

Bug reports, device reports and pull requests are all welcome. If a device doesn't work, these details make it much easier to fix:

- the device name and its VID:PID
- your ESP-IDF version and the USB Host settings from your `sdkconfig`
- the driver's log output, including `uac2_host_device_print_info()`
- the device's descriptors, if you can get them (`lsusb -v -d VID:PID` on Linux)
- what you expected and what happened instead

## Acknowledgements

The driver's structure follows Espressif's UAC1 driver, [`usb_host_uac`](https://github.com/espressif/esp-usb/tree/master/host/class/uac/usb_host_uac) (Apache-2.0). USBX and CherryUSB's UAC2 headers were protocol references.

## License

MIT
