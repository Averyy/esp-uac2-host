# Changelog for USB Host UAC2

All notable changes to this component will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Breaking Changes

- **`uac2_host_device_write()` larger than the ring buffer returns `ESP_ERR_INVALID_SIZE`** at once (0.1.3 returned `ESP_ERR_TIMEOUT` on every call).
- **`start()` returns `ESP_ERR_INVALID_SIZE`** for a `buffer_size` smaller than one packet (after trying the next matching alt) or a `buffer_threshold` above the ring size.
- **Mute and volume setters return `ESP_ERR_NOT_SUPPORTED` for read-only controls** without sending a request. The `*_all_channels()` setters skip read-only channels.
- **A volume RANGE with max below min** makes the volume setters and `get_volume_percent()` return `ESP_ERR_INVALID_RESPONSE`.
- **Only UAC2 interfaces (`bInterfaceProtocol` 0x20) of the first UAC2 audio function are reported.** UAC1 alt settings are skipped, and a second UAC2 function isn't reported.
- **A clock-setup request that times out during `start()`, `resume()` or `set_sample_rate()` returns `ESP_ERR_TIMEOUT`.** 0.1.3 reported the next request's `ESP_ERR_INVALID_STATE` instead.
- **`uac2_host_buffer_status_t`, `uac2_feature_unit_t` and `uac2_device_info_t` grew** (fields below). Recompile, and update hand-written bindings such as Rust FFI before calling `get_buffer_status()` or `get_info()`.

### Added

- `uac2_host_buffer_status_t.underrun_bytes` and `.dropped_bytes`: playback silence sent after the ring ran dry, and capture bytes dropped on a full ring, since `start()`/`resume()`.
- `uac2_feature_unit_t.mute_wr_ch_map` and `.volume_wr_ch_map`: channels whose control can be set.
- `CONFIG_UAC2_STOP_STREAMS_ON_RESTART` (default off): stop started streams before `esp_restart()`, for devices that need an orderly stop. The miniDSP 2x4 HD isn't known to.

### Fixed

- Asynchronous playback with a feedback endpoint other than 4 bytes, including the spec's 3-byte 10.14 format, failed to start. The driver now requests the endpoint's `wMaxPacketSize`.
- Devices with a USB-MIDI function after the audio function (MOTU UltraLite AVB, Focusrite 1235:8201) had their class requests sent to the MIDI interface.
- `stop()` and `suspend()` could hang forever if a channel error hit during an endpoint halt (an ESP-IDF bug in 5.4 and `espressif/usb` 1.6.0). Teardown now lets submitted URBs finish (about 9 ms) before SET_INTERFACE(0), as Linux does, so their audio plays out.
- On ESP-IDF 5.x, the control request right after a STALL could fail with `ESP_ERR_INVALID_STATE`. The driver retries for up to 50 ms.
- Control IN requests are rounded up to whole `bMaxPacketSize0` packets, as Espressif's class drivers do.

## [0.1.3] - 2026-10-29

### Breaking Changes

Code written for 0.1.2 may behave differently in these cases:

- **Sample rates the device doesn't advertise are refused.** `uac2_host_device_start()`, `uac2_host_device_resume()` and `uac2_host_device_set_sample_rate()` return `ESP_ERR_NOT_SUPPORTED` for a rate outside the clock's RANGE (resolution steps included); 0.1.2 only logged a warning. Some devices acknowledge such a rate and then play silence. Enable `CONFIG_UAC2_ALLOW_UNADVERTISED_RATES` to get the old behavior. A rate is still sent if the RANGE can't be read, or if the device lists more ranges than the driver reads.
- **`uac2_host_device_set_sample_rate()` returns `ESP_ERR_INVALID_STATE`** while any stream on the device is started or suspended. The clock is shared, and `start()` applies the stream's own rate. `start()` and `resume()` likewise refuse a rate that differs from another running stream on the same device.
- **`uac2_host_device_start()` returns `ESP_ERR_INVALID_STATE`** on a handle whose device was unplugged but not yet closed, or whose previous stream wasn't torn down (call stop or close first).
- **`write()` and `read()` on a closing or closed handle return `ESP_ERR_INVALID_STATE`**, as documented; 0.1.2 returned `ESP_ERR_INVALID_ARG`.
- **`get_sample_rate_range()` and `get_volume_range()` treat a non-zero `*num_ranges` as the array's capacity** (0 still means the maximum). Initialize it before the call.
- **`uac2_device_info_t` is larger** (`UAC2_MAX_AS_INTERFACES` 4 -> 8). Recompile everything that uses it; hand-written bindings of the struct (Rust FFI, for example) must be updated.
- **If a device's last interface is closed while a timed-out control request is still pending, reopening that device and `uac2_host_uninstall()` return `ESP_ERR_INVALID_STATE`** until the transfer completes (the device answers or is unplugged). The close itself succeeds; other devices are unaffected. 0.1.2 could trip an ESP-IDF assert in this situation.
- **`start()` rejects endpoints the ESP32 host can't schedule** with `ESP_ERR_NOT_SUPPORTED`: data endpoints with `bInterval` other than 1, and feedback endpoints whose interval doesn't fit the host's isochronous list. 0.1.2 warned and continued; a feedback `bInterval` of 7 or more could abort the host. Other matching alt settings are still tried.
- **`get_start_time()` and `get_feedback()` return 0** unless the stream is active and the device attached (0.1.2 returned stale values after stop or unplug).
- On ESP-IDF 6.x the manifest pulls in `espressif/usb` (1.6.0 or newer).
- **The component folder is now `components/usb_host_uac2`** (was `components/uac2_host`), matching its registry name `averyy/usb_host_uac2`, and the build component name is `usb_host_uac2`. Update `path:` dependencies, `EXTRA_COMPONENT_DIRS` and `REQUIRES uac2_host`. Header and function names don't change.

### Added

- ESP-IDF 6.x support (USB Host Library from the `espressif/usb` component).
- `uac2_host_device_get_stream_info()`: the alt setting `start()` chose, bit depth, sub-slot size, rate, endpoints and sync type.
- `uac2_host_device_get_buffer_status()`: queued and in-flight audio, for latency reporting and draining before `stop()`.
- `uac2_host_device_set_mute_all_channels()`.
- Playback and capture at the same time on ESP32-S3, opt-in: `CONFIG_UAC2_ALLOW_DUPLEX` plus `UAC2_HOST_DUPLEX_FIFO_*` in `usb_host_config_t.fifo_settings_custom` (ESP-IDF 5.5+/6.x).
- Devices already attached when the driver is installed are reported on the first event pass, so a reinstall doesn't need a replug.

### Changed

- 44.1 kHz plays on adaptive and synchronous endpoints without a feedback endpoint, with the exact 44/45-frame packet pattern. Only asynchronous endpoints need feedback for fractional rates.
- `start()` tries every alt setting that matches the format, best first, and moves to the next when one can't carry the stream (packet size, endpoints the host can't fit, unsupported intervals).
- The clock is set before SET_INTERFACE on start and resume (the order Linux uses for UAC2). After a SET, the rate is read back and a mismatch is logged.
- Packet sizes follow the device's feedback at every sample rate; 0.1.2 used fixed sizes at integer-kHz rates.
- `set_volume_all_channels()` continues past a master control that STALLs; `set_volume_percent()` snaps to the device's volume resolution.
- Callback warnings (skipped or failed packets, capture overflow) are aggregated and logged at most once per second.
- Up to 8 streaming alt settings per device (was 4).

### Fixed

- A control transfer that timed out stayed in flight (ESP-IDF can't cancel EP0 transfers), and its late completion could touch freed memory or trip an ESP-IDF assert at close. It's now tracked until it completes, and the device stays open until then.
- Closing a device right after a control transfer could assert in `usbh_dev_close()` on ESP-IDF 5.x; the driver waits 10 ms first.
- Teardown waits for in-flight transfers before releasing the interface, which avoids ESP-IDF #17707 refusals and a 100 ms sleep in `espressif/usb` on every stop.
- A ring buffer could be deleted while a task was still blocked in `write()` or `read()`.
- Disconnect, close and uninstall races (references taken outside locks, a lifecycle mutex deleted under a caller, concurrent uninstalls, the background task outliving its installation).
- Polling loops never slept at `CONFIG_FREERTOS_HZ=100` (`pdMS_TO_TICKS(5)` is 0).
- A dead playback endpoint never reached `STREAM_ERROR` while its feedback endpoint kept working; URBs whose every packet failed weren't counted as errors.
- Malformed configuration descriptors that ESP-IDF's own descriptor walk loops on, or reads past, are rejected; alt settings are kept only if ESP-IDF would claim exactly the endpoints that were validated.
- The range queries could overflow a caller array smaller than the maximum.
- A failed URB or feedback allocation left the stream half-built or streaming without feedback; such starts now fail cleanly.

### Known issues

- Once, during a test-harness session on a miniDSP 2x4 HD, the device's output went silent while USB looked healthy, until the miniDSP was power-cycled. It didn't reproduce: capture, 44.1/48 kHz switching, idle sample-rate changes and a driver reinstall each left a freshly power-cycled miniDSP playing, and so did a 10-minute 48 kHz run. Playback and capture at the same time is unverified on that device.

## [0.1.2] - 2026-04-14

### Fixed

- Valid audio was replaced with silence at the TX ring buffer's internal wrap point.
- Periodic dropouts under system load (WiFi, HTTP, HID): URBs now carry several packets (`CONFIG_UAC2_NUM_PACKETS_PER_URB`, default 3), giving a 9 ms pipeline.

## [0.1.1] - 2026-04-09

### Added

- Clock topology walk from the terminal through selectors and multipliers to the clock source (0.1.0 used the first clock source).

### Fixed

- Hot-unplug during playback: the interface is released after the device is gone and resources are kept until callbacks finish.

- `resume()` reclaims the interface and restarts playback.
- Feature-unit controls are addressed to the right interface.

## [0.1.0] - 2026-04-06

First version: Espressif class driver pattern (install/uninstall, device discovery, reference-counted interfaces), UAC2 descriptor parsing, isochronous playback and capture with ring buffers, async feedback, clock and feature-unit control, suspend/resume, disconnect-aware teardown.
