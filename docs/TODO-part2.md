# TODO Part 2 — New Features & Architectural Changes

New features, architectural changes, and significant new code. Do after real miniDSP 2x4 HD hardware testing. See also `TODO-part1.md` (code review findings), `TODO-real-device-testing.md`, and `TODO-publish-component-registry.md`.

---

## 1. Install/Uninstall Lifecycle

Every official Espressif USB host class driver (CDC-ACM, HID, MSC, UAC1) follows this pattern:

```c
uac2_host_driver_config_t config = {
    .create_background_task = true,
    .task_priority = 5,
    .stack_size = 4096,
    .callback = my_driver_event_cb,   // fires on device connect/disconnect
    .callback_arg = NULL,
};
uac2_host_install(&config);

// Driver fires callback with UAC2_HOST_DRIVER_EVENT_TX_CONNECTED
// App opens from callback:
uac2_host_device_open(addr, iface_num, &dev_config, &handle);

// Shutdown
uac2_host_uninstall();
```

### What to implement

- [ ] `uac2_host_install()` / `uac2_host_uninstall()` — registers USB Host client, optionally creates background task
- [ ] `uac2_host_handle_events(timeout)` — for manual event pumping when `create_background_task = false`
- [ ] Driver-level event callback: `UAC2_HOST_DRIVER_EVENT_TX_CONNECTED`, `RX_CONNECTED`, `DEVICE_DISCONNECTED`
- [ ] Internal device discovery: on `USB_HOST_CLIENT_EVENT_NEW_DEV`, parse config descriptor for UAC2 interfaces
- [ ] Internal disconnect handling: on `USB_HOST_CLIENT_EVENT_DEV_GONE`, stop streams, notify app
- [ ] Singleton driver state with linked list of open devices
- [ ] Change `uac2_host_device_open()` signature to `(addr, iface_num, config, &handle)`
- [ ] `uac2_host_device_open_with_vid_pid()` convenience function
- [ ] Reference counting (`opened_cnt`) on the physical USB device. UAC1 tracks how many interfaces share the USB device handle and only closes it when the count drops to zero. Needed if the driver manages the USB device lifecycle internally.
- [ ] Two-phase disconnect: on `DEV_GONE`, set a flag on the device, fire the user's disconnect callback, and let the user call `device_close()` when ready. Prevents use-after-free if the user is mid-operation. UAC1 uses `FLAG_INTERFACE_WAIT_USER_DELETE` for this.

### Trade-offs

~500 lines of new code. Full API break. Not needed for minidsp-open (which already has a USB Host client for HID). Only do this for public release or if other consumers appear.

---

## 2. Suspend/Resume Without Full Teardown

Currently every `stream_stop`/`stream_start` cycle reallocates URBs and ring buffer. For measurement sweeps (~12s each, with brief pauses between), this wastes time and fragments memory.

### What to implement

- [ ] `uac2_host_stream_suspend(dev, dir)` — SET_INTERFACE(alt=0), halt/flush endpoints, park URBs. Resources stay allocated.
- [ ] `uac2_host_stream_resume(dev, dir)` — SET_INTERFACE(alt=N), resubmit URBs. No reallocation.
- [ ] `READY` state between `IDLE` and `ACTIVE`
- [ ] Optional: `FLAG_STREAM_SUSPEND_AFTER_START` to pre-arm without starting transfers

### Lesson learned

The SET_INTERFACE ordering matters: send SET_INTERFACE(alt=0) BEFORE halt/flush/clear (device stops first, then host cleans up). Reverse order causes the device to send data into halted pipes. This was caught during code review and fixed — the suspend implementation should follow the same pattern.

---

## 3. Kconfig — DONE

### What to implement

- [x] Create `components/uac2_host/Kconfig`:
  ```
  menu "UAC2 Host Driver"
      config UAC2_HOST_NUM_ISOC_URBS
          int "Number of isochronous URBs"
          default 3
      config UAC2_HOST_NUM_PACKETS_PER_URB
          int "Packets per URB"
          default 1
      config UAC2_HOST_CTRL_XFER_TIMEOUT_MS
          int "Control transfer timeout (ms)"
          default 5000
      config UAC2_HOST_CTRL_XFER_MAX_SIZE
          int "Max control transfer data size"
          default 256
      config UAC2_HOST_MAX_CONSECUTIVE_ERRORS
          int "Max consecutive transfer errors before stopping"
          default 10
      config UAC2_HOST_RINGBUF_SAFE_DELETE_WAIT_MS
          int "Delay before ringbuffer deletion (ms)"
          default 50
          help
              After unblocking tasks waiting on the ringbuffer, wait this long
              before deleting it to ensure blocked tasks have exited.
      config UAC2_HOST_PRINTF_DESCRIPTORS
          bool "Print descriptors on device connect"
          default n
          help
              Log parsed UAC2 descriptor info at INFO level when a device is opened.
  endmenu
  ```
- [x] Replace `#define UAC2_*` in header with `CONFIG_UAC2_*` (with fallback defaults)
- [x] Kconfig created with 4 tunable parameters

---

## 4. Device Handle Validation

Official drivers validate handles by walking an internal linked list before dereferencing. Catches stale/freed handles.

- [ ] `is_device_in_list(handle)` check at top of every public API function
- [ ] Return `ESP_ERR_INVALID_ARG` for invalid handles
- [ ] Requires the device linked list from section 1
- [ ] Prevent double-open: if `uac2_host_device_open()` is called for a USB device that's already open, return the existing handle instead of creating a duplicate. Two handles sharing the same EP0 would race on control transfers. UAC1 handles this by returning `ESP_OK` with the existing handle when `opened_cnt > 0`.

---

## 5. State Mutex for Public API — PARTIALLY DONE

Code review identified that `stream_write`/`stream_read` read `stream->state` without holding any lock. Mitigated: per-stream spinlock added, state recheck after blocking ringbuf calls, `closing` flag on device, safe ringbuffer deletion with conditional delay. Full `api_mutex` wrap deferred.

- [ ] Add `SemaphoreHandle_t api_mutex` to device struct
- [ ] Wrap all public functions in mutex take/give — this includes `set_mute`, `get_mute`, `set_volume`, `get_volume`, not just stream functions. Currently these attempt control transfers with no state check — they'll send USB requests even if the device is mid-close.
- [ ] Use timeout to prevent deadlocks

---

## 6. Stream Dead Notification

When the consecutive error limit is hit and URBs stop re-submitting, the stream silently dies. The caller has no way to detect this — `stream_write` keeps accepting data into the ringbuf (which nobody drains), eventually returning `ESP_ERR_TIMEOUT`.

- [ ] Fire `UAC2_HOST_EVENT_STREAM_DEAD` when error limit reached
- [ ] Or transition state to `UAC2_STREAM_STATE_ERROR` so `stream_write`/`stream_read` return `ESP_ERR_INVALID_STATE`

---

## 7. UAC2_HOST_EVENT_DISCONNECTED

Currently declared in the enum but never fired. Disconnect is only detected via `USB_TRANSFER_STATUS_NO_DEVICE` in URB callbacks, which silently stops the stream.

- [ ] Fire `UAC2_HOST_EVENT_DISCONNECTED` on first `NO_DEVICE` callback (use a flag to avoid firing 3+ times for multiple in-flight URBs)
- [ ] Adopt UAC1's two-phase disconnect handshake: set a flag on the device, fire the disconnect callback, but don't free resources. Let the user call `device_close()` when ready. This prevents use-after-free if the user is mid-operation when disconnect fires. See also section 1.

---

## 8. Volume/Mute Hardening

### Volume range caching and validation

UAC1 queries and caches the volume range at device open, then rejects out-of-range values in `set_volume()`. Our driver sends values blindly — if the user passes an out-of-range value, the device will STALL.

- [ ] At device open, query volume RANGE (min/max/res) and cache on the device struct
- [ ] In `uac2_host_set_volume()`, validate against cached range. Return `ESP_ERR_INVALID_ARG` if out of range.

### Feature unit capability detection

**Correctness bug:** The driver uses the first feature unit found and assumes it supports both mute and volume. UAC2 feature units encode which controls are supported in `bmaControls[]` (4 bytes per channel: bits 0-1 = mute, bits 2-3 = volume). If the feature unit only supports, say, bass boost, mute/volume commands will STALL the device.

- [ ] Parse `bmaControls[]` from the feature unit descriptor during descriptor parsing
- [ ] Store capability bitmaps (`has_mute`, `has_volume`, per-channel maps) on `uac2_feature_unit_t`
- [ ] In `set_mute()`/`set_volume()`, check capability before sending. Return `ESP_ERR_NOT_SUPPORTED` if the control isn't available.

### Normalized volume API

- [ ] `uac2_host_set_volume_percent(dev, channel, uint8_t percent)` — maps 0-100 to device's cached min/max range
- [ ] `uac2_host_get_volume_percent(dev, channel, &percent)`

---

## 9. Debug Print Function

All official drivers have descriptor/state dump functions. Extremely useful for real hardware debugging.

- [ ] `uac2_host_device_printf_info(dev)` — logs parsed topology, clock info, stream state, endpoints
- [ ] Pattern: CDC-ACM has `cdc_acm_host_desc_print()`, MSC has `msc_host_print_descriptors()`

---

## 10. Sample Rate Validation at Stream Start

`find_matching_as_iface()` matches on channels and bit_resolution but doesn't validate sample rate. It sets whatever rate the caller asks for via the clock source. If the device doesn't support that rate, it will STALL or produce garbage.

- [ ] At `stream_start()` time, query RANGE from the clock source (already supported via `get_sample_rate_range()`)
- [ ] Validate that the requested rate falls within a supported range or matches a discrete value
- [ ] Return `ESP_ERR_NOT_SUPPORTED` with a clear log message if the rate isn't supported
- [ ] Alternative: warn but allow (some devices accept non-advertised rates). At minimum, log a warning.

---

## 11. Component Registry Packaging

- [x] `idf_component.yml` with version, description, dependencies
- [ ] LICENSE file in component directory
- [ ] API documentation (Doxygen comments on all public functions in `uac2_host.h`)
- [ ] Clean build with `-Werror -Wextra` and no warnings
- [ ] Examples directory with `basic_playback/` example (see `TODO-publish-component-registry.md`)

---

## Nice to Have (Lower Priority)

Items that improve polish but aren't blockers for a v1.0 release.

- [ ] **Query alt setting parameters:** Convenience function `uac2_host_get_alt_setting_info(dev, iface_num, alt, &info)` to let callers discover available alt settings (channels, bit depth, sample rates) before choosing one. Currently callers must inspect `uac2_device_info_t.as_ifaces[]` manually.
- [ ] **Set volume on all channels:** Convenience wrapper that iterates channels based on the feature unit's `bmaControls` bitmap, so callers don't need to loop manually.
- [ ] **Document TX silence behavior:** When the ringbuffer is empty, URBs send silence (zeros) and never stop re-submitting. This is correct for async audio (keeps feedback loop alive, prevents device buffer starvation) but differs from UAC1 which stops sending and parks URBs. Document this design choice in the README or header.

---

## Testing Checklist Before Release

- [ ] All items above implemented
- [ ] Tested against real miniDSP 2x4 HD
- [ ] Tested against at least one other UAC2 device (to confirm generic)
- [ ] Disconnect/reconnect stress test (10+ cycles)
- [ ] Long-running stability (30+ minutes)
- [ ] Memory leak check (`heap_caps_get_free_size` before/after open/close cycles)
- [ ] Stack watermark check (`uxTaskGetStackHighWaterMark`)
- [ ] `llms.txt` and `README.md` updated

---

## Lessons Learned (From Simulator Development)

These informed the design and should be preserved:

1. **ESP_LOGI in ISR = instant crash.** `tud_audio_rx_done_isr` (and any USB ISR callback) must use `ESP_DRAM_LOGI` with a `DRAM_ATTR` tag string. This was the root cause of the "SET_INTERFACE crash" that blocked development for a session.

2. **`usb_host_interface_claim` does NOT send SET_INTERFACE.** It only sets up host-side pipes. A separate control transfer is required to notify the device. This is undocumented in ESP-IDF.

3. **SET_INTERFACE ordering: device first, then host.** Send SET_INTERFACE(alt=0) to the device BEFORE calling halt/flush/clear on host endpoints. The Espressif UAC1 driver does this correctly; we initially had it reversed.

4. **`ctrl_request` mutex protocol is a bug factory.** The pattern where `ctrl_request` holds the mutex and callers must release it led to fragile code. The `ctrl_request_no_data()` wrapper (auto-releases on success) eliminated this entire class of bugs. For public release, consider always auto-releasing and having GET callers copy data from an internal buffer.

5. **URB lifecycle accounting must be airtight.** Every code path that touches a URB must correctly maintain `urbs_in_flight`. We found three separate leak paths during code review: RX resubmit failure, TX state-check bail-out, and feedback resubmit race. All fixed, but this is the most fragile part of the driver.

6. **Feedback endpoint is the canary.** When something goes wrong with SET_INTERFACE or endpoint activation, the feedback endpoint fails first (it's IN direction, smaller, more timing-sensitive). If feedback works, everything works.
