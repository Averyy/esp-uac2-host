# Prep Driver for Public Release

What needs to happen before publishing `uac2_host` as a standalone ESP-IDF component (ESP Component Registry or GitHub). Do this AFTER real hardware verification (see `TODO-real-device-testing.md`). Ordered by priority.

## Current State (as of 2026-04-05)

The driver is complete and verified end-to-end against an ESP32-to-ESP32 simulator. All 5 automated tests pass. 487,000+ audio frames received by simulator with zero errors. Full 6-agent code review completed, all critical/high/medium findings fixed.

**What's already solid:**
- UAC2 descriptor parsing with bLength validation
- SET_INTERFACE for proper endpoint lifecycle (matches Espressif UAC1 pattern)
- Feedback-based adaptive packet sizing (10.14 and 16.16 formats)
- `ctrl_request_no_data()` wrapper eliminates mutex leak bugs
- Transfer error limiting (10 consecutive → stop re-submission)
- Endpoint halt/flush/clear on stream stop
- Atomic `fb_value`, `urbs_in_flight`, `consecutive_errors`
- Spinlock-protected stream state transitions
- ISR-safe logging pattern established (`ESP_DRAM_LOGI` with `DRAM_ATTR` tag)

**What the API currently requires:**
The caller manages USB Host Library client, device enumeration, and event loop. This is fine for minidsp-open (which already has a USB Host client for HID) but diverges from the standard ESP-IDF class driver pattern.

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

### Trade-offs

~500 lines of new code. Full API break. Not needed for minidsp-open. Only do this for public release or if other consumers appear.

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

## 3. Kconfig

### What to implement

- [ ] Create `components/uac2_host/Kconfig`:
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
  endmenu
  ```
- [ ] Replace `#define UAC2_*` in header with `CONFIG_UAC2_HOST_*`

---

## 4. Device Handle Validation

Official drivers validate handles by walking an internal linked list before dereferencing. Catches stale/freed handles.

- [ ] `is_device_in_list(handle)` check at top of every public API function
- [ ] Return `ESP_ERR_INVALID_ARG` for invalid handles
- [ ] Requires the device linked list from section 1

---

## 5. State Mutex for Public API

Code review identified that `stream_write`/`stream_read` read `stream->state` without holding any lock. A separate task calling `stream_stop` concurrently could cause issues.

- [ ] Add `SemaphoreHandle_t api_mutex` to device struct
- [ ] Wrap all public functions in mutex take/give
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

---

## 8. Normalized Volume API

- [ ] `uac2_host_set_volume_percent(dev, channel, uint8_t percent)` — maps 0-100 to device's min/max range
- [ ] `uac2_host_get_volume_percent(dev, channel, &percent)`
- [ ] Query and cache volume range at device open time

---

## 9. Debug Print Function

All official drivers have descriptor/state dump functions. Extremely useful for real hardware debugging.

- [ ] `uac2_host_device_printf_info(dev)` — logs parsed topology, clock info, stream state, endpoints
- [ ] Pattern: CDC-ACM has `cdc_acm_host_desc_print()`, MSC has `msc_host_print_descriptors()`

---

## 10. bInterval Fixup for Full Speed

Some USB audio devices report incorrect `bInterval` values at Full Speed.

- [ ] For isochronous data endpoints at Full Speed, warn if `bInterval != 1` (USB 2.0 spec requires 1)
- [ ] NOTE: Feedback endpoints legitimately use `bInterval > 1` (miniDSP uses bInterval=4 = 8ms). Do NOT fixup feedback endpoints.

---

## 11. Component Registry Packaging

- [ ] `idf_component.yml` with version, description, dependencies
- [ ] LICENSE file in component directory
- [ ] API documentation (Doxygen comments on all public functions in `uac2_host.h`)
- [ ] Clean build with `-Werror -Wextra` and no warnings

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
