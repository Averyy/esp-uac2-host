# Prep Driver for Public Release

What needs to happen before publishing `uac2_host` as a standalone ESP-IDF component (ESP Component Registry or GitHub). Ordered by priority within each section.

## Current State

The driver works end-to-end: descriptor parsing, control requests (CUR/RANGE), isochronous streaming with feedback-based adaptive packet sizing, volume/mute, disconnect handling. Tested against an ESP32-to-ESP32 simulator for 100+ seconds sustained, all sample rates, stop/restart cycles, and hot unplug/replug. Not yet tested against real miniDSP hardware.

The API currently requires the caller to manage the USB Host Library client, device enumeration, and event loop. This is fine for minidsp-open (which already has a USB Host client for HID) but diverges from the standard ESP-IDF class driver pattern used by CDC-ACM, HID, MSC, and UAC1 drivers.

---

## 1. Install/Uninstall Lifecycle (Architectural)

Every official Espressif USB host class driver (CDC-ACM, HID, MSC, UAC1) follows this pattern:

```c
// App installs the driver once at startup
uac2_host_driver_config_t config = {
    .create_background_task = true,
    .task_priority = 5,
    .stack_size = 4096,
    .core_id = 0,
    .callback = my_driver_event_cb,   // fires on device connect/disconnect
    .callback_arg = NULL,
};
uac2_host_install(&config);

// Driver internally:
// - Registers its own USB Host client
// - Optionally creates a background task to pump usb_host_client_handle_events()
// - Scans new devices for Audio Class + bInterfaceProtocol=0x20
// - Fires callback with UAC2_HOST_DRIVER_EVENT_TX_CONNECTED / RX_CONNECTED

// App opens a specific device/interface from the callback
uac2_host_device_open(addr, iface_num, &dev_config, &handle);

// At shutdown
uac2_host_uninstall();
```

### What to implement

- [ ] Add `uac2_host_install(const uac2_host_driver_config_t *config)` — registers USB Host client, optionally creates background task
- [ ] Add `uac2_host_uninstall()` — verifies all devices closed, deregisters client, deletes task
- [ ] Add `uac2_host_handle_events(TickType_t timeout)` — for manual event pumping when `create_background_task = false`
- [ ] Add driver-level event callback with events: `UAC2_HOST_DRIVER_EVENT_TX_CONNECTED`, `UAC2_HOST_DRIVER_EVENT_RX_CONNECTED`, `UAC2_HOST_DRIVER_EVENT_DEVICE_DISCONNECTED`
- [ ] Internal device discovery: on `USB_HOST_CLIENT_EVENT_NEW_DEV`, open device, get config descriptor, parse for UAC2 interfaces, fire callback if found
- [ ] Internal disconnect handling: on `USB_HOST_CLIENT_EVENT_DEV_GONE`, find all open interfaces for that device, stop streams, notify app
- [ ] Singleton driver state (`s_uac2_driver`) with linked list of open devices/interfaces
- [ ] Change `uac2_host_device_open()` signature: take `(addr, iface_num, config, &handle)` instead of `(client, usb_dev, cb, cb_arg, &handle)`
- [ ] Add `uac2_host_device_open_with_vid_pid()` convenience function
- [ ] Update test harness main.c to use new API

### Why

- Standard ESP-IDF pattern — users expect it
- Composite device safety — driver gets its own USB Host client, can't interfere with HID
- Disconnect handling moves inside the driver — fewer race conditions for app code
- Required for ESP Component Registry publishing

### Trade-offs

- ~500 lines of new code (singleton state, linked list, task management, event dispatch)
- Full API break — all downstream code changes
- Makes USB event flow less visible for debugging
- Not needed for the only current consumer (minidsp-open already has its own USB client)

---

## 2. Suspend/Resume Without Full Teardown

UAC1 has a 4-state lifecycle: `IDLE -> READY -> ACTIVE -> SUSPENDING -> READY`. Key operations:

- `suspend()`: SET_INTERFACE(0), flush ring buffer, return URBs to free list. Resources stay allocated.
- `resume()`: Re-select alt setting, re-set frequency, resubmit URBs. No allocation.
- `stop()`: Full teardown (suspend + release interface + free transfers).

Our driver only has `stream_start()` (allocate everything) and `stream_stop()` (free everything). Every pause requires full teardown and reallocation.

### What to implement

- [ ] Add `uac2_host_stream_suspend(dev, dir)` — sets alt 0, flushes ring buffer, parks URBs. Keeps interface claimed and resources allocated.
- [ ] Add `uac2_host_stream_resume(dev, dir)` — re-selects alt setting, resubmits URBs. No reallocation.
- [ ] Add `READY` state between `IDLE` and `ACTIVE` for pre-claimed-but-not-streaming
- [ ] Optional: `FLAG_STREAM_SUSPEND_AFTER_START` to claim interface and allocate without starting transfers

### Why

- Measurement sweeps are ~12s. Between sweeps, the app pauses briefly, possibly changes sample rate, then plays again. Full teardown/reallocation on every pause is slow and fragments memory on ESP32.
- Reduces risk of allocation failure on a constrained system.

---

## 3. Kconfig for Build-Time Configuration

All official ESP-IDF components expose tunables via `menuconfig`.

### What to implement

- [ ] Create `components/uac2_host/Kconfig` with:
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
  endmenu
  ```
- [ ] Replace `#define UAC2_NUM_ISOC_URBS` etc. with `CONFIG_UAC2_HOST_*` references
- [ ] Add `sdkconfig.defaults` entries if any non-default values are needed

---

## 4. Device Handle Validation

Official drivers validate every handle by walking an internal linked list before dereferencing. This catches stale/freed handles.

### What to implement

- [ ] Add `is_device_in_list(handle)` check at the top of every public API function
- [ ] Return `ESP_ERR_INVALID_ARG` for invalid handles instead of dereferencing freed memory
- [ ] Requires the device linked list from section 1

---

## 5. State Mutex for Public API Functions

UAC1 wraps every public API call in a per-interface mutex (`try_lock` / `unlock`). This prevents races like calling `set_volume()` while `stream_stop()` is tearing down.

### What to implement

- [ ] Add `SemaphoreHandle_t api_mutex` to the device struct
- [ ] Wrap all public functions (`stream_start`, `stream_stop`, `stream_write`, `set_volume`, etc.) in `xSemaphoreTake(api_mutex)` / `xSemaphoreGive(api_mutex)`
- [ ] Use `xSemaphoreTake` with timeout to prevent deadlocks

---

## 6. Normalized Volume API

UAC1 exposes both raw dB and normalized 0-100 volume. Convenient for apps that don't want to deal with 1/256 dB units.

### What to implement

- [ ] Add `uac2_host_set_volume_percent(dev, channel, uint8_t percent)` — maps 0-100 to the device's reported min/max range
- [ ] Add `uac2_host_get_volume_percent(dev, channel, uint8_t *percent)`
- [ ] Query volume range once at device open, cache min/max

---

## 7. bInterfaceProtocol Check in UAC2 Detection

Currently UAC2 detection relies solely on `bcdADC >= 0x0200`. The spec says UAC2 also has `bInterfaceProtocol = 0x20`. Checking both provides defense against devices with malformed AC headers.

### What to implement

- [ ] In the descriptor parser, also check `bInterfaceProtocol == 0x20` on the Audio Control interface
- [ ] Use the existing `UAC2_PROTOCOL_VERSION_02_00` constant (defined but unused)

---

## 8. Debug Print Function

All official drivers have a `printf_device_param()` or similar for dumping device/interface state.

### What to implement

- [ ] Add `uac2_host_device_printf_info(dev)` — logs parsed descriptor topology, clock info, stream state, endpoint config
- [ ] Useful for debugging real hardware (miniDSP, JDS Labs, etc.)

---

## 9. bInterval Fixup for Full Speed

Some USB audio devices report incorrect `bInterval` values at Full Speed. UAC1 detects and patches this.

### What to implement

- [ ] During descriptor parsing, if device is Full Speed and `bInterval != 1` for isochronous endpoints, log a warning and override to 1
- [ ] Full Speed isochronous endpoints must use `bInterval = 1` per USB 2.0 spec

---

## 10. Multi-Interface / Reference Counting

UAC1 tracks `opened_cnt` per physical USB device. Multiple interfaces (TX speaker + RX mic) share one parent device struct. The USB device handle is only closed when the last interface closes.

### What to implement

- [ ] Add reference counting per USB device (for future TX+RX on different interfaces)
- [ ] Only call `usb_host_device_close()` when refcount reaches 0
- [ ] Not critical until duplex support is added (ESP32-S3 FIFO limitation prevents simultaneous TX+RX anyway)

---

## Testing Checklist Before Release

- [ ] All items above implemented
- [ ] Tested against real miniDSP 2x4 HD (not just simulator)
- [ ] Tested against JDS Labs Atom DAC+ (UAC1 fallback device — should get `ESP_ERR_NOT_SUPPORTED`)
- [ ] Tested against cheap USB sound card (UAC1 — should get `ESP_ERR_NOT_SUPPORTED`)
- [ ] Disconnect/reconnect stress test (10+ cycles)
- [ ] Long-running stability (30+ minutes)
- [ ] Memory leak check (`heap_caps_get_free_size` before/after open/close cycles)
- [ ] Stack watermark check on all tasks (`uxTaskGetStackHighWaterMark`)
- [ ] Clean build with `-Werror -Wextra` and no warnings
- [ ] API documentation (Doxygen comments on all public functions)
- [ ] README with usage example
- [ ] LICENSE file in component directory
- [ ] `idf_component.yml` for ESP Component Registry
