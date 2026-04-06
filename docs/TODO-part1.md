# TODO Part 1 — Code Review Findings

Full codebase review (security, correctness, architecture, regression). All findings below. Excludes `ref/` and `managed_components/` (third-party).

---

## High Priority

### 1. `device_close()` silently skips SET_INTERFACE(alt=0)

**File**: `uac2_host.c:1030` and `:148`

`device_close()` sets `dev->closing = true` before calling `stream_stop()`. Inside `stream_stop()`, the SET_INTERFACE(alt=0) control request hits `ctrl_request()`'s early-return guard (`if (dev->closing) return ESP_ERR_INVALID_ARG`) and is silently dropped. The USB device is never told to deactivate its isochronous endpoints on normal close. On real hardware, the device continues driving its feedback endpoint and DSP state across what should be a clean teardown.

**Fix**: Move `dev->closing = true` to after the `stream_stop()` calls. If external-caller protection during close is needed, add a separate `bool internal_close` flag or have `stream_stop` bypass the `closing` check when called internally.

### 2. Integer underflow in `usb_string_to_ascii` — OOB read from hostile USB device

**File**: `uac2_host.c:896`

If a rogue device sends a string descriptor with `bLength = 0` or `1`, the expression `(str_desc->bLength - 2) / 2` underflows (uint8_t wraps to 254/255), yielding `num_chars` = 126/127. The `out` buffer is protected by the size clamp, but `wData[i]` is read far past the actual descriptor length — out-of-bounds read that could crash or leak heap contents to the serial log.

**Fix**: Add `if (str_desc->bLength < 2) return;` before the subtraction.

### 3. `UAC2_HOST_EVENT_DISCONNECTED` declared in public API but never fired

**File**: `uac2_host.h:84`
**Also in**: `TODO-part2.md` section 7, `TODO-real-device-testing.md`

The enum value is documented and handled by callers (`main.c:168`), but the driver never emits it. Disconnect is only observable via `USB_TRANSFER_STATUS_NO_DEVICE` in isochronous callbacks, which silently stops re-submission. Any consumer relying on this event for disconnect handling gets no notification — broken API contract.

**Fix**: Fire on the first `NO_DEVICE` callback (with an `atomic_bool disconnect_fired` flag to prevent N firings for N in-flight URBs), or remove from the public header entirely.

### 4. `stream_write`/`stream_read` use-after-free — 20ms heuristic is not safe

**File**: `uac2_host.c:780-782` and `:1370-1382`
**Also in**: `TODO-part2.md` section 5 (partially)

`stream_free()` unblocks tasks blocked on the ringbuffer, then waits `vTaskDelay(20ms)` before calling `heap_caps_free(stream)`. If the blocked task doesn't wake and return within 20ms (lower priority, scheduler load), it accesses freed memory. The `stream_read` path is worse: after the dummy-byte unblock, `vRingbufferReturnItem` may be called after `vRingbufferDelete`.

**Fix**: Replace the 20ms heuristic with a semaphore handshake — `stream_write`/`stream_read` signal before returning, `stream_free` waits on that signal before freeing. Or document that `stream_stop` must be called from the same task as `stream_write`/`stream_read`.

---

## Medium Priority

### 5. Clock Selector `nr_pins` OOB read in log loop

**File**: `uac2_desc.c:322-324`

Parser clamps `nr_pins` storage to 4 entries (`source_ids[4]`), but stores the raw value in `cx->nr_pins`. `uac2_log_device_info` iterates `for (j = 0; j < cx->nr_pins; j++)` with no bound — reads past the array for any device with `bNrInPins > 4`.

**Fix**: Cap at parse time: `cx->nr_pins = (desc[4] < 4) ? desc[4] : 4;`

### 6. `calc_packet_size` integer overflow with device-controlled inputs

**File**: `uac2_host.c:648-656`

`samples_per_frame * channels * sub_slot_size` is computed as `uint32_t` but cast to `uint16_t` without overflow check. A rogue device with `bNrChannels = 255`, `bSubSlotSize = 4`, `sample_rate = 96000` produces 97920, which wraps to 32384. The subsequent MPS check uses the truncated value, potentially leading to undersized URB allocation.

**Fix**: Add overflow check: `if (result > UINT16_MAX) { ESP_LOGE(...); return 0; }` or validate inputs against reasonable bounds.

### 7. Feedback value not validated — device can force zero-length packets

**File**: `uac2_host.c:399-413`

A rogue feedback endpoint can send `fb = 0xFFFF0000`, making `nominal_samples = 65535`, which wraps `samples_this_frame` to 0 via `uint16_t`. Zero-length isochronous transfers are submitted.

**Fix**: Reject `nominal_samples == 0` or values outside a reasonable window around the nominal sample rate. Fall back to `stream->packet_size`.

### 8. `tx_done_pending` data race — plain `bool` across task contexts

**File**: `uac2_host.c:86, 430-438, 1382`
**Also in**: `TODO-part2.md` section 5 (broadly)

Written from USB callback context and application task without atomics or locks. Undefined behavior per C11 on dual-core ESP32-S3. Every other shared field (`urbs_in_flight`, `consecutive_errors`, `fb_value`) uses `_Atomic`.

**Fix**: Declare as `_Atomic bool` and use `atomic_load`/`atomic_store`.

### 9. `volatile bool closing` insufficient for cross-core visibility

**File**: `uac2_host.c:111, 148, 1030`

`volatile` prevents register caching but doesn't guarantee cross-core visibility (no memory barrier). Every other shared field uses `_Atomic` — this is the inconsistent one.

**Fix**: Declare as `_Atomic bool`.

### 10. `stream_get_start_time` TOCTOU null deref

**File**: `uac2_host.c:1422-1426`

Between the `!dev->tx_stream` null check and the dereference `dev->tx_stream->first_frame_us`, another task can set the pointer to NULL and free the stream.

**Fix**: Load pointer once into a local: `uac2_stream_t *s = dev->tx_stream; if (!s) return 0; return s->first_frame_us;`

### 11. `UAC2_MAX_AS_INTERFACES = 4` silently truncates without logging

**File**: `uac2_desc.h:216` and `uac2_desc.c:244`

Devices with >4 AS interfaces are silently truncated. `stream_start` returns `ESP_ERR_NOT_FOUND` with no diagnostic that the limit was hit.

**Fix**: Add `ESP_LOGW` at the truncation site. Consider making configurable via Kconfig with default of 8.

### 12. Control transfer timeout recovery can corrupt next request

**File**: `uac2_host.c:196-212`

After timeout, the halt/flush/clear sequence may trigger a late callback. If the callback fires after the mutex is released, the next `ctrl_request` caller gets a premature semaphore signal from the stale transfer — causing it to return before its actual callback fires, reading wrong data.

**Fix**: Add a generation counter to `ctrl_xfer` so callbacks can self-identify as stale, or do a longer drain before releasing the mutex.

### 13. `fb_accumulator` not atomic — fragile single-threaded assumption

**File**: `uac2_host.c:404-406`

`fb_accumulator` is plain `uint32_t` modified in USB callbacks. Safe only because the USB host callback is single-threaded (USB client event task). Not enforced or documented — fragile if URB dispatch model changes.

**Fix**: Make `_Atomic` or add a comment documenting the single-threaded callback assumption.

---

## Low Priority

### 14. Descriptor parser: `desc[2]` read without `len >= 3` check

**File**: `uac2_desc.c:149-176`

`parse_ac_entity` reads `desc[2]` (subtype) before verifying `len >= 3`. A malformed CS_INTERFACE descriptor with `bLength < 3` causes OOB read.

**Fix**: Add `if (len < 3) { offset += len; continue; }` before CS_INTERFACE dispatch.

### 15. Descriptor parser: `bLength = 1` doesn't terminate loop

**File**: `uac2_desc.c:214-216`

The loop guards against `len == 0` but not `len == 1`. A 1-byte descriptor causes byte-by-byte misparse of remaining data — no buffer overflow but massive garbage parse.

**Fix**: Change to `if (len < 2) break;`

### 16. `ep_interval` patch mutates canonical `desc_info` in-place

**File**: `uac2_host.c:1202-1208`

After patching, `uac2_host_device_get_info()` returns the mutated value, not the device's actual descriptor value. Leaks a driver workaround into the public API surface.

**Fix**: Store override in `uac2_stream_t` instead of mutating `desc_info`.

### 17. Duplicate `vid`/`pid` fields on device struct

**File**: `uac2_host.c:89-120`

`dev->vid`/`dev->pid` and `dev->desc_info.vid`/`dev->desc_info.pid` store the same data. Dual ownership — future code could update one and miss the other.

**Fix**: Remove `vid`/`pid` from `struct uac2_host_device`; read from `desc_info` everywhere.

### 18. Kconfig help text for `UAC2_CTRL_XFER_MAX_SIZE` is misleading

**File**: `Kconfig:25`

Help text says "Increase to 512+ if your device has a large configuration descriptor" — but this parameter controls class-specific GET_RANGE responses, not config descriptor fetching. That's `USB_HOST_CONTROL_TRANSFER_MAX_SIZE`.

**Fix**: Clarify help text. Document `USB_HOST_CONTROL_TRANSFER_MAX_SIZE` separately.

### 19. `tone_gen.c` silently mishandles bit depths other than 16 or 24

**File**: `tone_gen.c:29`

The branch at line 42 only checks `if (bit_depth == 24)` — anything else falls to `else` which assumes 16-bit. A caller passing `bit_depth = 32` writes 2-byte samples into 4-byte slots. In practice all callers use 24, but the function signature doesn't enforce this.

**Fix**: Add `assert(bit_depth == 16 || bit_depth == 24)` in `tone_gen_init`.

---

## Summary

| Severity | Count |
|----------|-------|
| High     | 4     |
| Medium   | 9     |
| Low      | 6     |

Items that overlap with `TODO-part2.md`: #3 (section 7), #4 (section 5), #8 (section 5).
