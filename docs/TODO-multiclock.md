# TODO Multi-Clock Support

Updated: 2026-04-09

This file tracks the remaining engineering work required to support valid UAC2 devices that expose more than one clock tree.

## Current Limitation

The driver is still effectively single-clock.

- `resolve_clock_source()` resolves one device-global `dev->clock_source_id`
- clock-control APIs reuse that one entity for every opened interface
- `device_start()` and `device_resume()` also reuse that one entity for sample-rate setup

That behavior is correct for the currently validated miniDSP path, but it is not correct for devices with:

- separate playback and capture clocks
- multiple output paths with distinct clock sources
- selectors or multipliers that resolve differently per terminal/interface

## Goal

Make clock control interface-specific instead of device-global, while preserving current behavior for single-clock devices.

Desired end state:

- each opened AS interface resolves its own effective clock source
- all clock-control APIs operate on that interface's resolved clock entity
- playback and capture interfaces can coexist as independent control paths even if the ESP32-S3 still rejects simultaneous streaming
- single-clock devices continue to work unchanged

## Proposed Design

### 1. Resolve clocks per interface

Move clock resolution from a device-global field to interface-scoped state.

- keep topology parsing in `uac2_desc.c`
- resolve the effective clock from the interface's terminal link during `uac2_host_device_open()`
- store the resolved clock source ID on `uac2_iface_t`
- keep the device-global clock data only as parsed descriptor info, not as the runtime control target

### 2. Route all clock APIs through the interface

Audit every path that currently uses `dev->clock_source_id` and switch it to the interface-scoped clock ID.

Expected callers:

- `uac2_host_device_get_sample_rate()`
- `uac2_host_device_set_sample_rate()`
- `uac2_host_device_get_sample_rate_range()`
- `uac2_host_device_get_clock_valid()`
- startup sample-rate application in `uac2_host_device_start()`
- resume sample-rate application in `uac2_host_device_resume()`

### 3. Keep per-interface runtime state coherent

The interface should own the runtime state that is logically tied to its clock.

- resolved clock source ID
- selected sample rate
- any cached range data that is clock-specific

Do not silently share mutable clock state across interfaces unless the topology truly resolves to the same source ID.

### 4. Preserve shared-device safety

Two interfaces may still resolve to the same underlying clock source. The refactor must not assume that different interfaces always mean different clocks.

Handle these cases cleanly:

- playback + capture resolve to the same clock source
- playback + capture resolve to different clock sources
- multiple interfaces resolve through selectors or multipliers to the same leaf clock source

## Implementation Plan

1. Add interface-scoped clock metadata to `uac2_iface_t`.
2. Resolve the effective clock source from the opened interface's terminal link.
3. Replace all runtime use of `dev->clock_source_id` with the interface-scoped field.
4. Update logging so the resolved clock is visible per interface.
5. Remove or demote the device-global runtime clock field once nothing depends on it.
6. Update public docs to remove the single-clock restriction only after validation on real multi-clock hardware.

## Validation Plan

Single-clock regression coverage:

- repo root harness on the real miniDSP 2x4 HD
- simulator reruns for normal, no-feedback, and channel-only profiles

Multi-clock-specific coverage still needed:

- enumerate a real or simulated device with separate playback and capture clocks
- verify playback clock queries/setters target the playback path's clock
- verify capture clock queries/setters target the capture path's clock
- verify start/resume reapply the correct clock per interface
- verify shared-clock topologies still behave like the current single-clock path

## Exit Criteria

Multi-clock support is ready only when all of the following are true:

- no runtime clock-control path depends on a device-global resolved clock
- each interface can report which clock source it resolved to
- single-clock hardware still passes the existing harness unchanged
- at least one genuine multi-clock topology is validated end-to-end
