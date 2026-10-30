# TODO UAC1 Devices (Coexistence, Not Support)

Updated: 2026-10-29

This file records why UAC1 support is not planned for this driver, and the smaller work that would help applications that need both UAC1 and UAC2 devices.

## Decision

Do not add UAC1 to `usb_host_uac2`.

- Espressif already ships a UAC1 host driver, [`usb_host_uac`](https://components.espressif.com/components/espressif/usb_host_uac) (esp-usb, Apache-2.0). It is UAC1-only; esp-usb issue #173 asks Espressif for UAC2. The two drivers complement each other.
- UAC1 would touch device discovery and descriptor parsing, the code the miniDSP path depends on, for little gain: different descriptors, sample rate set per endpoint instead of on a clock entity, different feature-unit request layout.
- The validated devices (miniDSP 2x4 HD) and the main consumer (minidsp-open) are UAC2.

## Who Asked

- rbouteiller/airplay-esp32 PR #89 (closed, 2026-06): the contributor wrote his own UAC1+UAC2 host output instead of reusing this driver, citing UAC1 support (Bose QC Ultra 2 is UAC1), a queue-based buffer, and testing on several headsets. He continued in `wasdwasd0105/airplay-esp32-usb`.
- easymcucourse/esp32_music_player: runs this driver (vendored v0.1.2) for a Synaptics CX31993 and Espressif's UAC1 driver for a Creative Sound Blaster PLAY! 3, side by side.

## What Would Help Instead

- [ ] A short guide (component README section) for running both drivers in one application:
  - both register as USB Host clients and both see every new device;
  - this driver only reports and parses UAC2 interfaces (`bInterfaceProtocol` 0x20): AudioStreaming alts of other protocols are skipped, and the AudioControl interface, header and units come from the first protocol-0x20 AudioControl interface (`uac2_desc.c` `uac2_parse_config_descriptor()`), so a trailing UAC1 AC interface (USB-MIDI) is ignored. Only the first UAC2 audio function is used: its streaming interfaces are those inside the IAD holding its AC interface (without IADs, those before a second UAC2 AC interface), and a second UAC2 function is not reported;
  - Espressif's driver's connect callback also reports UAC2 streaming interfaces (the music player filters by VID/PID before opening, `main/sb_play3_output.c`), so the application must open only UAC1 devices from it;
  - the format and lifecycle code the application writes twice (open, pick a format, start, stop, close with retry, unplug handling) and how to share it.
- [ ] Optionally, an example that plays to whichever kind of device is plugged in, using both drivers.
- [ ] Check whether the two drivers' event tasks, priorities and the USB Host Library task interact badly (both call `usb_host_client_handle_events()` on their own clients; the task-priority rule in `uac2_host.h` applies to this driver's task).

## Out of Scope

- Implicit feedback (asynchronous playback endpoints without a feedback endpoint, timed from a capture stream). Rare on UAC2 devices; not tied to UAC1 but listed here because the airplay fork's headsets raised the question of devices without explicit feedback.
