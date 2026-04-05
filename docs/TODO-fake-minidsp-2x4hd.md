# Building a Faithful miniDSP 2x4 HD Simulator

Everything needed to turn the existing `test_device/` into a near-perfect replica of a real miniDSP 2x4 HD on USB. The goal: test the full composite device scenario (UAC2 audio + HID control simultaneously) without the real hardware.

## What We Have vs What We Need

| Feature | Current Simulator | Real miniDSP | Gap |
|---|---|---|---|
| VID/PID | 0x2752/0x9999 | 0x2752/0x0011 | Change PID |
| bcdDevice | 0x06F2 | 0x06F2 | Match |
| bNumConfigurations | 1 | 2 | Add second config (low priority) |
| Playback IF1 Alt 1 (24-bit) | Yes | Yes | Match |
| Playback IF1 Alt 2 (16-bit) | No | Yes | Add |
| Capture IF2 (24-bit) | No | Yes | Add |
| HID IF3 | No | Yes | Add (Pass 2) |
| DFU IF4 | No | Yes | Skip (no test value) |
| Sample rates | 44.1k, 48k | 44.1k, 48k | Match |
| Feedback format | 16.16 (TinyUSB default) | 10.14 at FS | Fix |
| Capture FU11 (4ch controls) | No | Yes | Add with capture |
| Capture IT1 (Microphone) | No | Yes | Add with capture |
| Capture OT22 (USB Streaming) | No | Yes | Add with capture |
| String descriptors | Partial | Full set | Complete |

## Step 0: Rename test_device/ to simulators/

Before building the new simulator, restructure:

```
test_device/              -> simulators/simple/
(new)                     -> simulators/minidsp-2x4hd/
```

- `simulators/simple/` — minimal UAC2 device for quick driver iteration (current test_device)
- `simulators/minidsp-2x4hd/` — faithful replica for integration testing

Both share the same ESP-IDF + TinyUSB toolchain. The simple one stays as the default for driver development.

Update: top-level README, CLAUDE.md, test_device/README.md (move to simulators/simple/README.md), and any CI/scripts that reference `test_device/`.

## Implementation Plan

One build. Full replica: audio (playback + capture, both alt settings) + HID + all control requests.

### Device Identity
- Change `idProduct` from `0x9999` to `0x0011`
- Update string descriptors: Index 1 = "miniDSP", Index 2 = "2x4HD", Index 3 = "SIM00001"

### Audio: Playback Alt Setting 2 (16-bit)
- Same AS interface 1, alt setting 2
- Format Type I: `bSubslotSize=2`, `bBitResolution=16`
- EP 0x01 OUT: `wMaxPacketSize=196`, Feedback EP 0x81 IN: same as alt 1

### Audio: Capture Interface (IF2)
- Alt 0: zero bandwidth
- Alt 1: 24-bit stereo capture
  - AS General: `bTerminalLink=22`, `bNrChannels=2`, PCM
  - Format Type I: `bSubslotSize=3`, `bBitResolution=24`
  - EP 0x82 IN: async isochronous, `wMaxPacketSize=294`, `bInterval=1`
  - CS Endpoint General: `bLockDelayUnits=2`, `wLockDelay=8`
- Capture path AC entities: IT1 (Microphone), FU11 (master+4ch), OT22 (USB Streaming)
- `tud_audio_tx_done_pre_load_cb()`: fill TX buffer with silence or test pattern

### Audio: Feedback Fix
- 10.14 format at Full Speed (TinyUSB config or manual)
- 48kHz = `0x0C0000`, 44.1kHz = `0x0B0666`

### HID Interface (IF3)

#### Report Descriptor
```
// miniDSP HID: simple vendor-defined 64-byte IN/OUT reports
0x06, 0x00, 0xFF,  // Usage Page (Vendor Defined)
0x09, 0x01,        // Usage (Vendor Usage 1)
0xA1, 0x01,        // Collection (Application)
0x09, 0x02,        //   Usage (Vendor Usage 2)
0x15, 0x00,        //   Logical Minimum (0)
0x26, 0xFF, 0x00,  //   Logical Maximum (255)
0x75, 0x08,        //   Report Size (8 bits)
0x95, 0x40,        //   Report Count (64)
0x81, 0x02,        //   Input (Data, Variable, Absolute)
0x09, 0x03,        //   Usage (Vendor Usage 3)
0x15, 0x00,        //   Logical Minimum (0)
0x26, 0xFF, 0x00,  //   Logical Maximum (255)
0x75, 0x08,        //   Report Size (8 bits)
0x95, 0x40,        //   Report Count (64)
0x91, 0x02,        //   Output (Data, Variable, Absolute)
0xC0               // End Collection
```
Total: 34 bytes. Endpoints: EP 0x83 IN + EP 0x03 OUT, interrupt, 64-byte MPS, 1ms interval.

#### HID Command Handler
Implement all commands minidsp-open uses:

**Must implement:**
| Cmd | Name | Behavior |
|---|---|---|
| 0x31 | ReadHardwareId | Return `[0x31, 0x01, 0x55, 0x0A]` (fw 1.85, hw_id=10) |
| 0x05 | ReadFlash | Return stored EEPROM values |
| 0x14 | ReadFloats | Return stored DSP parameter values |
| 0x42 | SetVolume | Store volume, log |
| 0x17 | SetMute | Store mute state, log |
| 0x25 | SetConfig | Store preset index, return 0xAB |
| 0x34 | SetSource | Store source index, log |
| 0x13 | WriteDSP | Store float at address, log |
| 0x30 | WriteBiquad | Store 5 coefficients at address, log |

**Can stub (return ACK):**
| Cmd | Name |
|---|---|
| 0x03 | Reset |
| 0x04 | WriteFlash |
| 0x19 | BypassFilter |
| 0x39 | FirLoadStart |
| 0x3A | FirLoadData |
| 0x3B | ReloadDspParam |

#### HID Frame Protocol
```c
// Receive: 64 bytes from EP 0x03 OUT
// Byte 0: payload length (including self)
// Bytes 1..N-1: command data
// Byte N: checksum = sum(bytes 0..N-1) & 0xFF
// Remaining bytes: 0xFF padding

// Response: 64 bytes to EP 0x83 IN
// Same framing: length, data, checksum, 0xFF padding
```

#### EEPROM State (Flash Page 0xFF)
Pre-populate with defaults:
```c
static struct {
    uint8_t  current_preset;     // 0
    uint8_t  current_source;     // 2 (USB)
    uint8_t  master_volume;      // 0 (0 dB)
    uint8_t  master_mute;        // 0 (unmuted)
    uint8_t  dirac_bypass;       // 0
    uint16_t serial_number;      // 1234 (actual = 901234)
    uint32_t mod_tokens[4];      // per-preset modification counters
} eeprom_state;
```

#### DSP Parameter State
Pre-populate with flat/passthrough defaults:
```c
// Routing matrix: input 1 → all outputs, input 2 → none
// All gains: 0.0 dB (1.0 as float)
// All PEQ: bypass (b0=1.0, b1=0, b2=0, a1=0, a2=0)
// All delays: 0.0 ms
// Compressors: bypassed
```

### Simulator Test Modes

Add runtime flags (via serial command or GPIO) to trigger fault injection for robustness testing:
- **Config switch delay**: NAK audio for 4-5s after SetConfig command (simulates real device behavior)
- **Empty HID response**: Return zero-length response on demand (simulates broken state from #516)
- **HID NAK**: Reject first N writes, then accept (simulates retry scenario)
- **Command timeout**: Withhold HID response for >10s (tests host timeout recovery)
- **Slow boot**: Delay TinyUSB init by configurable seconds (tests enumeration retry)

### Skip

- **DFU (IF4)**: No test value. minidsp-open doesn't use it. Just descriptor stub bytes.
- **bNumConfigurations=2**: Second config is for HS mode. Our host is FS only.

## Descriptor Byte Map

Full 373-byte config descriptor structure:

```
Offset  Length  Description
──────  ──────  ───────────────────────────────
0       9       Configuration Descriptor
9       8       IAD (Audio Function, IF0-IF1)
17      9       AC Interface 0, Alt 0
26      9       AC Header (bcdADC=0x0200, category=I/O Box)
35      8       Clock Source ID=41
43      8       Clock Selector ID=40 (1 pin → src 41)
51      17      Input Terminal ID=2 (USB Streaming, 2ch, clock=40)
68      18      Feature Unit ID=10 (src=2, master+2ch, mute+volume)
86      12      Output Terminal ID=20 (Speaker, src=10, clock=40)
98      17      Input Terminal ID=1 (Microphone, 2ch, clock=40)
115     26      Feature Unit ID=11 (src=1, master+4ch controls)
141     12      Output Terminal ID=22 (USB Streaming, src=11, clock=40)
──── AS Playback (IF1) ────
153     9       AS Interface 1, Alt 0 (zero bandwidth)
162     9       AS Interface 1, Alt 1 (24-bit stereo)
171     16      AS General (terminal=2, PCM, 2ch)
187     6       Format Type I (3-byte subslot, 24-bit)
193     7       EP 0x01 OUT (async iso, MPS=294, interval=1)
200     8       CS EP General (lock delay=8ms)
208     7       EP 0x81 IN (feedback, MPS=4, interval=4)
215     9       AS Interface 1, Alt 2 (16-bit stereo)
224     16      AS General (terminal=2, PCM, 2ch)
240     6       Format Type I (2-byte subslot, 16-bit)
246     7       EP 0x01 OUT (async iso, MPS=196, interval=1)
253     8       CS EP General (lock delay=8ms)
261     7       EP 0x81 IN (feedback, MPS=4, interval=4)
──── AS Capture (IF2) ────
268     9       AS Interface 2, Alt 0 (zero bandwidth)
277     9       AS Interface 2, Alt 1 (24-bit stereo capture)
286     16      AS General (terminal=22, PCM, 2ch)
302     6       Format Type I (3-byte subslot, 24-bit)
308     7       EP 0x82 IN (async iso, MPS=294, interval=1)
315     8       CS EP General (lock delay=8ms)
──── HID (IF3) ────
323     9       HID Interface 3, Alt 0 (2 endpoints)
332     9       HID Descriptor (bcdHID=0x0110, report desc=34 bytes)
341     7       EP 0x83 IN (interrupt, MPS=64, interval=1)
348     7       EP 0x03 OUT (interrupt, MPS=64, interval=1)
──── DFU (IF4) — SKIP ────
355     9       DFU Interface 4, Alt 0
364     9       DFU Functional Descriptor
──────  ──────
Total:  373 bytes
```

## Source References

### From esp-uac2-host
- `ref/minidsp_2x4hd_descriptors.h` — full 373-byte descriptor (257 captured + 116 reconstructed)
- `test_device/main/main.c` — current simulator (audio-only, the base to extend)
- `docs/ref-research.md` — XMOS firmware behavior, feedback format, bandwidth analysis

### From minidsp-open
- `PROJECT.md` lines 75-250 — complete HID protocol reference (command table, framing, checksums)
- `docs/ref-devices.md` — device registry (hw_id=10, dsp_version=100, capabilities)
- `docs/ref-address-tables.md` — full DSP 100 address map (routing, PEQ, compressor, delay, crossover, FIR)
- `docs/ref-command-pacing.md` — HID timing (3-5ms round-trip, 10ms safe interval)
- `app/MiniDSP Device Console.app/.../HW/10/info.xml` — hw_id 10 metadata
- `app/MiniDSP Device Console.app/.../DSP_Structures/100/DSP_default_parameters.xml` — DSP structure

### From miniDSP hardware dumps
- Arduino USB Host Shield descriptor dump (GitHub: felis/USB_Host_Shield_2.0#594) — confirmed UAC2 at FS
- Linux ALSA stream0 dump of DDRC-24 (same XMOS XU216) — confirmed EP 0x82 capture, async mode
- XMOS sw_usb_audio v6.1 reference firmware — default descriptor templates

## What's Missing / Needs Verification

1. **Exact HID report descriptor bytes** — We have the structure (34-byte vendor-defined, 64-byte IN/OUT) but not the exact bytes from a real device. The report descriptor above is reconstructed from the minidsp-rs/minidsp-open protocol. Could capture with `lsusb -v` on a real device or from Windows USB analyzer.

3. **String descriptor indices 4-13** — We know indices 1-3 (manufacturer, product, serial). The audio interface strings (indices 4, 5, 7, 11, 13 referenced in descriptors) are guesses. Not critical — the driver doesn't use them.

4. **Second configuration descriptor** — `bNumConfigurations=2`. We don't know if config 2 differs from config 1 at FS. Likely identical (HS-only differences). Low priority.

5. **Exact capture FU11 bmaControls** — The `0x0F000000` value (mute+volume+bass+mid+treble) is from XMOS defaults. Real device may differ. Only mute+volume matter for our testing.

6. **DFU functional descriptor exact bytes** — Estimated from XMOS defaults. Doesn't matter since we're skipping DFU.

6. **Feedback endpoint exact timing** — XMOS averages feedback over 128 SOFs (128ms). Our simulator sends feedback immediately on sample rate change. Good enough for driver testing.

None of these gaps block building the simulator. Items 1-2 can be captured from a real device when available but aren't needed for driver testing.

## Real-World Test Scenarios (from minidsp-rs issues)

These are device behaviors reported by real users that we should be able to reproduce or handle. The simulator should support triggering these scenarios for driver robustness testing.

### Must test (driver correctness)

- [ ] **Slow boot enumeration** (#511) — device takes up to 30s after power-on before USB is ready. Simulator: add a configurable startup delay before TinyUSB init. Driver should retry enumeration with backoff, not fail on first attempt.
- [ ] **Transfer errors during config switch** (#383, #282) — device internally reconfigures for 4-5s, audio stream sees errors. Simulator: add a HID command that triggers a "config switch" mode where audio data is NAK'd or returns errors for N seconds. Driver should survive without crashing and resume when errors stop.
- [ ] **Device disconnect during streaming** (already tested) — verified clean shutdown + reconnect. Keep in regression suite.
- [ ] **Long-running stability** (already tested at 100+ sec) — extend to 30+ minutes for release validation.

### Should test (integration robustness)

- [ ] **HID commands during active audio stream** — verify control requests (volume, mute) don't disrupt isochronous transfers. Simulator: already supports this. Test more aggressively with rapid command bursts.
- [ ] **Audio streaming resumes after disruption** — after a simulated config switch disruption (errors for 4-5s), verify the stream recovers automatically without app intervention.
- [ ] **Multiple start/stop cycles** (already tested) — extend to 50+ cycles to check for memory leaks (`heap_caps_get_free_size` before/after).

### Must test (composite device — ezBEQ / measurement use case)

The primary minidsp-open workflow involves simultaneous UAC2 audio + HID control on the same composite device:

- [ ] **HID mute commands during active audio stream** — during a measurement sweep (`POST /play`), minidsp-open mutes/unmutes individual outputs via HID to isolate each sub. This is simultaneous isochronous OUT (audio) + interrupt OUT (HID) on the same USB device. Verify audio doesn't glitch when HID commands fire.
- [ ] **Rapid HID burst during audio stream** — ezBEQ loads filters with ~40 WriteBiquad HID commands in ~120ms. If a measurement sweep is playing (or even idle streaming), this burst must not crash the USB host or corrupt audio. Simulator: trigger a burst of HID commands from a test script while streaming.
- [ ] **USB host channel budget** — ESP32-S3 has 8 host channels. Active channels during measurement: EP0 (control), EP 0x01 OUT (iso audio), EP 0x81 IN (feedback), EP 0x83 IN (HID interrupt), EP 0x03 OUT (HID interrupt) = 5 channels. Verify we stay under 8.

### Nice to test (edge cases from field reports)

- [ ] **Empty/malformed control responses** (#516) — device returns zero-length or garbage after days of operation. Simulator: add a mode that returns empty responses to GET requests. Driver should return an error, not crash.
- [ ] **Source switch while streaming** — user changes audio source away from USB via HID (`0x34 0x00`). Audio stream should get transfer errors and handle gracefully.
- [ ] **Rapid connect/disconnect cycles** — unplug/replug 10+ times in quick succession. Check for resource leaks or hung tasks.

None of these block building the simulator. They define the test matrix for release validation.

## Full Test Matrix (all minidsp-open features)

81 test cases across 4 categories. Current test suite covers 13 of 16 audio-only cases. HID and combined tests require the Pass 2 simulator.

### Audio Streaming (16 tests — need UAC2 driver)

| # | Test | Status |
|---|------|--------|
| 1.1 | 12s sweep playback (48kHz/24-bit/stereo, zero underruns) | Covered (TEST 1 + TEST 5) |
| 1.2 | Sample rate query + set (GET_RANGE, SET_CUR, GET_CUR) | Covered (enumeration) |
| 1.3 | Tone burst at known sample offset (byte count verification) | Not yet |
| 1.4 | startedAtUs timestamp (non-zero, monotonic) | Implemented, not yet in test suite |
| 1.5 | Stream start/stop/restart lifecycle | Covered (TEST 3) |
| 1.6 | Clean stop after playback (no leaked URBs or memory) | Covered (implicit) |
| 1.7 | 44.1kHz streaming with fractional feedback | Covered (TEST 4) |
| 1.8 | Feedback-based adaptive packet sizing | Implemented, verified by sustained streaming |
| 1.9 | 2s ASRC preamble (continuous isochronous, no gaps) | Covered (first 2s of any stream) |
| 1.10 | Ring buffer backpressure (write blocks when full) | Covered (implicit in all stream tests) |
| 1.11 | UAC2 volume control during streaming | Covered (TEST 2) |
| 1.12 | UAC2 mute control during streaming | Covered (TEST 2) |
| 1.13 | Volume range query (GET_RANGE on Feature Unit) | Not yet |
| 1.14 | 16-bit alt setting selection and streaming | Needs alt 2 in simulator (Pass 1) |
| 1.15 | Capture interface (isochronous IN) | Needs capture in simulator (Pass 1). ESP32-S3 FIFO prevents simultaneous TX+RX. |
| 1.16 | Clock validity check during streaming | Covered (enumeration) |

### HID-Only (40 tests — need HID interface, Pass 2)

| # | Test | Priority |
|---|------|----------|
| 2.1 | Composite device enumeration (Audio + HID discovered) | Must |
| 2.2 | ReadHardwareId (cmd 0x31) → hw_id=10, dsp=100 | Must |
| 2.3 | Master status read (ReadFlash 0xFFD8) | Must |
| 2.4 | SetVolume (cmd 0x42) | Must |
| 2.5 | SetMute (cmd 0x17) | Must |
| 2.6 | SetSource (cmd 0x34) | Must |
| 2.7 | SetConfig/preset switch (cmd 0x25, 0xAB response) | Must |
| 2.8 | WriteBiquad PEQ (cmd 0x30, 5x float32) | Must |
| 2.9 | BypassFilter (cmd 0x19) | Must |
| 2.10 | ezBEQ batch load (40 commands in ~120ms) | Must |
| 2.11 | ezBEQ batch clear (24 commands) | Must |
| 2.12 | WriteDSP generic (cmd 0x13) | Should |
| 2.13 | ReadFloats for levels (cmd 0x14) | Must |
| 2.14 | ReadFloats for config read-back (all PEQ coefficients) | Should |
| 2.15 | Level polling at 250ms interval (5s) | Must |
| 2.16 | Level polling doesn't block config writes | Must |
| 2.17 | Modification tokens read (ReadFlash 0xFFC8) | Should |
| 2.18 | WriteFlash + ReadFlash round-trip | Should |
| 2.19-2.20 | Display brightness/idle (cmds 0x1A, 0x1B) | Nice |
| 2.21 | SetDRE (cmd 0x1E) | Nice |
| 2.22 | CopyPreset (cmd 0x27) | Should |
| 2.23 | DiracBypass (cmd 0x3F) | Nice |
| 2.24 | FIR load (cmd 0x39 + 0x3A, multi-packet) | Should |
| 2.25 | Reset (cmd 0x03, 0xAA response) | Should |
| 2.26-2.27 | Measurement mode enter/restore | Must |
| 2.28 | Per-source volume offsets | Nice |
| 2.29 | Serial number read (ReadFlash 0xFFFE) | Should |
| 2.30 | Error response handling (0xFF) | Must |
| 2.31 | Command timeout (simulator withholds response) | Must |
| 2.32 | HID write retry on NAK | Should |
| 2.33-2.40 | Crossover, delay, compressor, gain, mute, polarity writes | Should |

### Combined Audio + HID (10 tests — need both, Pass 2)

| # | Test | Priority |
|---|------|----------|
| 3.1 | HID mute commands during active audio stream | Must |
| 3.2 | Level polling during audio playback (250ms interval) | Must |
| 3.3 | Preset switch during audio (stop → switch → restart) | Must |
| 3.4 | Source switch to USB → open audio → stream | Must |
| 3.5 | ezBEQ 40-command batch during audio stream | Must |
| 3.6 | Full measurement mode cycle (save → flatten → sweep → restore) | Must |
| 3.7 | POST /play end-to-end (HTTP → HID → UAC2 → timestamp) | Must |
| 3.8 | Volume adjust via both HID and UAC2 during playback | Should |
| 3.9 | Config read-back while audio interface claimed (alt 0) | Should |
| 3.10 | WebSocket levels + audio stream simultaneously | Should |

### Device Lifecycle (15 tests)

| # | Test | Status |
|---|------|--------|
| 4.1 | Composite device enumeration (full config descriptor) | Needs Pass 1 |
| 4.2 | Two-stage device ID (enumerate → ReadHardwareId) | Needs Pass 2 |
| 4.3 | USB hotplug detection | Covered |
| 4.4 | USB disconnect during idle | Covered |
| 4.5 | Disconnect during audio streaming | Covered (disconnect test) |
| 4.6 | Disconnect during HID command | Needs Pass 2 |
| 4.7 | Reconnect after disconnect | Covered (reconnect test) |
| 4.8 | Full Speed negotiation | Covered (implicit) |
| 4.9 | 373-byte config descriptor parse | Needs Pass 1 |
| 4.10 | Interface release retry (ESP-IDF #17707) | Covered (logs show retry on every stop) |
| 4.11 | Simultaneous HID + Audio interface claim | Needs Pass 2 |
| 4.12 | Control transfer timeout recovery | Implemented, not yet stress-tested |
| 4.13 | Boot race (host starts before device ready) | Not yet |
| 4.14 | Device standby/wake (disconnect/reconnect cycle) | Covered |
| 4.15 | Rapid connect/disconnect stress (10+ cycles) | Not yet |

### Summary

| Category | Total | Covered Now | After Simulator |
|----------|-------|-------------|-----------------|
| Audio streaming | 16 | 11 | 16 |
| HID-only | 40 | 0 | 40 |
| Combined audio+HID | 10 | 0 | 10 |
| Device lifecycle | 15 | 7 | 15 |
| **Total** | **81** | **18** | **81** |
