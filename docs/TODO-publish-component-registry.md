# Publish to ESP Component Registry

Publishing `uac2_host` as a standalone component on [components.espressif.com](https://components.espressif.com). This is the only UAC2 host driver in the ESP-IDF ecosystem — no competing implementations exist.

**Prerequisite:** Complete real-device testing (`TODO-real-device-testing.md`) and remaining TODO items (`TODO.md`) first. Easy cleanup is done.

---

## 1. Component Packaging

- [x] Create `components/uac2_host/idf_component.yml`:
  ```yaml
  version: "1.0.0"
  description: "USB Audio Class 2.0 (UAC2) host driver for ESP32-S3"
  url: "https://github.com/yourusername/esp-uac2-host"
  repository: "https://github.com/yourusername/esp-uac2-host.git"
  license: "MIT"
  targets:
    - esp32s3
  tags:
    - usb
    - audio
    - uac2
    - host
    - driver
    - dac
  dependencies:
    idf:
      version: ">=5.4"
  ```
- [ ] Copy `LICENSE` into `components/uac2_host/`
- [ ] Add `components/uac2_host/README.md` (component-level README for the registry page — usage, API overview, quick start, hardware requirements)
- [ ] Verify `CMakeLists.txt` in `components/uac2_host/` uses `idf_component_register()` correctly

## 2. Code Quality for Release

These are the minimum items from `TODO.md` that should land before v1.0:

- [ ] Install/uninstall lifecycle (`uac2_host_install()` / `uac2_host_uninstall()`) — matches Espressif class driver pattern, required for discoverability and adoption
- [ ] Kconfig for tunable constants (URB count, timeout, error limit)
- [ ] Debug print function (`uac2_host_device_printf_info()`)
- [ ] Component registry packaging (item 11 in `TODO.md`)
- [ ] Clean build with `-Werror -Wextra` and no warnings
- [ ] Doxygen comments on all public functions in `uac2_host.h` (most are already there)

## 3. Examples Directory

The registry auto-discovers an `examples/` directory and displays them on the component page.

- [ ] Create `components/uac2_host/examples/basic_playback/` — minimal example: enumerate, open, stream 48kHz sine wave
- [ ] Each example needs its own `CMakeLists.txt`, `main/`, and `README.md`
- [ ] Examples should use `idf_component.yml` with a dependency on the component itself (so they build standalone)

## 4. GitHub Repo Prep

- [ ] Decide on repo name and GitHub namespace (e.g. `esp-uac2-host` or `usb_host_uac2`)
- [ ] Clean up repo: ensure `ref/`, `simulators/`, `build/`, `sdkconfig` are excluded or clearly separated from the component
- [ ] Update top-level `README.md` with Component Registry install instructions:
  ```
  idf.py add-dependency "namespace/usb_host_uac2^1.0.0"
  ```
- [ ] Add Component Registry badge to README
- [ ] Tag conventions: `v1.0.0`, `v1.1.0`, etc. (semver, prefixed with `v`)

## 5. CI / GitHub Actions

- [ ] Add build CI (build the component + examples against ESP-IDF v5.4)
- [ ] Add auto-publish workflow for tagged releases:
  ```yaml
  name: Upload Component
  on:
    push:
      tags: ['v*']
  jobs:
    upload:
      runs-on: ubuntu-latest
      permissions:
        id-token: write
        contents: read
      steps:
        - uses: actions/checkout@v4
        - uses: espressif/upload-components-ci-action@v2
          with:
            directories: "components/usb_host_uac2"
            namespace: "yournamespace"
  ```
- [ ] Test the upload workflow against the [staging registry](https://components-staging.espressif.com) first

## 6. Registry Account Setup

- [ ] Create account on [components.espressif.com](https://components.espressif.com) (GitHub OAuth)
- [ ] Claim namespace
- [ ] Verify OIDC token permissions work for GitHub Actions uploads

## 7. Post-Publish

- [ ] Verify component page renders correctly (README, examples, API docs)
- [ ] Test install from a fresh ESP-IDF project: `idf.py create-project test && cd test && idf.py add-dependency "namespace/usb_host_uac2^1.0.0" && idf.py build`
- [ ] Announce: ESP32 forum, Reddit r/esp32, relevant GitHub issues/discussions
- [ ] Update `llms.txt` with component registry install instructions
- [ ] Add tested device compatibility list to README (miniDSP 2x4 HD, plus any others verified)

---

## Naming Decision

The Espressif convention for USB host class drivers is `usb_host_<class>`:
- `usb_host_cdc_acm`
- `usb_host_hid`
- `usb_host_msc`
- `usb_host_uvc`
- `usb_host_uac` (UAC1)

**Recommended component name: `usb_host_uac2`** — follows the pattern exactly. The repo can stay `esp-uac2-host` or be renamed to match.
