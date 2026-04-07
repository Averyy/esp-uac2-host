# Publish to ESP Component Registry

Publishing `usb_host_uac2` on [components.espressif.com](https://components.espressif.com). Driver is complete and hardware-verified — these are the remaining packaging tasks.

---

## 1. Component Packaging

- [x] `idf_component.yml` created
- [x] Component-level `README.md` for the registry page (usage, API overview, quick start, hardware requirements)
- [x] Verify `CMakeLists.txt` uses `idf_component_register()` correctly for standalone consumption

## 2. Examples Directory

The registry auto-discovers `examples/` and displays them on the component page.

- [x] `components/uac2_host/examples/basic_playback/` — minimal: enumerate, open, stream 48kHz sine wave
- [x] Each example needs its own `CMakeLists.txt`, `main/`, and `README.md`
- [x] Examples use `idf_component.yml` with dependency on the component for standalone local builds (`path` now, registry dependency after publish)

## 3. GitHub Repo Prep

- [x] Final namespace chosen: `averyy` (matching GitHub username)
- [x] Clean up repo: ignore generated `build/`, `sdkconfig`, `dependencies.lock`, and example artifacts; keep `ref/` and `simulators/` clearly separated
- [x] Top-level `README.md` with local component usage and planned Component Registry install instructions
- [ ] Component Registry badge in README
- [ ] Tag conventions: `v1.0.0`, `v1.1.0`, etc. (semver, `v` prefix)

## 4. CI / GitHub Actions

- [x] Build CI (root project + component example against ESP-IDF v5.4)
- [x] Manual/gated publish workflow scaffold added (`espressif/upload-components-ci-action@v2`) but not enabled for automatic release
- [ ] Test against [staging registry](https://components-staging.espressif.com) first

## 5. Registry Account Setup

- [x] Create account on components.espressif.com (GitHub OAuth)
- [x] Confirm default namespace exists: `averyy`
- [ ] Verify OIDC token permissions for GitHub Actions uploads
  Add `Averyy/esp-uac2-host` as a trusted uploader in the ESP Component Registry UI for the `Publish Component` workflow (`publish-component.yml`) before enabling workflow-based uploads.

## 6. Post-Publish

- [ ] Verify component page renders correctly
- [ ] Test install from fresh project: `idf.py add-dependency "averyy/usb_host_uac2^1.0.0"`
- [ ] Announce: ESP32 forum, Reddit r/esp32
- [ ] Update `llms.txt` with install instructions
- [ ] Tested device compatibility list in README

---

## Naming

Espressif convention: `usb_host_<class>` (`usb_host_cdc_acm`, `usb_host_hid`, `usb_host_uac`).

**Component name: `usb_host_uac2`**
**Planned package name: `averyy/usb_host_uac2`**
