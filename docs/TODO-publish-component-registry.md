# Publish to ESP Component Registry

Publishing `usb_host_uac2` on [components.espressif.com](https://components.espressif.com). Driver is complete and hardware-verified — these are the remaining packaging tasks.

---

## 1. Component Packaging

- [x] `idf_component.yml` created
- [ ] Component-level `README.md` for the registry page (usage, API overview, quick start, hardware requirements)
- [x] Verify `CMakeLists.txt` uses `idf_component_register()` correctly for standalone consumption

## 2. Examples Directory

The registry auto-discovers `examples/` and displays them on the component page.

- [ ] `components/uac2_host/examples/basic_playback/` — minimal: enumerate, open, stream 48kHz sine wave
- [ ] Each example needs its own `CMakeLists.txt`, `main/`, and `README.md`
- [ ] Examples use `idf_component.yml` with dependency on the component (standalone builds)

## 3. GitHub Repo Prep

- [ ] Decide on repo name and GitHub namespace
- [ ] Clean up repo: ensure `ref/`, `simulators/`, `build/`, `sdkconfig` are excluded or clearly separated
- [ ] Top-level `README.md` with Component Registry install instructions
- [ ] Component Registry badge in README
- [ ] Tag conventions: `v1.0.0`, `v1.1.0`, etc. (semver, `v` prefix)

## 4. CI / GitHub Actions

- [ ] Build CI (component + examples against ESP-IDF v5.4)
- [ ] Auto-publish workflow for tagged releases (`espressif/upload-components-ci-action@v2`)
- [ ] Test against [staging registry](https://components-staging.espressif.com) first

## 5. Registry Account Setup

- [ ] Create account on components.espressif.com (GitHub OAuth)
- [ ] Claim namespace
- [ ] Verify OIDC token permissions for GitHub Actions uploads

## 6. Post-Publish

- [ ] Verify component page renders correctly
- [ ] Test install from fresh project: `idf.py add-dependency "namespace/usb_host_uac2^1.0.0"`
- [ ] Announce: ESP32 forum, Reddit r/esp32
- [ ] Update `llms.txt` with install instructions
- [ ] Tested device compatibility list in README

---

## Naming

Espressif convention: `usb_host_<class>` (`usb_host_cdc_acm`, `usb_host_hid`, `usb_host_uac`).

**Component name: `usb_host_uac2`**
