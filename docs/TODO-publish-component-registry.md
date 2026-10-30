# ESP Component Registry

Published as [`averyy/usb_host_uac2`](https://components.espressif.com/components/averyy/usb_host_uac2) (first release 0.1.3, 2026-10-29). The component lives in `components/usb_host_uac2`, named after the registry component like Espressif's own `usb_host_<class>` drivers, and the example depends on `averyy/usb_host_uac2` with an `override_path` (stripped when a project is created from the registry).

## Releasing a version

1. Bump the version in `components/usb_host_uac2/idf_component.yml`, `UAC2_HOST_VER_*` in `components/usb_host_uac2/include/usb/uac2_host.h` and `PROJECT_VER` in the root `CMakeLists.txt`.
2. Add a dated section to `components/usb_host_uac2/CHANGELOG.md` (Keep a Changelog; list anything existing callers must change under "Breaking Changes").
3. Commit, push to `main` and wait for CI.
4. Tag and release: `git tag -a vX.Y.Z -m vX.Y.Z && git push origin vX.Y.Z`, then `gh release create vX.Y.Z --title vX.Y.Z --notes-file <CHANGELOG section>`.
5. Publish: `gh workflow run publish-component.yml --ref main -f namespace=averyy -f execute_upload=true`. The workflow builds the example, then uploads with GitHub OIDC. The registry trusts `Averyy/esp-uac2-host`, workflow `publish-component.yml`, branch `main` (set under Permissions -> averyy -> usb_host_uac2 -> Trusted Uploaders). A version can be uploaded only once.
6. Check: the registry page, `idf.py add-dependency "averyy/usb_host_uac2^X.Y.Z"` in a fresh project, and `idf.py create-project-from-example "averyy/usb_host_uac2^X.Y.Z:basic_playback"`.

`compote component pack --name usb_host_uac2` (in a copy of the component folder) shows exactly what will ship; `files.exclude` in the manifest keeps local build output out.

## Open

- [x] Offered the driver on [espressif/esp-usb#173](https://github.com/espressif/esp-usb/issues/173#issuecomment-6510933805) (2026-10-29); if Espressif wants a PR, title it `USB-173: ...`
- [ ] Announce: ESP32 forum, r/esp32
- [ ] Tested-device list in the README as reports come in
