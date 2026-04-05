# Reference Sources

Read-only reference files for developing the esp-uac2-host driver.
These are **not compiled** as part of this project. Do not modify them.

## Files

### Espressif UAC1 Host Driver (Apache-2.0)

Baseline architecture to fork for UAC2. From `espressif/esp-usb` v1.3.3+.

| File | Source |
|------|--------|
| `espressif-uac1/uac_host.c` | https://github.com/espressif/esp-usb/blob/master/host/class/uac/usb_host_uac/uac_host.c |
| `espressif-uac1/uac_descriptors.c` | https://github.com/espressif/esp-usb/blob/master/host/class/uac/usb_host_uac/uac_descriptors.c |
| `espressif-uac1/include/usb/uac_host.h` | https://github.com/espressif/esp-usb/blob/master/host/class/uac/usb_host_uac/include/usb/uac_host.h |
| `espressif-uac1/include/usb/uac.h` | https://github.com/espressif/esp-usb/blob/master/host/class/uac/usb_host_uac/include/usb/uac.h |

### USBX Audio 2.0 Definitions (MIT)

Complete UAC2 descriptor structs and constants. From `eclipse-threadx/usbx`.

| File | Source |
|------|--------|
| `usbx-uac2/ux_class_audio20.h` | https://github.com/eclipse-threadx/usbx/blob/master/common/core/inc/ux_class_audio20.h |

### CherryUSB Audio Definitions (Apache-2.0)

UAC1 + UAC2 descriptor structs and constants. From `cherry-embedded/CherryUSB`.

| File | Source |
|------|--------|
| `cherryusb-uac2/usb_audio.h` | https://github.com/cherry-embedded/CherryUSB/blob/master/class/audio/usb_audio.h |

## Downloaded

2026-04-05
