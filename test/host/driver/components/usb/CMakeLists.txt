# Fake USB Host Library: replaces ESP-IDF's `usb` component in this project.
# Public headers and descriptor helpers are ESP-IDF's own, so the driver
# compiles against the real API.
idf_component_register(SRCS "fake_usb_host.c"
                            "fake_esp_timer.c"
                            "${IDF_PATH}/components/usb/usb_helpers.c"
                       INCLUDE_DIRS "${IDF_PATH}/components/usb/include" "include"
                       PRIV_INCLUDE_DIRS "../../../../../ref"
                       REQUIRES freertos esp_timer)
