idf_component_register(SRCS "main.c" "tone_gen.c"
                       INCLUDE_DIRS "."
                       PRIV_INCLUDE_DIRS "../ref"
                       PRIV_REQUIRES usb_host_uac2 usb esp_timer)
