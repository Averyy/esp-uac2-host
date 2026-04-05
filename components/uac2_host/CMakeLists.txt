idf_component_register(SRCS "uac2_desc.c" "uac2_host.c"
                       INCLUDE_DIRS "include"
                       REQUIRES usb
                       PRIV_REQUIRES esp_ringbuf esp_timer)
