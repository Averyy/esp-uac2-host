idf_component_register(SRCS "main.c" "uac2_desc.c" "uac2_host.c" "tone_gen.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES usb esp_ringbuf)
