idf_component_register(SRCS "main.c" "tone_gen.c"
                       INCLUDE_DIRS "."
                       PRIV_INCLUDE_DIRS "../ref"
                       PRIV_REQUIRES uac2_host usb)
