idf_component_register(SRCS "test_driver.c"
                       PRIV_INCLUDE_DIRS "../../../../ref"
                       REQUIRES usb_host_uac2 usb unity freertos esp_timer)
