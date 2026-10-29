# Runs the UAC2 driver on the ESP-IDF Linux target against a fake USB Host
# Library (components/usb). Build and run with ./run.sh.
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../components/usb_host_uac2")
set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# run.sh passes -DUAC2_SANITIZE=1 when the host compiler supports it
if(UAC2_SANITIZE)
    idf_build_set_property(COMPILE_OPTIONS "-fsanitize=address,undefined" APPEND)
    idf_build_set_property(COMPILE_OPTIONS "-fno-omit-frame-pointer" APPEND)
    idf_build_set_property(LINK_OPTIONS "-fsanitize=address,undefined" APPEND)
endif()

project(uac2_host_driver_test)
