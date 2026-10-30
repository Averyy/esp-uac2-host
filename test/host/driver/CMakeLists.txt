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

# Darwin longjmp() restores the signal mask process-wide, unblocking SIGALRM
# on suspended FreeRTOS pthreads after a Unity assertion fails. Only the
# running task may receive the POSIX port's tick signal. Use Unity's hooks
# with the jump pair that leaves signal masks alone, in Unity and its users.
if(APPLE)
    idf_component_get_property(unity_lib unity COMPONENT_LIB)
    target_compile_options(${unity_lib} PUBLIC
        "-DUNITY_TEST_PROTECT()=(_setjmp(Unity.AbortFrame)==0)"
        "-DUNITY_TEST_ABORT()=_longjmp(Unity.AbortFrame,1)")
endif()
