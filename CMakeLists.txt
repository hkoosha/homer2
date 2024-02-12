cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(homer2 VERSION 0.1 LANGUAGES C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_BOARD pico_w)
set(PICO_CXX_ENABLE_EXCEPTIONS 1)
pico_sdk_init()

add_subdirectory(homer2_base)
add_subdirectory(homer2_sensor)

configure_file(src/homer2_config.h.in src/homer2_config.h)

if ("$ENV{HOMER2_VICTORIA_WRITE_INITIAL_DELAY_MILLIS_DISABLE}" STREQUAL "true")
    set(_HOMER2_VICTORIA_WRITE_INITIAL_DELAY_MILLIS_DISABLE true)
else ()
    set(_HOMER2_VICTORIA_WRITE_INITIAL_DELAY_MILLIS_DISABLE false)
endif ()

add_executable(homer2 homer2.c)
target_compile_definitions(
    homer2 PRIVATE
    WIFI_SSID=\"${WIFI_SSID}\"
    WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
    WIFI_COUNTRY=\"${WIFI_COUNTRY}\"
    HOMER2_VICTORIA_ADDR=\"${HOMER2_VICTORIA_ADDR}\"
    HOMER2_VICTORIA_PORT=\"${HOMER2_VICTORIA_PORT}\"
    HOMER2_VICTORIA_WRITE_INITIAL_DELAY_MILLIS_DISABLE=${_HOMER2_VICTORIA_WRITE_INITIAL_DELAY_MILLIS_DISABLE}
)
target_sources(
    homer2 PRIVATE

    src/homer2_pusher.cpp
    src/homer2_pusher.hpp

    src/homer2_sensor.cpp
    src/homer2_sensor.hpp

    src/homer2_init.cpp
    src/homer2_init.hpp

    src/homer2_main.cpp
    src/homer2_main.h
)

target_include_directories(
    homer2 PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${CMAKE_BINARY_DIR}/src"
)
target_link_libraries(
    homer2 PRIVATE

    pico_cyw43_arch_lwip_threadsafe_background
    pico_stdlib
    hardware_i2c

    homer2_util
    homer2_logging
    homer2_i2c
    homer2_bme68x
    homer2_sht4x
    homer2_sgp40
    homer2_bmp3xx
    homer2_sunrise
    homer2_pmsx00x
)

pico_add_extra_outputs(homer2)
pico_enable_stdio_usb(homer2 1)
pico_enable_stdio_uart(homer2 1)
