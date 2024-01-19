cmake_minimum_required(VERSION 3.13)

add_library(
    homer2_bmp3xx STATIC

    bosch/bmp3_defs.h
    bosch/bmp3.h
    bosch/bmp3.c

    src/homer2_bmp3xx_base.hpp
    src/homer2_bmp3xx_base.cxx
    src/homer2_bmp3xx_sensor.hpp
    src/homer2_bmp3xx_sensor.cxx

    include/homer2_bmp3xx.hpp
    include/homer2_bmp3xx.cxx
)

target_include_directories(
    homer2_bmp3xx PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
    homer2_bmp3xx PRIVATE

    hardware_i2c

    homer2_util
    homer2_logging
    homer2_i2c
)
