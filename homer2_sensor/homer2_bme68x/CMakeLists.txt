cmake_minimum_required(VERSION 3.13)

add_library(
    homer2_bme68x STATIC

    bosch/bme68x_defs.h
    bosch/bme68x.h
    bosch/bme68x.c

    src/homer2_bme68x_base.hpp
    src/homer2_bme68x_base.cxx
    src/homer2_bme68x_sensor.hpp
    src/homer2_bme68x_sensor.cxx

    include/homer2_bme68x.hpp
    include/homer2_bme68x.cxx
)

target_include_directories(
    homer2_bme68x PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
    homer2_bme68x PRIVATE

    hardware_i2c

    homer2_util
    homer2_logging
    homer2_i2c
)
