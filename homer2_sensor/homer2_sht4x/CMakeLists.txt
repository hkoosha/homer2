cmake_minimum_required(VERSION 3.13)

add_library(
    homer2_sht4x STATIC

    src/homer2_sht4x_base.hpp
    src/homer2_sht4x_base.cxx
    src/homer2_sht4x_sensor.hpp
    src/homer2_sht4x_sensor.cxx

    include/homer2_sht4x.hpp
    include/homer2_sht4x.cxx
)

target_include_directories(
    homer2_sht4x PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
    homer2_sht4x PRIVATE

    hardware_i2c

    homer2_util
    homer2_logging
    homer2_i2c
)
