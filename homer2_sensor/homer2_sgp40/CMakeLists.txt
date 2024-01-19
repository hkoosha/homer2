cmake_minimum_required(VERSION 3.13)

add_library(
    homer2_sgp40 STATIC

    sensirion/sensirion_voc_algorithm.h
    sensirion/sensirion_voc_algorithm.c
    sensirion/sensirion_arch_config.h

    src/homer2_sgp40_base.hpp
    src/homer2_sgp40_base.cxx
    src/homer2_sgp40_sensor.hpp
    src/homer2_sgp40_sensor.cxx

    include/homer2_sgp40.hpp
    include/homer2_sgp40.cxx
)

target_include_directories(
    homer2_sgp40 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
    homer2_sgp40 PRIVATE

    hardware_i2c

    homer2_util
    homer2_logging
    homer2_i2c
)
