cmake_minimum_required(VERSION 3.13)

add_library(
    homer2_sunrise STATIC

    src/homer2_sunrise_base.hpp
    src/homer2_sunrise_base.cxx
    src/homer2_sunrise_sensor.hpp
    src/homer2_sunrise_sensor.cxx

    include/homer2_sunrise.hpp
    include/homer2_sunrise.cxx
)

target_include_directories(
    homer2_sunrise PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
    homer2_sunrise PRIVATE

    hardware_i2c

    homer2_util
    homer2_logging
    homer2_i2c
)
