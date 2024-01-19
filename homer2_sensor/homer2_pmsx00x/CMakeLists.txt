cmake_minimum_required(VERSION 3.13)

add_library(
    homer2_pmsx00x STATIC

    src/homer2_pmsx00x_base.hpp
    src/homer2_pmsx00x_base.cxx
    src/homer2_pmsx00x_sensor.hpp
    src/homer2_pmsx00x_sensor.cxx

    include/homer2_pmsx00x.hpp
    include/homer2_pmsx00x.cxx
)

target_include_directories(
    homer2_pmsx00x PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(
    homer2_pmsx00x PRIVATE

    hardware_uart

    homer2_util
    homer2_logging
)
