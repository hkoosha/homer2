cmake_minimum_required(VERSION 3.13)

add_library(
    homer2_i2c STATIC

    homer2_i2c.hpp
    homer2_i2c.cxx
)

target_include_directories(
    homer2_i2c PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(
    homer2_i2c PRIVATE

    hardware_i2c

    homer2_util
    homer2_logging
)
