cmake_minimum_required(VERSION 3.13)

add_library(
    homer2_logging STATIC

    homer2_logging.hpp
    homer2_logging.cxx
)

target_include_directories(
    homer2_logging PUBLIC

    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(
    homer2_logging PRIVATE

    pico_stdlib
)
