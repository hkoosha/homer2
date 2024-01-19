cmake_minimum_required(VERSION 3.13)

add_library(
    homer2_util STATIC

    homer2_util.hpp
    homer2_util.cxx
)

target_include_directories(
    homer2_util PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(
    homer2_util PRIVATE

    pico_stdlib
)
