#pragma once

#include <iomanip>
#include <iostream>
#include <cstring>

#include <pico/mutex.h>

#define HOMER2_LOG_USE_MUTEX false

namespace homer2::logging {

    namespace internal {

        void enter();

        void exit();

        void print_time(
            uint64_t nowUs
        );

        bool is_logger_enabled(
            const char* tag
        );

    }

    void init();

}

#define HOMER2_USE_COLOR true

#if HOMER2_USE_COLOR
#   define HOMER2_USE_COLOR true
#   define HOMER2_COLOR_RESET "\033[0m"
#   define HOMER2_COLOR_RED "\033[31m"
#   define HOMER2_COLOR_GREEN "\033[32m"
#   define HOMER2_COLOR_GRAY "\033[90m"
#   define HOMER2_COLOR_YELLOW "\033[33m"
#   define HOMER2_COLOR_BLUE "\033[34m"
#   define HOMER2_COLOR_MAGENTA "\033[35m"
#else
#   define HOMER2_COLOR_RESET ""
#   define HOMER2_COLOR_RED ""
#   define HOMER2_COLOR_GREEN ""
#   define HOMER2_COLOR_GRAY ""
#   define HOMER2_COLOR_YELLOW ""
#   define HOMER2_COLOR_BLUE ""
#   define HOMER2_COLOR_MAGENTA ""
#endif

#define HOMER2_LOGGER_IS_TAG_ENABLED homer2::logging::internal::is_logger_enabled

#define HOMER2_LOGGER(COLOR, TAG, LEVEL, X) \
    do { \
        if(!(HOMER2_LOGGER_IS_TAG_ENABLED(TAG))) break; \
        homer2::logging::internal::enter(); \
        const auto homer2_logging_original_flags = std::cout.flags(); \
        const auto homer2_logging_original_width = std::cout.width(); \
        const auto homer2_logging_original_fill = std::cout.fill(); \
        std::cout << HOMER2_COLOR_BLUE << '['; \
        homer2::logging::internal::print_time(time_us_64()); \
        std::cout << "]" << (COLOR) << '[' << (LEVEL) << ']' \
                  << HOMER2_COLOR_GREEN << '[' << std::setw(10) << std::setfill('.') << (TAG) << "]: "; \
        std::cout.flags(homer2_logging_original_flags); \
        std::cout.width(homer2_logging_original_width); \
        std::cout.fill(homer2_logging_original_fill); \
        std::cout << (COLOR) << X << std::endl << HOMER2_COLOR_RESET; \
        std::cout.flags(homer2_logging_original_flags); \
        std::cout.width(homer2_logging_original_width); \
        std::cout.fill(homer2_logging_original_fill); \
        homer2::logging::internal::exit(); \
    } while(false)

#ifndef HOMER2_INFO_ON
#   define HOMER2_INFO_ON true
#endif
#ifndef HOMER2_WARN_ON
#   define HOMER2_WARN_ON true
#endif
#ifndef HOMER2_ERROR_ON
#   define HOMER2_ERROR_ON true
#endif
#ifndef HOMER2_DEBUG_LEVEL
#    define HOMER2_DEBUG_LEVEL 3
#endif

#define DEBUG_ENABLED_AT_LEVEL(LEVEL) (LEVEL) <= (HOMER2_DEBUG_LEVEL)


#if HOMER2_DEBUG_LEVEL >= 0
#   define HOMER2_LOGGER_DEBUG_0(TAG, X) HOMER2_LOGGER(HOMER2_COLOR_GRAY, TAG, "DG0", X)
#else
#   define HOMER2_LOGGER_DEBUG_0(TAG, X) do { } while(false)
#endif

#if HOMER2_DEBUG_LEVEL >= 1
#   define HOMER2_LOGGER_DEBUG_1(TAG, X) HOMER2_LOGGER(HOMER2_COLOR_GRAY, TAG, "DG1", X)
#else
#   define HOMER2_LOGGER_DEBUG_1(TAG, X) do { } while(false)
#endif

#if HOMER2_DEBUG_LEVEL >= 2
#   define HOMER2_LOGGER_DEBUG_2(TAG, X) HOMER2_LOGGER(HOMER2_COLOR_GRAY, TAG, "DG2", X)
#else
#   define HOMER2_LOGGER_DEBUG_2(TAG, X) do { } while(false)
#endif

#if HOMER2_DEBUG_LEVEL >= 3
#   define HOMER2_LOGGER_DEBUG_3(TAG, X) HOMER2_LOGGER(HOMER2_COLOR_GRAY, TAG, "DG3", X)
#else
#   define HOMER2_LOGGER_DEBUG_3(TAG, X) do { } while(false)
#endif

#if HOMER2_DEBUG_LEVEL >= 4
#   define HOMER2_LOGGER_DEBUG_4(TAG, X) HOMER2_LOGGER(HOMER2_COLOR_GRAY, TAG, "DG4", X)
#else
#   define HOMER2_LOGGER_DEBUG_4(TAG, X) do { } while(false)
#endif

#if HOMER2_DEBUG_LEVEL >= 5
#   define HOMER2_LOGGER_DEBUG_5(TAG, X) HOMER2_LOGGER(HOMER2_COLOR_GRAY, TAG, "DG5", X)
#else
#   define HOMER2_LOGGER_DEBUG_5(TAG, X) do { } while(false)
#endif

#if HOMER2_DEBUG_LEVEL >= 6
#   define HOMER2_LOGGER_DEBUG_6(TAG, X) HOMER2_LOGGER(HOMER2_COLOR_GRAY, TAG, "DG6", X)
#else
#   define HOMER2_LOGGER_DEBUG_6(TAG, X) do { } while(false)
#endif

#define D(LEVEL, TAG, X) HOMER2_LOGGER_DEBUG_##LEVEL(TAG, X)

#if HOMER2_INFO_ON
#   define I(TAG, X) HOMER2_LOGGER(HOMER2_COLOR_RESET, TAG, "INF", X)
#else
#   define I(TAG, X) do { } while(false)
#endif

#if HOMER2_WARN_ON
#   define W(TAG, X) HOMER2_LOGGER(HOMER2_COLOR_YELLOW, TAG, "WRN", X)
#else
#   define W(TAG, X) do { } while(false)
#endif

#if HOMER2_ERROR_ON
#   define E(TAG, X) HOMER2_LOGGER(HOMER2_COLOR_RED, TAG, "ERR", X)
#else
#   define E(TAG, X) do { } while(false)
#endif
