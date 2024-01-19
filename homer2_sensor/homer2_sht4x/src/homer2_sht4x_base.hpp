#pragma once

#include <cstdint>
#include <ostream>

namespace homer2::sensor::sht4x {

    namespace internal {

        extern const char* const TAG;

        enum class SHT4xTask : uint8_t {
            idle,
            read_serial_number,
            measure,
            reset,
        };

        std::ostream& operator<<(
            std::ostream& out,
            SHT4xTask value
        );

    }


    enum class [[maybe_unused]] Precision : uint8_t {
        low = 0,
        medium,
        high,
    };

    std::ostream& operator<<(
        std::ostream& out,
        Precision value
    );


    enum class [[maybe_unused]] HeaterConf : uint8_t {
        off = 0,
        low_100ms,
        low_1000ms,
        medium_100ms,
        medium_1000ms,
        high_100ms,
        high_1000ms,
    };

    std::ostream& operator<<(
        std::ostream& out,
        HeaterConf value
    );

}
