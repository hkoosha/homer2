#pragma once

#include <cstdint>
#include <ostream>

namespace homer2::sensor::sunrise::internal {

    extern const char* const TAG;

    enum class SunriseTask : uint8_t {
        idle,
        measure,
        reset,
    };

    std::ostream& operator<<(
        std::ostream& out,
        SunriseTask value
    );

}
