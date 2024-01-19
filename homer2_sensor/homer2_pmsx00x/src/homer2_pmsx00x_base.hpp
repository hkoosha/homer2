#pragma once

#include <cstdint>
#include <ostream>

namespace homer2::sensor::pmsx00x::internal {

    extern const char* const TAG;

    enum class PMSx00xTask : uint8_t {
        idle,
        measure,
    };

    std::ostream& operator<<(
        std::ostream& out,
        PMSx00xTask value
    );

}
