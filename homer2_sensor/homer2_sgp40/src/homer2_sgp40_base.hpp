#pragma once

#include <cstdint>
#include <ostream>

namespace homer2::sensor::sgp40::internal {

    extern const char* const TAG;

    enum class SGP40Task : uint8_t {
        idle,
        serial_number,
        feature_set,
        self_test,
        heater_off,
        measure,
        reset,
    };

    std::ostream& operator<<(
        std::ostream& out,
        SGP40Task value
    );

}
