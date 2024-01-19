#pragma once

#include <cstdint>
#include <ostream>

#include "../bosch/bme68x_defs.h"

namespace homer2::sensor::bme68x {

    namespace internal {

        extern const char* const TAG;

        enum class BME68xTask : uint8_t {
            idle,
            measure,
        };

        std::ostream& operator<<(
            std::ostream& out,
            BME68xTask value
        );

    }


    enum class [[maybe_unused]] BME68xIirFilterSize : uint8_t {
        off = BME68X_FILTER_OFF,
        x1 = BME68X_FILTER_SIZE_1,
        x3 = BME68X_FILTER_SIZE_3,
        x7 = BME68X_FILTER_SIZE_7,
        x15 = BME68X_FILTER_SIZE_15,
        x31 = BME68X_FILTER_SIZE_31,
        x63 = BME68X_FILTER_SIZE_63,
        x127 = BME68X_FILTER_SIZE_127,
    };

    std::ostream& operator<<(
        std::ostream& out,
        BME68xIirFilterSize value
    );


    enum class [[maybe_unused]] BME68xOversampling : uint8_t {
        off = BME68X_OS_NONE,
        x1 = BME68X_OS_1X,
        x2 = BME68X_OS_2X,
        x4 = BME68X_OS_4X,
        x8 = BME68X_OS_8X,
        x16 = BME68X_OS_16X,
    };

    std::ostream& operator<<(
        std::ostream& out,
        BME68xOversampling value
    );

}
