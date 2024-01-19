#pragma once

#include <cstdint>
#include <limits>
#include <ostream>

#include "../bosch/bmp3_defs.h"

namespace homer2::sensor::bmp3xx {

    namespace internal {

        extern const char* const TAG;

        enum class BMP3Task : uint8_t {
            idle,
            measure,
            reset,
        };

        std::ostream& operator<<(
            std::ostream& out,
            BMP3Task value
        );

    }


    enum class [[maybe_unused]] BMP3xxIirFilterSize : uint8_t {
        off = BMP3_IIR_FILTER_DISABLE,
        x1 = BMP3_IIR_FILTER_COEFF_1,
        x3 = BMP3_IIR_FILTER_COEFF_3,
        x7 = BMP3_IIR_FILTER_COEFF_7,
        x15 = BMP3_IIR_FILTER_COEFF_15,
        x31 = BMP3_IIR_FILTER_COEFF_31,
        x63 = BMP3_IIR_FILTER_COEFF_63,
        x127 = BMP3_IIR_FILTER_COEFF_127,
    };

    std::ostream& operator<<(
        std::ostream& out,
        BMP3xxIirFilterSize value
    );


    enum class [[maybe_unused]] BMP3xxOversampling : uint8_t {
        off = 0,
        x2 = BMP3_OVERSAMPLING_2X,
        x4 = BMP3_OVERSAMPLING_4X,
        x8 = BMP3_OVERSAMPLING_8X,
        x16 = BMP3_OVERSAMPLING_16X,
        x32 = BMP3_OVERSAMPLING_32X,
    };

    std::ostream& operator<<(
        std::ostream& out,
        BMP3xxOversampling value
    );


    enum class [[maybe_unused]] BMP3xxOutputDataRate : uint8_t {
        off = std::numeric_limits<uint8_t>::max(),
        hz_200_0 = BMP3_ODR_200_HZ,
        hz_100_0 = BMP3_ODR_100_HZ,
        hz_50_0 = BMP3_ODR_50_HZ,
        hz_25_0 = BMP3_ODR_25_HZ,
        hz_12_5 = BMP3_ODR_12_5_HZ,
        hz_6_25 = BMP3_ODR_6_25_HZ,
        hz_3_1 = BMP3_ODR_3_1_HZ,
        hz_1_5 = BMP3_ODR_1_5_HZ,
        hz_0_78 = BMP3_ODR_0_78_HZ,
        hz_0_39 = BMP3_ODR_0_39_HZ,
        hz_0_2 = BMP3_ODR_0_2_HZ,
        hz_0_1 = BMP3_ODR_0_1_HZ,
        hz_0_05 = BMP3_ODR_0_05_HZ,
        hz_0_02 = BMP3_ODR_0_02_HZ,
        hz_0_01 = BMP3_ODR_0_01_HZ,
        hz_0_006 = BMP3_ODR_0_006_HZ,
        hz_0_003 = BMP3_ODR_0_003_HZ,
        hz_0_001 = BMP3_ODR_0_001_HZ,
    };

    std::ostream& operator<<(
        std::ostream& out,
        BMP3xxOutputDataRate value
    );

}
