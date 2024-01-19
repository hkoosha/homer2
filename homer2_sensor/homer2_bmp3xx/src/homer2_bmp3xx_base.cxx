#include <map>

#include "homer2_bmp3xx_base.hpp"

namespace homer2::sensor::bmp3xx {

    namespace internal {

        const char* const TAG = "BMP3xx";

        std::ostream& operator<<(
            std::ostream& out,
            BMP3Task value
        ) {
            static std::map<BMP3Task, std::string_view> strings{
                {BMP3Task::idle,    "idle"},
                {BMP3Task::measure, "measure"},
                {BMP3Task::reset,   "reset"},
            };

            return out << strings[value];
        }

    }

    std::ostream& operator<<(
        std::ostream& out,
        const BMP3xxIirFilterSize value
    ) {
        static std::map<BMP3xxIirFilterSize, std::string_view> strings{
            {BMP3xxIirFilterSize::off,  "off"},
            {BMP3xxIirFilterSize::x1,   "x1"},
            {BMP3xxIirFilterSize::x3,   "x3"},
            {BMP3xxIirFilterSize::x7,   "x7"},
            {BMP3xxIirFilterSize::x15,  "x15"},
            {BMP3xxIirFilterSize::x31,  "x31"},
            {BMP3xxIirFilterSize::x63,  "x63"},
            {BMP3xxIirFilterSize::x127, "x127"},
        };

        return out << strings[value];
    }

    std::ostream& operator<<(
        std::ostream& out,
        BMP3xxOversampling value
    ) {
        static std::map<BMP3xxOversampling, std::string_view> strings{
            {BMP3xxOversampling::off, "off"},
            {BMP3xxOversampling::x2,  "x2"},
            {BMP3xxOversampling::x4,  "x4"},
            {BMP3xxOversampling::x8,  "x8"},
            {BMP3xxOversampling::x16, "x16"},
            {BMP3xxOversampling::x32, "x32"},
        };

        return out << strings[value];
    }

    std::ostream& operator<<(
        std::ostream& out,
        BMP3xxOutputDataRate value
    ) {
        static std::map<BMP3xxOutputDataRate, std::string_view> strings{
            {BMP3xxOutputDataRate::off,      "off"},
            {BMP3xxOutputDataRate::hz_200_0, "200.0hz"},
            {BMP3xxOutputDataRate::hz_100_0, "100.0hz"},
            {BMP3xxOutputDataRate::hz_50_0,  "50.0hz"},
            {BMP3xxOutputDataRate::hz_25_0,  "25.0hz"},
            {BMP3xxOutputDataRate::hz_12_5,  "12.5hz"},
            {BMP3xxOutputDataRate::hz_6_25,  "6.25hz"},
            {BMP3xxOutputDataRate::hz_3_1,   "3.1hz"},
            {BMP3xxOutputDataRate::hz_1_5,   "1.5hz"},
            {BMP3xxOutputDataRate::hz_0_78,  "0.78hz"},
            {BMP3xxOutputDataRate::hz_0_39,  "0.39hz"},
            {BMP3xxOutputDataRate::hz_0_2,   "0.2hz"},
            {BMP3xxOutputDataRate::hz_0_1,   "0.1hz"},
            {BMP3xxOutputDataRate::hz_0_05,  "0.05hz"},
            {BMP3xxOutputDataRate::hz_0_02,  "0.02hz"},
            {BMP3xxOutputDataRate::hz_0_01,  "0.01hz"},
            {BMP3xxOutputDataRate::hz_0_006, "0.006hz"},
            {BMP3xxOutputDataRate::hz_0_003, "0.003hz"},
            {BMP3xxOutputDataRate::hz_0_001, "0.001hz"},
        };

        return out << strings[value];
    }

}
