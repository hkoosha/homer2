#include <map>

#include "homer2_bme68x_base.hpp"

namespace homer2::sensor::bme68x {

    namespace internal {

        const char* const TAG = "BME68x";

        std::ostream& operator<<(
            std::ostream& out,
            BME68xTask value
        ) {
            static std::map<BME68xTask, std::string_view> strings{
                {BME68xTask::idle,    "idle"},
                {BME68xTask::measure, "measure"},
            };

            return out << strings[value];
        }

    }

    std::ostream& operator<<(
        std::ostream& out,
        const BME68xIirFilterSize value
    ) {
        static std::map<BME68xIirFilterSize, std::string_view> strings{
            {BME68xIirFilterSize::off,  "off"},
            {BME68xIirFilterSize::x1,   "1x"},
            {BME68xIirFilterSize::x3,   "3x"},
            {BME68xIirFilterSize::x7,   "7x"},
            {BME68xIirFilterSize::x15,  "15x"},
            {BME68xIirFilterSize::x31,  "31x"},
            {BME68xIirFilterSize::x63,  "63x"},
            {BME68xIirFilterSize::x127, "127x"},
        };

        return out << strings[value];
    }

    std::ostream& operator<<(
        std::ostream& out,
        BME68xOversampling value
    ) {
        static std::map<BME68xOversampling, std::string_view> strings{
            {BME68xOversampling::off, "off"},
            {BME68xOversampling::x1,  "1x"},
            {BME68xOversampling::x2,  "2x"},
            {BME68xOversampling::x4,  "4x"},
            {BME68xOversampling::x8,  "8x"},
            {BME68xOversampling::x16, "16x"},
        };

        return out << strings[value];
    }

}
