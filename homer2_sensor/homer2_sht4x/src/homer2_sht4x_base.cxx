#include <map>
#include <string_view>

#include "homer2_sht4x_base.hpp"

namespace homer2::sensor::sht4x {

    namespace internal {

        const char* const TAG = "SHT4x";

        std::ostream& operator<<(
            std::ostream& out,
            SHT4xTask value
        ) {
            static std::map<SHT4xTask, std::string_view> strings{
                {SHT4xTask::idle,               "idle"},
                {SHT4xTask::read_serial_number, "read_serial_number"},
                {SHT4xTask::measure,            "measure"},
                {SHT4xTask::reset,              "reset"},
            };

            return out << strings[value];
        }

    }

    std::ostream& operator<<(
        std::ostream& out,
        const Precision value
    ) {
        static std::map<Precision, std::string_view> strings{
            {Precision::low,    "low"},
            {Precision::medium, "medium"},
            {Precision::high,   "high"},
        };

        return out << strings[value];
    }

    std::ostream& operator<<(
        std::ostream& out,
        const HeaterConf value
    ) {

        static std::map<HeaterConf, std::string_view> strings{
            {HeaterConf::off,           "off"},
            {HeaterConf::low_100ms,     "low_100ms"},
            {HeaterConf::low_1000ms,    "low_1000ms"},
            {HeaterConf::medium_100ms,  "medium_100ms"},
            {HeaterConf::medium_1000ms, "medium_1000ms"},
            {HeaterConf::high_100ms,    "high_100ms"},
            {HeaterConf::high_1000ms,   "high_1000ms"},
        };

        return out << strings[value];
    }

}
