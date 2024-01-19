#include <ostream>
#include <map>
#include <string_view>

#include "homer2_sgp40_base.hpp"

namespace homer2::sensor::sgp40::internal {

    const char* const TAG = "SGP40";

    std::ostream& operator<<(
        std::ostream& out,
        SGP40Task value
    ) {
        static std::map<SGP40Task, std::string_view> strings{
            {SGP40Task::idle,               "idle"},
            {SGP40Task::serial_number, "serial_number"},
            {SGP40Task::feature_set,        "feature_set"},
            {SGP40Task::self_test,          "self_test"},
            {SGP40Task::heater_off,         "heater_off"},
            {SGP40Task::measure,            "measure"},
            {SGP40Task::reset,              "reset"},
        };

        return out << strings[value];
    }

}
