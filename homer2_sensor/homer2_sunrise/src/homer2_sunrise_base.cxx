#include <map>
#include <string_view>

#include "homer2_sunrise_base.hpp"

namespace homer2::sensor::sunrise::internal {

    const char* const TAG = "Sunrise";

    std::ostream& operator<<(
        std::ostream& out,
        const SunriseTask value
    ) {
        static std::map<SunriseTask, std::string_view> strings{
            {SunriseTask::idle,    "idle"},
            {SunriseTask::measure, "measure"},
            {SunriseTask::reset,   "reset"},
        };

        return out << strings[value];
    }

}
