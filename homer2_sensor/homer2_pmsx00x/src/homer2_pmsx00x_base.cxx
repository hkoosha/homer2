#include <map>
#include <string_view>

#include "homer2_pmsx00x_base.hpp"

namespace homer2::sensor::pmsx00x::internal {

    const char* const TAG = "PMSx00x";

    std::ostream& operator<<(
        std::ostream& out,
        PMSx00xTask value
    ) {
        static std::map<PMSx00xTask, std::string_view> strings{
            {PMSx00xTask::idle,    "idle"},
            {PMSx00xTask::measure, "measure"},
        };

        return out << strings[value];
    }

}
