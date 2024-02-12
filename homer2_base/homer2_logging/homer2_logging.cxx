#include "homer2_logging.hpp"

namespace homer2::logging {

    namespace internal {

#if HOMER2_LOG_USE_MUTEX
        mutex_t homer2_logging_mutex;

        void enter() {

            mutex_enter_blocking(&homer2_logging_mutex);
        }

        void exit() {

            mutex_exit(&homer2_logging_mutex);
        }
#else
        void enter() {
        }

        void exit() {
        }
#endif

        void print_time(
            const uint64_t nowUs
        ) {

            const uint64_t now_millis = nowUs / 1000;
            const uint64_t now_seconds = now_millis / 1'000;
            const uint64_t hours = now_seconds / 3600;
            const uint64_t minute_seconds = now_seconds - hours * 3600;
            const uint64_t minutes = minute_seconds / 60;
            const uint64_t seconds = minute_seconds - minutes * 60;
            const uint64_t millis = now_millis % 1'000;

            std::cout << hours << ':'
                      << std::setw(2) << std::setfill('0') << minutes << ':'
                      << std::setw(2) << std::setfill('0') << seconds << '.';

            if (millis < 10)
                std::cout << "00" << millis;
            else if (millis < 100)
                std::cout << "0" << millis;
            else
                std::cout << millis;
        }

        bool is_logger_enabled(
            const char* const tag
        ) {
            (void) tag;

#pragma clang diagnostic push
#pragma ide diagnostic ignored "Simplify"
            // return 0 == strcasecmp(tag, "Pusher") ||
            //        0 == strcasecmp(tag, "Main") ||
            //        0 == strcasecmp(tag, "Init") ||
            //        0 == strcasecmp(tag, "Sensor");
#pragma clang diagnostic pop

            return true;
        }

    }

    void init() {

#if HOMER2_LOG_USE_MUTEX
        if (mutex_is_initialized(&internal::homer2_logging_mutex))
            panic("logging mutex already initialized");

        mutex_init(&internal::homer2_logging_mutex);
#endif
    }

}
