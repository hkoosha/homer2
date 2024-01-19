#pragma once

#include <optional>
#include <memory>

#include <hardware/i2c.h>

#include "../src/homer2_sunrise_sensor.hpp"
#include "../src/homer2_sunrise_base.hpp"

namespace homer2::sensor::sunrise {

    class SunriseData {
    public:

        SunriseData() = delete;


        SunriseData(const SunriseData& other) noexcept = default;

        SunriseData& operator=(const SunriseData& other) noexcept = default;


        SunriseData(SunriseData&& other) noexcept = default;

        SunriseData& operator=(SunriseData&& other) noexcept = default;


        explicit SunriseData(
            uint16_t co2Ppm,
            uint8_t errorStatus
        ) noexcept;


        [[maybe_unused]]
        [[nodiscard]]
        uint16_t getCo2Ppm() const noexcept;

        [[maybe_unused]]
        [[nodiscard]]
        uint8_t getErrorStatus() const noexcept;

    private:

        uint16_t _co2Ppm;
        uint8_t _errorStatus;

    };

}

namespace homer2::sensor::sunrise {

    class Sunrise {
    public:

        Sunrise& operator=(const Sunrise& other) noexcept = delete;

        Sunrise(const Sunrise& other) noexcept = delete;

        Sunrise& operator=(Sunrise&& other) = delete;

        Sunrise(Sunrise&& other) = delete;

        Sunrise() = delete;


        explicit Sunrise(
            std::shared_ptr<i2c::Homer2I2c> i2c
        ) noexcept;


        [[nodiscard]]
        std::optional<SunriseData> measure(uint64_t nowMillis);

    private:

        void init();

        void setUninitialized() noexcept;

        void setTask(internal::SunriseTask task);

        void setIdle(internal::SunriseTask fromTask);

        std::shared_ptr<i2c::Homer2I2c> _i2c;

        std::unique_ptr<internal::sensor::SunriseSensor> _sensor{nullptr};
        internal::SunriseTask _task{internal::SunriseTask::idle};

    };

}
