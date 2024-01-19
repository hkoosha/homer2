#pragma once

#include <limits>
#include <memory>
#include <array>

#include <hardware/i2c.h>

#include <homer2_i2c.hpp>

#include "homer2_sunrise_base.hpp"

namespace homer2::sensor::sunrise::internal::sensor {

    class SunriseSensor {
    public:

        SunriseSensor& operator=(const SunriseSensor& other) noexcept = delete;

        SunriseSensor& operator=(SunriseSensor&& other) = delete;

        SunriseSensor(SunriseSensor&& other) = delete;

        SunriseSensor() = delete;

        SunriseSensor(const SunriseSensor& other) noexcept = delete;


        explicit SunriseSensor(
            std::shared_ptr<i2c::Homer2I2c> i2c
        );


        void measure();

        [[maybe_unused]]
        void disableABC();

        [[maybe_unused]]
        void enableABC();

        [[maybe_unused]]
        void readSensorConfig();

        void setToContinuousMode();


        [[nodiscard]]
        [[maybe_unused]]
        uint16_t getCo2Ppm() const noexcept;

        [[nodiscard]]
        [[maybe_unused]]
        uint8_t getErrorStatus() const noexcept;

    private:

        void wakeup();

        [[noreturn]]
        void haltSystem() const noexcept;

        // -----------------------------

        void doRequestMeasurement(uint64_t nowMillis);

        [[nodiscard]]
        bool doReadMeasurement();

        // -----------------------------

        void setABC(bool enabled);

        // -----------------------------

        homer2::i2c::I2cConnection _i2c;

        uint8_t _measurementMode{0};
        uint16_t _measurementPeriodMillis{0};
        uint16_t _numberOfSamples{0};
        uint16_t _abcPeriodHours{0};
        uint8_t _meterControl{0};

        uint8_t _errorStatus{0};
        uint16_t _co2Ppm{std::numeric_limits<int16_t>::max()};

        uint64_t _dataReadyAtMillis{0};

    };

}
