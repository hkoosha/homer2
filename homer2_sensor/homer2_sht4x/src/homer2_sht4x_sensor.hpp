#pragma once

#include <limits>
#include <memory>
#include <array>

#include <hardware/i2c.h>

#include <homer2_i2c.hpp>

#include "homer2_sht4x_base.hpp"

namespace homer2::sensor::sht4x::internal::sensor {

    class SHT4xSensor {
    public:

        SHT4xSensor& operator=(const SHT4xSensor& other) noexcept = delete;

        SHT4xSensor& operator=(SHT4xSensor&& other) = delete;

        SHT4xSensor(SHT4xSensor&& other) = delete;

        SHT4xSensor() = delete;

        SHT4xSensor(const SHT4xSensor& other) noexcept = delete;


        SHT4xSensor(
            std::shared_ptr<i2c::Homer2I2c> i2c,
            bool useAltAddr,
            Precision precision,
            HeaterConf heaterConf
        );


        [[nodiscard]]
        bool measure(uint64_t nowMillis);

        [[nodiscard]]
        bool readSerial(uint64_t nowMillis);

        [[nodiscard]]
        bool reset(uint64_t nowMillis);


        [[nodiscard]]
        float getTemperatureCelsius() const noexcept;

        [[nodiscard]]
        float getRelativeHumidityPercent() const noexcept;

        [[nodiscard]]
        uint32_t getSerial() const noexcept;


    private:

        void doRequestMeasurement(uint64_t nowMillis);

        [[nodiscard]]
        bool doReadMeasurement();

        // -----------------------------

        void doRequestSerial(uint64_t nowMillis);

        [[nodiscard]]
        bool doReadSerial();

        // -----------------------------

        void doReset(uint64_t nowMillis);

        [[nodiscard]]
        bool doReadReset();

        // -----------------------------

        const HeaterConf _heaterConf;
        const Precision _precision;

        homer2::i2c::I2cConnection _i2c;

        float _temperatureCelsius{std::numeric_limits<float>::quiet_NaN()};
        float _relativeHumidityPercent{std::numeric_limits<float>::quiet_NaN()};
        uint32_t _serial{0};

        uint64_t _dataReadyAtMillis{0};

    };

}
