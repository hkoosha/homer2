#pragma once

#include <optional>
#include <memory>

#include <hardware/i2c.h>

#include "../src/homer2_sht4x_sensor.hpp"
#include "../src/homer2_sht4x_base.hpp"

namespace homer2::sensor::sht4x {

    class SHT4xData {
    public:

        SHT4xData() = delete;


        SHT4xData(const SHT4xData& other) noexcept = default;

        SHT4xData& operator=(const SHT4xData& other) noexcept = default;


        SHT4xData(SHT4xData&& other) noexcept = default;

        SHT4xData& operator=(SHT4xData&& other) noexcept = default;


        SHT4xData(
            float temperatureCelsius,
            float relativeHumidityPercent
        ) noexcept;


        [[maybe_unused]]
        [[nodiscard]]
        float getTemperatureCelsius() const noexcept;

        [[maybe_unused]]
        [[nodiscard]]
        float getRelativeHumidityPercent() const noexcept;

    private:

        float _temperatureCelsius;
        float _relativeHumidityPercent;

    };

}

namespace homer2::sensor::sht4x {

    class SHT4x {
    public:

        SHT4x& operator=(const SHT4x& other) noexcept = delete;

        SHT4x(const SHT4x& other) noexcept = delete;

        SHT4x& operator=(SHT4x&& other) = delete;

        SHT4x(SHT4x&& other) = delete;

        SHT4x() = delete;


        explicit SHT4x(
            std::shared_ptr<i2c::Homer2I2c> i2c
        ) noexcept;


        [[maybe_unused]]
        SHT4x* setAltAddress(bool useAltAddress) noexcept;

        [[maybe_unused]]
        SHT4x* setPrecision(Precision precision) noexcept;

        [[maybe_unused]]
        SHT4x* setHeaterConf(HeaterConf heaterConf) noexcept;


        [[nodiscard]]
        std::optional<SHT4xData> measure(uint64_t nowMillis);

        [[nodiscard]]
        std::optional<uint32_t> readSerial(uint64_t nowMillis);

        [[nodiscard]]
        std::optional<bool> reset(uint64_t nowMillis);

    private:

        void init();

        void setUninitialized() noexcept;

        void setTask(internal::SHT4xTask task);

        void setIdle(internal::SHT4xTask fromTask);

        HeaterConf _heaterConf{HeaterConf::off};
        Precision _precision{Precision::high};

        bool _useAltAddr{false};
        std::shared_ptr<i2c::Homer2I2c> _i2c;

        std::unique_ptr<internal::sensor::SHT4xSensor> _sensor{nullptr};
        internal::SHT4xTask _task{internal::SHT4xTask::idle};

    };

}
