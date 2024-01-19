#pragma once

#include <memory>
#include <optional>

#include <hardware/i2c.h>

#include "../src/homer2_bme68x_sensor.hpp"

namespace homer2::sensor::bme68x {

    class BME68xData {
    public:

        BME68xData() = delete;


        BME68xData(const BME68xData& other) noexcept = default;

        BME68xData& operator=(const BME68xData& other) noexcept = default;


        BME68xData(BME68xData&& other) noexcept = default;

        BME68xData& operator=(BME68xData&& other) noexcept = default;


        BME68xData(
            float temperatureCelsius,
            float pressureHPa,
            float relativeHumidityPercent,
            float gasResistanceOhms
        ) noexcept;


        [[maybe_unused]]
        [[nodiscard]]
        float getTemperatureCelsius() const noexcept;

        [[maybe_unused]]
        [[nodiscard]]
        float getPressureHPa() const noexcept;

        [[maybe_unused]]
        [[nodiscard]]
        float getRelativeHumidityPercent() const noexcept;

        [[maybe_unused]]
        [[nodiscard]]
        float getGasResistanceOhms() const noexcept;

    private:

        float _temperatureCelsius;
        float _pressureHPa;
        float _relativeHumidityPercent;
        float _gasResistanceOhms;

    };

}

namespace homer2::sensor::bme68x {

    using homer2::sensor::bme68x::BME68xIirFilterSize;
    using homer2::sensor::bme68x::BME68xOversampling;

    class BME68x {
    public:

        BME68x(const BME68x& other) noexcept = delete;

        BME68x(BME68x&& other) = delete;

        BME68x() = delete;

        BME68x& operator=(const BME68x& other) noexcept = delete;

        BME68x& operator=(BME68x&& other) = delete;


        explicit BME68x(
            std::shared_ptr<i2c::Homer2I2c> i2c
        ) noexcept;


        [[maybe_unused]]
        BME68x* setAltAddress(bool useAltAddress) noexcept;

        [[maybe_unused]]
        BME68x* setIirFilterSize(BME68xIirFilterSize filter_size) noexcept;

        [[maybe_unused]]
        BME68x* setTemperatureOversampling(BME68xOversampling os) noexcept;

        [[maybe_unused]]
        BME68x* setPressureOversampling(BME68xOversampling os) noexcept;

        [[maybe_unused]]
        BME68x* setHumidityOversampling(BME68xOversampling os) noexcept;

        [[maybe_unused]]
        BME68x* setGasHeaterTemperatureCelsius(uint16_t temperatureCelsius) noexcept;

        [[maybe_unused]]
        BME68x* setGasHeaterDurationMillis(uint16_t durationMillis) noexcept;

        [[maybe_unused]]
        BME68x* setAmbientTemperatureCelsius(int8_t temperatureCelsius) noexcept;


        [[nodiscard]]
        std::optional<BME68xData> measure(uint64_t nowMillis);


    private:

        void init();

        void setUninitialized() noexcept;

        void setTask(internal::BME68xTask task);

        void setIdle(internal::BME68xTask fromTask);


        BME68xIirFilterSize _iirFilterSize{BME68xIirFilterSize::off};
        BME68xOversampling _temperatureOversampling{BME68xOversampling::off};
        BME68xOversampling _humidityOversampling{BME68xOversampling::off};
        BME68xOversampling _pressureOversampling{BME68xOversampling::off};
        uint16_t _heaterTemperatureCelsius{320};
        uint16_t _heaterDurationMillis{150};
        int8_t _ambientTemperatureCelsius{25};

        bool _useAltAddress{false};
        std::shared_ptr<i2c::Homer2I2c> _i2c;

        std::unique_ptr<internal::sensor::BME68xSensor> _sensor{nullptr};
        internal::BME68xTask _task{internal::BME68xTask::idle};

    };

}
