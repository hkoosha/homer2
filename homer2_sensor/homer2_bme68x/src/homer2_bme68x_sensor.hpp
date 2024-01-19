#pragma once

#include <limits>
#include <memory>

#include <hardware/i2c.h>

#include <homer2_i2c.hpp>

#include "homer2_bme68x_base.hpp"
#include "../bosch/bme68x_defs.h"

namespace homer2::sensor::bme68x::internal::sensor {

    class BME68xSensor {
    public:

        BME68xSensor& operator=(const BME68xSensor& other) noexcept = delete;

        BME68xSensor& operator=(BME68xSensor&& other) = delete;

        BME68xSensor(BME68xSensor&& other) = delete;

        BME68xSensor() = delete;

        BME68xSensor(const BME68xSensor& other) noexcept = delete;


        BME68xSensor(
            std::shared_ptr<i2c::Homer2I2c> i2c,
            bool useAltAddr,
            BME68xIirFilterSize iirFilterSize,
            BME68xOversampling temperatureOversampling,
            BME68xOversampling pressureOversampling,
            BME68xOversampling humidityOversampling,
            uint16_t gasHeaterTemperatureCelsius,
            uint16_t gasHeaterDurationMillis,
            int8_t ambientTemperatureCelsius
        );


        [[nodiscard]]
        bool measure(uint64_t nowMillis);


        [[nodiscard]]
        float getTemperatureCelsius() const noexcept;

        [[nodiscard]]
        float getPressureHPa() const noexcept;

        [[nodiscard]]
        float getRelativeHumidityPercent() const noexcept;

        [[nodiscard]]
        float getGasResistanceOhms() const noexcept;


    private:

        void doRequestMeasurement(uint64_t nowMillis);

        bool doReadMeasurement();

        i2c::I2cConnection _i2c;
        bme68x_dev _dev{};
        bme68x_conf _conf{};
        bme68x_heatr_conf _heaterConf{};

        float _temperatureCelsius{std::numeric_limits<float>::quiet_NaN()};
        float _pressureHPa{std::numeric_limits<float>::quiet_NaN()};
        float _relativeHumidityPercent{std::numeric_limits<float>::quiet_NaN()};
        float _gasResistanceOhms{std::numeric_limits<float>::quiet_NaN()};

        uint64_t _dataReadyAtMillis{0};

    };

}


