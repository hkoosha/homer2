#include <iostream>
#include <utility>

#include <homer2_logging.hpp>
#include <homer2_util.hpp>

#include "homer2_bme68x.hpp"

using homer2::sensor::bme68x::internal::sensor::BME68xSensor;

namespace homer2::sensor::bme68x {

    BME68xData::BME68xData(
        const float temperatureCelsius,
        const float pressureHPa,
        const float relativeHumidityPercent,
        const float gasResistanceOhms
    ) noexcept:
        _temperatureCelsius{temperatureCelsius},
        _pressureHPa{pressureHPa},
        _relativeHumidityPercent{relativeHumidityPercent},
        _gasResistanceOhms{gasResistanceOhms} {
    }

    [[maybe_unused]]
    [[nodiscard]]
    float BME68xData::getTemperatureCelsius() const noexcept {

        return this->_temperatureCelsius;
    }

    [[maybe_unused]]
    [[nodiscard]]
    float BME68xData::getPressureHPa() const noexcept {

        return this->_pressureHPa;
    }

    [[maybe_unused]]
    [[nodiscard]]
    float BME68xData::getRelativeHumidityPercent() const noexcept {

        return this->_relativeHumidityPercent;
    }

    [[maybe_unused]]
    [[nodiscard]]
    float BME68xData::getGasResistanceOhms() const noexcept {

        return this->_gasResistanceOhms;
    }

}

namespace homer2::sensor::bme68x {

    using internal::TAG;
    using internal::BME68xTask;

    BME68x::BME68x(
        std::shared_ptr<i2c::Homer2I2c> i2c
    ) noexcept:
        _i2c{std::move(i2c)} {
    }


    void BME68x::init() {

        D(0, TAG, "init");

        this->_sensor = nullptr;
        this->_task = BME68xTask::idle;

        this->_sensor = std::make_unique<BME68xSensor>(
            this->_i2c,
            this->_useAltAddress,
            this->_iirFilterSize,
            this->_temperatureOversampling,
            this->_pressureOversampling,
            this->_humidityOversampling,
            this->_heaterTemperatureCelsius,
            this->_heaterDurationMillis,
            this->_ambientTemperatureCelsius
        );

        D(3, TAG, "fully initialized");
    }

    [[nodiscard]]
    std::optional<BME68xData> BME68x::measure(const uint64_t nowMillis) {

        if (nullptr == this->_sensor)
            this->init();

        this->setTask(BME68xTask::measure);

        try {
            if (this->_sensor->measure(nowMillis)) {

                return std::make_optional<BME68xData>(
                    this->_sensor->getTemperatureCelsius(),
                    this->_sensor->getPressureHPa(),
                    this->_sensor->getRelativeHumidityPercent(),
                    this->_sensor->getGasResistanceOhms()
                );

            }
            else {

                return std::nullopt;

            }
        }
        catch (...) {

            this->setIdle(BME68xTask::measure);
            throw;

        }
    }


    [[maybe_unused]]
    BME68x* BME68x::setAltAddress(const bool useAltAddress) noexcept {

        this->setUninitialized();

        D(4, TAG, "using alt address: " << (useAltAddress ? "yes" : "no") << " => " << (useAltAddress ? "yes" : "no"));

        this->_useAltAddress = useAltAddress;
        return this;
    }

    [[maybe_unused]]
    BME68x* BME68x::setIirFilterSize(const BME68xIirFilterSize filter_size) noexcept {

        this->setUninitialized();

        D(4, TAG, "IIR filter size: " << this->_iirFilterSize << " => " << filter_size);

        this->_iirFilterSize = filter_size;
        return this;
    }

    [[maybe_unused]]
    BME68x* BME68x::setTemperatureOversampling(const BME68xOversampling os) noexcept {

        this->setUninitialized();

        D(4, TAG, "temperature oversampling: " << this->_temperatureOversampling << " => " << os);

        this->_temperatureOversampling = os;
        return this;
    }

    [[maybe_unused]]
    BME68x* BME68x::setPressureOversampling(const BME68xOversampling os) noexcept {

        this->setUninitialized();

        D(4, TAG, "pressure oversampling: " << this->_pressureOversampling << " => " << os);

        this->_pressureOversampling = os;
        return this;
    }

    [[maybe_unused]]
    BME68x* BME68x::setHumidityOversampling(const BME68xOversampling os) noexcept {

        this->setUninitialized();

        D(4, TAG, "humidity oversampling: " << this->_humidityOversampling << " => " << os);

        this->_humidityOversampling = os;
        return this;
    }

    [[maybe_unused]]
    BME68x* BME68x::setGasHeaterTemperatureCelsius(const uint16_t temperatureCelsius) noexcept {

        this->setUninitialized();

        D(4, TAG,
          "gas heater temperature celsius: " << this->_heaterTemperatureCelsius << " => " << temperatureCelsius);

        this->_heaterTemperatureCelsius = temperatureCelsius;
        return this;
    }

    [[maybe_unused]]
    BME68x* BME68x::setGasHeaterDurationMillis(const uint16_t durationMillis) noexcept {

        this->setUninitialized();

        D(4, TAG, "gas heater duration millis: " << this->_heaterDurationMillis << " => " << durationMillis);

        this->_heaterDurationMillis = durationMillis;
        return this;
    }

    [[maybe_unused]]
    BME68x* BME68x::setAmbientTemperatureCelsius(const int8_t temperatureCelsius) noexcept {

        this->setUninitialized();

        D(4, TAG, "ambient temperature: "
            << std::to_string(this->_ambientTemperatureCelsius)
            << " => "
            << std::to_string(temperatureCelsius));

        this->_ambientTemperatureCelsius = temperatureCelsius;
        return this;
    }


    void BME68x::setUninitialized() noexcept {

        this->_sensor = nullptr;
        this->_task = BME68xTask::idle;
    }

    void BME68x::setTask(const BME68xTask task) {

        if (BME68xTask::idle != this->_task && task != this->_task) {
            E(TAG, "sensor is busy with: " << this->_task << ", rejecting: " << task);
            throw std::runtime_error{"BME68x: sensor is busy"};
        }

        this->_task = task;
    }

    void BME68x::setIdle(const BME68xTask fromTask) {

        if (fromTask != this->_task) {
            E(TAG, "sensor is not busy with given task, will not set to idle, actual and current task: "
                << this->_task
                << ", assumed task: " << fromTask
                << ", rejecting");

            throw std::runtime_error{"BME68x: sensor is not busy with the given task"};
        }

        this->_task = BME68xTask::idle;
    }

}
