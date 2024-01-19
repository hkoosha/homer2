#include <iostream>
#include <utility>

#include <homer2_logging.hpp>
#include <homer2_util.hpp>

#include "homer2_sht4x.hpp"

using homer2::sensor::sht4x::internal::sensor::SHT4xSensor;

namespace homer2::sensor::sht4x {

    SHT4xData::SHT4xData(
        const float temperatureCelsius,
        const float relativeHumidityPercent
    ) noexcept:
        _temperatureCelsius{temperatureCelsius},
        _relativeHumidityPercent{relativeHumidityPercent} {
    }

    [[maybe_unused]]
    [[nodiscard]]
    float SHT4xData::getRelativeHumidityPercent() const noexcept {

        return this->_relativeHumidityPercent;
    }

    [[maybe_unused]]
    [[nodiscard]]
    float SHT4xData::getTemperatureCelsius() const noexcept {

        return this->_temperatureCelsius;
    }

}

namespace homer2::sensor::sht4x {

    using internal::TAG;
    using internal::SHT4xTask;

    SHT4x::SHT4x(
        std::shared_ptr<i2c::Homer2I2c> i2c
    ) noexcept:
        _i2c{std::move(i2c)} {
    }


    void SHT4x::init() {

        D(0, TAG, "init");

        this->_sensor = nullptr;
        this->_task = SHT4xTask::idle;

        this->_sensor = std::make_unique<SHT4xSensor>(
            this->_i2c,
            this->_useAltAddr,
            this->_precision,
            this->_heaterConf
        );

        D(3, TAG, "fully initialized");
    }

    [[nodiscard]]
    std::optional<SHT4xData> SHT4x::measure(
        const uint64_t nowMillis
    ) {
        if (nullptr == this->_sensor)
            init();

        this->setTask(SHT4xTask::measure);

        try {
            if (this->_sensor->measure(nowMillis)) {

                this->setIdle(SHT4xTask::measure);
                return std::make_optional<SHT4xData>(
                    this->_sensor->getTemperatureCelsius(),
                    this->_sensor->getRelativeHumidityPercent()
                );

            }
            else {

                return std::nullopt;

            }
        }
        catch (...) {

            this->setIdle(SHT4xTask::measure);
            throw;

        }
    }

    [[nodiscard]]
    std::optional<uint32_t> SHT4x::readSerial(
        const uint64_t nowMillis
    ) {
        if (nullptr == this->_sensor)
            this->init();

        this->setTask(SHT4xTask::read_serial_number);

        try {
            if (this->_sensor->readSerial(nowMillis)) {

                this->setIdle(SHT4xTask::read_serial_number);
                return {this->_sensor->getSerial()};

            }
            else {

                return std::nullopt;

            }
        }
        catch (...) {

            this->setIdle(SHT4xTask::read_serial_number);
            throw;

        }
    }

    [[nodiscard]]
    std::optional<bool> SHT4x::reset(
        const uint64_t nowMillis
    ) {
        if (nullptr == this->_sensor)
            this->init();

        this->setTask(SHT4xTask::reset);

        try {
            if (this->_sensor->reset(nowMillis)) {

                this->setIdle(SHT4xTask::reset);
                return {true};

            }
            else {

                return std::nullopt;

            }
        }
        catch (...) {

            this->setIdle(SHT4xTask::reset);
            throw;

        }
    }

    [[maybe_unused]]
    SHT4x* SHT4x::setAltAddress(const bool useAltAddress) noexcept {

        this->setUninitialized();

        D(4, TAG, "using alt address: " << (useAltAddress ? "yes" : "no") << " => " << (useAltAddress ? "yes" : "no"));

        this->_useAltAddr = useAltAddress;
        return this;
    }

    [[maybe_unused]]
    SHT4x* SHT4x::setPrecision(const Precision precision) noexcept {

        this->setUninitialized();

        D(4, TAG, "precision: " << this->_precision << " => " << precision);

        this->_precision = precision;
        return this;
    }

    [[maybe_unused]]
    SHT4x* SHT4x::setHeaterConf(const HeaterConf heaterConf) noexcept {

        this->setUninitialized();

        D(4, TAG, "heater conf: " << this->_heaterConf << " => " << heaterConf);

        this->_heaterConf = heaterConf;
        return this;
    }


    void SHT4x::setUninitialized() noexcept {

        this->_sensor = nullptr;
        this->_task = SHT4xTask::idle;
    }

    void SHT4x::setTask(const SHT4xTask task) {

        if (SHT4xTask::idle != this->_task && task != this->_task) {
            E(TAG, "sensor is busy with: " << this->_task << ", rejecting: " << task);
            throw std::runtime_error{"SHT4x: sensor is busy"};
        }

        this->_task = task;
    }

    void SHT4x::setIdle(const SHT4xTask fromTask) {

        if (fromTask != this->_task) {
            E(TAG, "sensor is not busy with given task, will not set to idle, actual and current task: "
                << this->_task
                << ", assumed task: " << fromTask
                << ", rejecting");

            throw std::runtime_error{"SHT4x: sensor is not busy with the given task"};
        }

        this->_task = SHT4xTask::idle;
    }

}
