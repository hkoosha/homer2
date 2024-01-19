#include <iostream>
#include <utility>

#include <homer2_logging.hpp>
#include <homer2_util.hpp>

#include "homer2_sunrise.hpp"

using homer2::sensor::sunrise::internal::sensor::SunriseSensor;

namespace homer2::sensor::sunrise {

    SunriseData::SunriseData(
        const uint16_t co2Ppm,
        const uint8_t errorStatus
    ) noexcept:
        _co2Ppm{co2Ppm},
        _errorStatus{errorStatus} {
    }

    [[maybe_unused]]
    [[nodiscard]]
    uint16_t SunriseData::getCo2Ppm() const noexcept {

        return this->_co2Ppm;
    }

    [[maybe_unused]]
    [[nodiscard]]
    uint8_t SunriseData::getErrorStatus() const noexcept {

        return this->_errorStatus;
    }

}

namespace homer2::sensor::sunrise {

    using internal::TAG;
    using internal::SunriseTask;

    Sunrise::Sunrise(
        std::shared_ptr<i2c::Homer2I2c> i2c
    ) noexcept:
        _i2c{std::move(i2c)} {
    }


    void Sunrise::init() {

        D(0, TAG, "init");

        this->_sensor = nullptr;
        this->_task = SunriseTask::idle;

        this->_sensor = std::make_unique<SunriseSensor>(this->_i2c);
        this->_sensor->readSensorConfig();
        this->_sensor->setToContinuousMode();
        this->_sensor->enableABC();

        D(3, TAG, "fully initialized");
    }

    [[nodiscard]]
    std::optional<SunriseData> Sunrise::measure(
        const uint64_t nowMillis
    ) {
        (void) nowMillis;

        if (nullptr == this->_sensor)
            init();

        this->setTask(SunriseTask::measure);

        try {

            this->_sensor->measure();

            this->setIdle(SunriseTask::measure);

            return std::make_optional<SunriseData>(
                this->_sensor->getCo2Ppm(),
                this->_sensor->getErrorStatus()
            );

        }
        catch (...) {

            this->setIdle(SunriseTask::measure);
            throw;

        }
    }


    void Sunrise::setUninitialized() noexcept {

        this->_sensor = nullptr;
        this->_task = SunriseTask::idle;
    }

    void Sunrise::setTask(const SunriseTask task) {

        if (SunriseTask::idle != this->_task && task != this->_task) {
            E(TAG, "sensor is busy with: " << this->_task << ", rejecting: " << task);
            throw std::runtime_error{"Sunrise: sensor is busy"};
        }

        this->_task = task;
    }

    void Sunrise::setIdle(const SunriseTask fromTask) {

        if (fromTask != this->_task) {
            E(TAG, "sensor is not busy with given task, will not set to idle, actual and current task: "
                << this->_task
                << ", assumed task: " << fromTask
                << ", rejecting");

            throw std::runtime_error{"Sunrise: sensor is not busy with the given task"};
        }

        this->_task = SunriseTask::idle;
    }

}
