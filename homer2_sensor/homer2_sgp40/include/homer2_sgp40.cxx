#include <iostream>
#include <memory>
#include <utility>

#include <homer2_logging.hpp>
#include <homer2_util.hpp>

#include "homer2_sgp40.hpp"

using homer2::sensor::sgp40::internal::sensor::SGP40Sensor;

namespace homer2::sensor::sgp40 {

    SGP40Data::SGP40Data(
        const uint16_t rawValue,
        const uint32_t vocIndex
    ) noexcept:
        _rawValue{rawValue},
        _vocIndex{vocIndex} {
    }

    [[maybe_unused]]
    [[nodiscard]]
    uint16_t SGP40Data::getRawValue() const noexcept {

        return this->_rawValue;
    }

    [[maybe_unused]]
    [[nodiscard]]
    uint32_t SGP40Data::getVocIndex() const noexcept {

        return this->_vocIndex;
    }

}

namespace homer2::sensor::sgp40 {

    using internal::TAG;
    using internal::SGP40Task;

    SGP40::SGP40(
        std::shared_ptr<i2c::Homer2I2c> i2c
    ) noexcept:
        _i2c{std::move(i2c)} {
    }


    void SGP40::init() {

        D(0, TAG, "init");

        this->_sensor = nullptr;
        this->_task = SGP40Task::idle;

        this->_sensor = std::make_unique<SGP40Sensor>(this->_i2c);

        D(3, TAG, "fully initialized");
    }

    [[nodiscard]]
    std::optional<SGP40Data> SGP40::measure(
        const uint64_t nowMillis,
        const float temperatureCelsius,
        const float relativeHumidityPercent
    ) {
        if (nullptr == this->_sensor)
            this->init();

        this->setTask(SGP40Task::measure);

        try {
            if (this->_sensor->measure(nowMillis, temperatureCelsius, relativeHumidityPercent)) {

                this->setIdle(SGP40Task::measure);
                return std::make_optional<SGP40Data>(
                    this->_sensor->getRawIndex(),
                    this->_sensor->getVocIndex()
                );

            }
            else {

                return std::nullopt;

            }
        }
        catch (...) {
            this->setIdle(SGP40Task::measure);
            throw;
        }
    }

    [[maybe_unused]]
    [[nodiscard]]
    const std::array<uint8_t, 6>* SGP40::readSerial(uint64_t nowMillis) {

        if (nullptr == this->_sensor)
            this->init();

        this->setTask(SGP40Task::serial_number);

        try {
            if (this->_sensor->readSerialNumber(nowMillis)) {

                this->setIdle(SGP40Task::serial_number);
                return &this->_sensor->getSerialNumber();

            }
            else {

                return nullptr;

            }
        }
        catch (...) {

            this->setIdle(SGP40Task::serial_number);
            throw;

        }
    }

    [[maybe_unused]]
    [[nodiscard]]
    std::optional<uint16_t> SGP40::readFeatureSet(const uint64_t nowMillis) {

        if (nullptr == this->_sensor)
            this->init();

        this->setTask(SGP40Task::feature_set);

        try {
            if (this->_sensor->readFeatureSet(nowMillis)) {

                this->setIdle(SGP40Task::feature_set);
                return {this->_sensor->getFeatureSet()};

            }
            else {

                return std::nullopt;

            }
        }
        catch (...) {

            this->setIdle(SGP40Task::feature_set);
            throw;

        }
    }

    [[maybe_unused]]
    [[nodiscard]]
    std::optional<bool> SGP40::turnHeaterOff(const uint64_t nowMillis) {

        if (nullptr == this->_sensor)
            this->init();

        this->setTask(SGP40Task::heater_off);

        try {
            if (this->_sensor->turnHeaterOff(nowMillis)) {

                this->setIdle(SGP40Task::heater_off);
                return {true};

            }
            else {

                return std::nullopt;

            }
        }
        catch (...) {

            this->setIdle(SGP40Task::heater_off);
            throw;

        }
    }

    [[maybe_unused]]
    [[nodiscard]]
    std::optional<bool> SGP40::reset(const uint64_t nowMillis) {

        if (nullptr == this->_sensor)
            this->init();

        this->setTask(SGP40Task::reset);

        try {
            if (this->_sensor->reset(nowMillis)) {

                this->setIdle(SGP40Task::reset);
                return {true};

            }
            else {

                return std::nullopt;

            }
        }
        catch (...) {

            this->setIdle(SGP40Task::reset);
            throw;

        }
    }

    [[maybe_unused]]
    [[nodiscard]]
    std::optional<bool> SGP40::selfTest(const uint64_t nowMillis) {

        if (nullptr == this->_sensor)
            this->init();

        this->setTask(SGP40Task::self_test);

        try {
            if (this->_sensor->selfTest(nowMillis)) {

                this->setIdle(SGP40Task::self_test);
                return {this->_sensor->getSelfTest()};

            }
            else {

                return std::nullopt;

            }
        }
        catch (...) {

            this->setIdle(SGP40Task::self_test);
            throw;

        }
    }


    [[maybe_unused]]
    void SGP40::setUninitialized() noexcept {

        this->_sensor = nullptr;
        this->_task = SGP40Task::idle;
    }

    void SGP40::setTask(SGP40Task task) {

        if (SGP40Task::idle != this->_task && task != this->_task) {
            E(TAG, "sensor is busy with: " << this->_task << ", rejecting: " << task);
            throw std::runtime_error{"SGP40: sensor is busy"};
        }

        this->_task = task;
    }

    void SGP40::setIdle(SGP40Task fromTask) {

        if (fromTask != this->_task) {
            E(TAG, "sensor is not busy with given task, will not set to idle, sensor: "
                << this->_task
                << ", given: " << fromTask
                << ", rejecting");

            throw std::runtime_error{"SGP40: sensor is not busy with the given task"};
        }

        this->_task = SGP40Task::idle;
    }

}
