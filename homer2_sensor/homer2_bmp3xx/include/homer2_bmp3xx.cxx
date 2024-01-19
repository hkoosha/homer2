#include <cmath>
#include <iostream>
#include <utility>

#include <homer2_logging.hpp>
#include <homer2_util.hpp>

#include "homer2_bmp3xx.hpp"

using homer2::sensor::bmp3xx::internal::sensor::BMP3xxSensor;

namespace homer2::sensor::bmp3xx {

    BMP3xxData::BMP3xxData(
        const float temperatureCelsius,
        const float pressureHPa
    ) noexcept:
        _temperatureCelsius{temperatureCelsius},
        _pressureHPa{pressureHPa} {
    }

    [[maybe_unused]]
    [[nodiscard]]
    float BMP3xxData::getTemperatureCelsius() const noexcept {

        return this->_temperatureCelsius;
    }

    [[maybe_unused]]
    [[nodiscard]]
    float BMP3xxData::getPressureHPa() const noexcept {

        return this->_pressureHPa;
    }

    [[maybe_unused]]
    [[nodiscard]]
    float BMP3xxData::getAltitude(
        const float seaLevelPressureHPa
    ) const noexcept {

        const float pressure = this->_pressureHPa / 100.0F;
        const float p = pressure / seaLevelPressureHPa;
        const float v = std::pow(p, 0.19029F);
        return 44330.0F * (1.0F - v);
    }

}

namespace homer2::sensor::bmp3xx {

    using internal::TAG;
    using internal::BMP3Task;

    BMP3xx::BMP3xx(
        std::shared_ptr<i2c::Homer2I2c> i2c
    ) noexcept:
        _i2c{std::move(i2c)} {
    }


    void BMP3xx::init() {

        D(0, TAG, "init");

        this->_sensor = nullptr;
        this->_sensor = std::make_unique<BMP3xxSensor>(
            this->_i2c,
            this->_useAltAddress,
            this->_iirFilterSize,
            this->_temperatureOversampling,
            this->_pressureOversampling,
            this->_outputDataRate
        );

        D(3, TAG, "fully initialized");
    }


    [[nodiscard]]
    std::optional<BMP3xxData> BMP3xx::measure(const uint64_t nowMillis) {

        if (nullptr == this->_sensor)
            this->init();

        this->setTask(BMP3Task::measure);

        try {
            if (this->_sensor->measure(nowMillis)) {

                return std::make_optional<BMP3xxData>(
                    this->_sensor->getTemperatureCelsius(),
                    this->_sensor->getPressureHPa()
                );

            }
            else {

                return std::nullopt;

            }
        }
        catch (...) {

            this->setIdle(BMP3Task::measure);
            throw;

        }
    }

    [[nodiscard]]
    std::optional<bool> BMP3xx::reset(const uint64_t nowMillis) {

        if (nullptr == this->_sensor)
            this->init();

        this->setTask(BMP3Task::idle);

        try {
            if (this->_sensor->reset(nowMillis)) {

                this->setIdle(BMP3Task::reset);
                return {true};

            }
            else {

                return std::nullopt;

            }

        }
        catch (...) {

            this->setIdle(BMP3Task::reset);
            throw;

        }
    }


    [[maybe_unused]]
    BMP3xx* BMP3xx::setAltAddress(const bool useAltAddress) noexcept {

        this->setUninitialized();

        D(4, TAG, "using alt address: " << (useAltAddress ? "yes" : "no") << " => " << (useAltAddress ? "yes" : "no"));

        this->_useAltAddress = useAltAddress;
        return this;
    }

    [[maybe_unused]]
    BMP3xx* BMP3xx::setIirFilterSize(BMP3xxIirFilterSize fs) noexcept {

        this->setUninitialized();

        D(4, TAG, "IIR filter size: " << this->_iirFilterSize << " => " << fs);

        this->_iirFilterSize = fs;
        return this;
    }

    [[maybe_unused]]
    BMP3xx* BMP3xx::setTemperatureOversampling(const BMP3xxOversampling os) noexcept {

        this->setUninitialized();

        D(4, TAG, "temperature oversampling: " << this->_temperatureOversampling << " => " << os);

        this->_temperatureOversampling = os;
        return this;
    }

    [[maybe_unused]]
    BMP3xx* BMP3xx::setPressureOversampling(const BMP3xxOversampling os) noexcept {

        this->setUninitialized();

        D(4, TAG, "pressure oversampling: " << this->_pressureOversampling << " => " << os);

        this->_pressureOversampling = os;
        return this;
    }

    [[maybe_unused]]
    BMP3xx* BMP3xx::setOutputDataRate(const BMP3xxOutputDataRate odr) noexcept {

        this->setUninitialized();

        D(4, TAG, "output data rate: " << this->_outputDataRate << " => " << odr);

        this->_outputDataRate = odr;
        return this;
    }


    void BMP3xx::setUninitialized() noexcept {

        this->_sensor = nullptr;
        this->_task = BMP3Task::idle;
    }

    void BMP3xx::setTask(const BMP3Task task) {

        if (BMP3Task::idle != this->_task && task != this->_task) {
            E(TAG, "sensor is busy with: " << this->_task << ", rejecting: " << task);
            throw std::runtime_error{"BMP3xx: sensor is busy"};
        }

        this->_task = task;
    }

    void BMP3xx::setIdle(const BMP3Task fromTask) {

        if (fromTask != this->_task) {
            E(TAG, "sensor is not busy with given task, will not set to idle, actual and current task: "
                << this->_task
                << ", assumed task: " << fromTask
                << ", rejecting");

            throw std::runtime_error{"BMP3xx: sensor is not busy with the given task"};
        }

        this->_task = BMP3Task::idle;
    }


}
