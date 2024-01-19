#include <iostream>

#include <homer2_logging.hpp>
#include <homer2_util.hpp>

#include "homer2_pmsx00x.hpp"

using homer2::sensor::pmsx00x::internal::sensor::PMSx00xSensor;

namespace homer2::sensor::pmsx00x {

    PMSx00xData::PMSx00xData(
        const uint16_t pm10Std,
        const uint16_t pm25Std,
        const uint16_t pm100Std,
        const uint16_t pm10Env,
        const uint16_t pm25Env,
        const uint16_t pm100Env,
        const uint16_t particles03,
        const uint16_t particles05,
        const uint16_t particles10,
        const uint16_t particles25,
        const uint16_t particles50,
        const uint16_t particles100
    ) noexcept:
        _pm10Std{pm10Std},
        _pm25Std{pm25Std},
        _pm100Std{pm100Std},
        _pm10Env{pm10Env},
        _pm25Env{pm25Env},
        _pm100Env{pm100Env},
        _particles03{particles03},
        _particles05{particles05},
        _particles10{particles10},
        _particles25{particles25},
        _particles50{particles50},
        _particles100{particles100} {
    }

    [[maybe_unused]]
    [[nodiscard]]
    uint16_t PMSx00xData::getPm10Std() const noexcept {

        return this->_pm10Std;
    }

    [[maybe_unused]]
    [[nodiscard]]
    uint16_t PMSx00xData::getPm25Std() const noexcept {

        return this->_pm25Std;
    }

    [[maybe_unused]]
    [[nodiscard]]
    uint16_t PMSx00xData::getPm100Std() const noexcept {

        return this->_pm100Std;
    }

    [[maybe_unused]]
    [[nodiscard]]
    uint16_t PMSx00xData::getPm10Env() const noexcept {

        return this->_pm10Env;
    }

    [[maybe_unused]]
    [[nodiscard]]
    uint16_t PMSx00xData::getPm25Env() const noexcept {

        return this->_pm25Env;
    }

    [[maybe_unused]]
    [[nodiscard]]
    uint16_t PMSx00xData::getPm100Env() const noexcept {

        return this->_pm100Env;
    }

    [[maybe_unused]]
    [[nodiscard]]
    uint16_t PMSx00xData::getParticles03() const noexcept {

        return this->_particles03;
    }

    [[maybe_unused]]
    [[nodiscard]]
    uint16_t PMSx00xData::getParticles05() const noexcept {

        return this->_particles05;
    }

    [[maybe_unused]]
    [[nodiscard]]
    uint16_t PMSx00xData::getParticles10() const noexcept {

        return this->_particles10;
    }

    [[maybe_unused]]
    [[nodiscard]]
    uint16_t PMSx00xData::getParticles25() const noexcept {

        return this->_particles25;
    }

    [[maybe_unused]]
    [[nodiscard]]
    uint16_t PMSx00xData::getParticles50() const noexcept {

        return this->_particles50;
    }

    [[maybe_unused]]
    [[nodiscard]]
    uint16_t PMSx00xData::getParticles100() const noexcept {

        return this->_particles100;
    }

}

namespace homer2::sensor::pmsx00x {

    using internal::TAG;
    using internal::PMSx00xTask;

    PMSx00x::PMSx00x(
        uart_inst_t* const uart
    ) :
        _uart{uart} {

        if (nullptr == uart)
            throw std::logic_error{"uart is not set"};
    }


    void PMSx00x::init() {

        D(0, TAG, "init");

        this->_sensor = nullptr;
        this->_task = PMSx00xTask::idle;

        this->_sensor = std::make_unique<PMSx00xSensor>(this->_uart);

        D(3, TAG, "fully initialized");
    }

    [[nodiscard]]
    std::optional<PMSx00xData> PMSx00x::measure(
        const uint64_t nowMillis
    ) {
        if (nullptr == this->_sensor)
            init();

        this->setTask(PMSx00xTask::measure);

        try {
            if (this->_sensor->measure(nowMillis)) {

                this->setIdle(PMSx00xTask::measure);
                return std::make_optional<PMSx00xData>(
                    this->_sensor->getPm10Std(),
                    this->_sensor->getPm25Std(),
                    this->_sensor->getPm100Std(),
                    this->_sensor->getPm10Env(),
                    this->_sensor->getPm25Env(),
                    this->_sensor->getPm100Env(),
                    this->_sensor->getParticles03(),
                    this->_sensor->getParticles05(),
                    this->_sensor->getParticles10(),
                    this->_sensor->getParticles25(),
                    this->_sensor->getParticles50(),
                    this->_sensor->getParticles100()
                );

            }
            else {

                return std::nullopt;

            }
        }
        catch (...) {

            this->setIdle(PMSx00xTask::measure);
            throw;

        }
    }

    void PMSx00x::setUninitialized() noexcept {

        this->_sensor = nullptr;
        this->_task = PMSx00xTask::idle;
    }

    void PMSx00x::setTask(const PMSx00xTask task) {

        if (PMSx00xTask::idle != this->_task && task != this->_task) {
            E(TAG, "sensor is busy with: " << this->_task << ", rejecting: " << task);
            throw std::runtime_error{"PMSx00x: sensor is busy"};
        }

        this->_task = task;
    }

    void PMSx00x::setIdle(const PMSx00xTask fromTask) {

        if (fromTask != this->_task) {
            E(TAG, "sensor is not busy with given task, will not set to idle, actual and current task: "
                << this->_task
                << ", assumed task: " << fromTask
                << ", rejecting");

            throw std::runtime_error{"PMSx00x: sensor is not busy with the given task"};
        }

        this->_task = PMSx00xTask::idle;
    }

}
