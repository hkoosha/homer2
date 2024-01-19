#pragma once

#include <optional>
#include <memory>

#include <hardware/uart.h>

#include "../src/homer2_pmsx00x_sensor.hpp"
#include "../src/homer2_pmsx00x_base.hpp"

namespace homer2::sensor::pmsx00x {

    class PMSx00xData {
    public:

        PMSx00xData() = delete;


        PMSx00xData(const PMSx00xData& other) noexcept = default;

        PMSx00xData& operator=(const PMSx00xData& other) noexcept = default;


        PMSx00xData(PMSx00xData&& other) noexcept = default;

        PMSx00xData& operator=(PMSx00xData&& other) noexcept = default;


        PMSx00xData(
            uint16_t pm10Std,
            uint16_t pm25Std,
            uint16_t pm100Std,
            uint16_t pm10Env,
            uint16_t pm25Env,
            uint16_t pm100Env,
            uint16_t particles03,
            uint16_t particles05,
            uint16_t particles10,
            uint16_t particles25,
            uint16_t particles50,
            uint16_t particles100
        ) noexcept;


        [[maybe_unused]]
        [[nodiscard]]
        uint16_t getPm10Std() const noexcept;

        [[maybe_unused]]
        [[nodiscard]]
        uint16_t getPm25Std() const noexcept;

        [[maybe_unused]]
        [[nodiscard]]
        uint16_t getPm100Std() const noexcept;

        [[maybe_unused]]
        [[nodiscard]]
        uint16_t getPm10Env() const noexcept;

        [[maybe_unused]]
        [[nodiscard]]
        uint16_t getPm25Env() const noexcept;

        [[maybe_unused]]
        [[nodiscard]]
        uint16_t getPm100Env() const noexcept;

        [[maybe_unused]]
        [[nodiscard]]
        uint16_t getParticles03() const noexcept;

        [[maybe_unused]]
        [[nodiscard]]
        uint16_t getParticles05() const noexcept;

        [[maybe_unused]]
        [[nodiscard]]
        uint16_t getParticles10() const noexcept;

        [[maybe_unused]]
        [[nodiscard]]
        uint16_t getParticles25() const noexcept;

        [[maybe_unused]]
        [[nodiscard]]
        uint16_t getParticles50() const noexcept;

        [[maybe_unused]]
        [[nodiscard]]
        uint16_t getParticles100() const noexcept;

    private:

        uint16_t _pm10Std;
        uint16_t _pm25Std;
        uint16_t _pm100Std;
        uint16_t _pm10Env;
        uint16_t _pm25Env;
        uint16_t _pm100Env;
        uint16_t _particles03;
        uint16_t _particles05;
        uint16_t _particles10;
        uint16_t _particles25;
        uint16_t _particles50;
        uint16_t _particles100;

    };

}

namespace homer2::sensor::pmsx00x {

    class PMSx00x {
    public:

        PMSx00x& operator=(const PMSx00x& other) noexcept = delete;

        PMSx00x(const PMSx00x& other) noexcept = delete;

        PMSx00x& operator=(PMSx00x&& other) = delete;

        PMSx00x(PMSx00x&& other) = delete;


        explicit PMSx00x(uart_inst_t* uart);


        [[nodiscard]]
        std::optional<PMSx00xData> measure(uint64_t nowMillis);

    private:

        void init();

        void setUninitialized() noexcept;

        void setTask(internal::PMSx00xTask task);

        void setIdle(internal::PMSx00xTask fromTask);

        uart_inst_t* const _uart;

        std::unique_ptr<internal::sensor::PMSx00xSensor> _sensor{nullptr};
        internal::PMSx00xTask _task{internal::PMSx00xTask::idle};

    };

}
