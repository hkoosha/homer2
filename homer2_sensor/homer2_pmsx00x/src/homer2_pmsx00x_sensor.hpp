#pragma once

#include <limits>
#include <memory>
#include <array>

#include <hardware/uart.h>

#include "homer2_pmsx00x_base.hpp"

namespace homer2::sensor::pmsx00x::internal::sensor {

    class PMSx00xSensor {
    public:

        PMSx00xSensor& operator=(const PMSx00xSensor& other) noexcept = delete;

        PMSx00xSensor& operator=(PMSx00xSensor&& other) = delete;

        PMSx00xSensor(PMSx00xSensor&& other) = delete;

        PMSx00xSensor() = delete;

        PMSx00xSensor(const PMSx00xSensor& other) noexcept = delete;


        explicit PMSx00xSensor(uart_inst_t* uart);


        [[nodiscard]]
        bool measure(uint64_t nowMillis);


        [[nodiscard]]
        uint16_t getPm10Std() const noexcept;

        [[nodiscard]]
        uint16_t getPm25Std() const noexcept;

        [[nodiscard]]
        uint16_t getPm100Std() const noexcept;

        [[nodiscard]]
        uint16_t getPm10Env() const noexcept;

        [[nodiscard]]
        uint16_t getPm25Env() const noexcept;

        [[nodiscard]]
        uint16_t getPm100Env() const noexcept;

        [[nodiscard]]
        uint16_t getParticles03() const noexcept;

        [[nodiscard]]
        uint16_t getParticles05() const noexcept;

        [[nodiscard]]
        uint16_t getParticles10() const noexcept;

        [[nodiscard]]
        uint16_t getParticles25() const noexcept;

        [[nodiscard]]
        uint16_t getParticles50() const noexcept;

        [[nodiscard]]
        uint16_t getParticles100() const noexcept;

    private:

        void doRequestMeasurement(uint64_t nowMillis);

        [[nodiscard]]
        bool doReadMeasurement(uint64_t nowMillis);

        // -----------------------------

        uart_inst_t* const _uart;

        uint16_t _pm10Std{0};
        uint16_t _pm25Std{0};
        uint16_t _pm100Std{0};
        uint16_t _pm10Env{0};
        uint16_t _pm25Env{0};
        uint16_t _pm100Env{0};
        uint16_t _particles03{0};
        uint16_t _particles05{0};
        uint16_t _particles10{0};
        uint16_t _particles25{0};
        uint16_t _particles50{0};
        uint16_t _particles100{0};

        uint64_t _dataReadyAtMillis{0};
        uint64_t _stabilizedAt{0};

        size_t _bufIndex{0};
        std::array<uint8_t, 32> _buffer{0};

    };

}
