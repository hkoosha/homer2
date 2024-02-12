#pragma once

#include <optional>
#include <array>

#include <hardware/i2c.h>

#include "../src/homer2_sgp40_sensor.hpp"

namespace homer2::sensor::sgp40 {

    class SGP40Data {
    public:

        SGP40Data() = delete;


        SGP40Data(const SGP40Data& other) noexcept = default;

        SGP40Data& operator=(const SGP40Data& other) noexcept = default;


        SGP40Data(SGP40Data&& other) noexcept = default;

        SGP40Data& operator=(SGP40Data&& other) noexcept = default;


        SGP40Data(
            uint16_t rawValue,
            int32_t vocIndex
        ) noexcept;


        [[maybe_unused]]
        [[nodiscard]]
        uint16_t getRawValue() const noexcept;

        [[maybe_unused]]
        [[nodiscard]]
        int32_t getVocIndex() const noexcept;

    private:

        uint16_t _rawValue;
        int32_t _vocIndex;

    };

}

namespace homer2::sensor::sgp40 {

    class SGP40 {
    public:

        SGP40(const SGP40& other) noexcept = delete;

        SGP40(SGP40&& other) = delete;

        SGP40() = delete;

        SGP40& operator=(const SGP40& other) noexcept = delete;

        SGP40& operator=(SGP40&& other) = delete;


        explicit SGP40(
            std::shared_ptr<i2c::Homer2I2c> i2c
        ) noexcept;


        [[nodiscard]]
        std::optional<SGP40Data> measure(
            uint64_t nowMillis,
            float temperatureCelsius,
            float relativeHumidityPercent
        );

        [[maybe_unused]]
        [[nodiscard]]
        const std::array<uint8_t, 6>* readSerial(uint64_t nowMillis);

        [[maybe_unused]]
        [[nodiscard]]
        std::optional<uint16_t> readFeatureSet(uint64_t nowMillis);

        [[maybe_unused]]
        [[nodiscard]]
        std::optional<bool> reset(uint64_t nowMillis);

        [[maybe_unused]]
        [[nodiscard]]
        std::optional<bool> selfTest(uint64_t nowMillis);

        [[maybe_unused]]
        [[nodiscard]]
        std::optional<bool> turnHeaterOff(uint64_t nowMillis);

    private:

        void init();

        [[maybe_unused]]
        void setUninitialized() noexcept;

        void setTask(internal::SGP40Task task);

        void setIdle(internal::SGP40Task fromTask);

        std::shared_ptr<i2c::Homer2I2c> _i2c;

        std::unique_ptr<internal::sensor::SGP40Sensor> _sensor{nullptr};
        internal::SGP40Task _task{internal::SGP40Task::idle};
    };

}
