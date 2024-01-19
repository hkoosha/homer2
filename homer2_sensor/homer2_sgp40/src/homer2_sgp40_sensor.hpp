#pragma once

#include <memory>
#include <array>
#include <limits>

#include <hardware/i2c.h>

#include <homer2_i2c.hpp>

#include "homer2_sgp40_base.hpp"

extern "C" {
#include "../sensirion/sensirion_arch_config.h"
#include "../sensirion/sensirion_voc_algorithm.h"
}

namespace homer2::sensor::sgp40::internal::sensor {

    class SGP40Sensor {
    public:

        SGP40Sensor& operator=(const SGP40Sensor& other) noexcept = delete;

        SGP40Sensor& operator=(SGP40Sensor&& other) = delete;

        SGP40Sensor(SGP40Sensor&& other) = delete;

        SGP40Sensor() = delete;

        SGP40Sensor(const SGP40Sensor& other) noexcept = delete;

        explicit SGP40Sensor(
            std::shared_ptr<i2c::Homer2I2c> i2c
        ) noexcept;

        [[nodiscard]]
        bool measure(
            uint64_t nowMillis,
            float temperatureCelsius,
            float relativeHumidityPercent
        );

        [[nodiscard]]
        bool reset(uint64_t nowMillis);

        [[nodiscard]]
        bool turnHeaterOff(uint64_t nowMillis);

        [[nodiscard]]
        bool selfTest(uint64_t nowMillis);

        [[nodiscard]]
        bool readSerialNumber(uint64_t nowMillis);

        [[nodiscard]]
        bool readFeatureSet(uint64_t nowMillis);


        [[nodiscard]]
        uint16_t getRawIndex() const noexcept;

        [[nodiscard]]
        int32_t getVocIndex() const noexcept;

        [[nodiscard]]
        bool getSelfTest() const noexcept;

        [[nodiscard]]
        const std::array<uint8_t, 6>& getSerialNumber() const noexcept;

        [[nodiscard]]
        uint16_t getFeatureSet() const noexcept;

    private:

        void doRequestMeasurement(
            uint64_t nowMillis,
            float temperatureCelsius,
            float relativeHumidityPercent
        );

        [[nodiscard]]
        bool doReadMeasurement();

        void doRequestReset(uint64_t nowMillis);

        [[nodiscard]]
        bool doReadReset();

        void doRequestHeaterOff(uint64_t nowMillis);

        [[nodiscard]]
        bool doReadHeaterOff();

        void doRequestSelfTest(uint64_t nowMillis);

        [[nodiscard]]
        bool doReadSelfTest();

        void doRequestSerialNumber(uint64_t nowMillis);

        [[nodiscard]]
        bool doReadSerialNumber();

        void doRequestFeatureSet(uint64_t nowMillis);

        [[nodiscard]]
        bool doReadFeatureSet();

        [[nodiscard]]
        uint16_t convertToUInt16(size_t index = 0);

        [[nodiscard]]
        i2c::Homer2I2cError read(size_t len) noexcept;

        i2c::I2cConnection _i2c;

        bool _selfTestOk{false};
        uint16_t _rawIndex{0};
        int32_t _vocIndex{0};
        std::array<uint8_t, 6> _serialNumber{0};
        uint16_t _featureSet{0};
        VocAlgorithmParams _vocAlgorithmParams{};

        uint64_t _dataReadyAtMillis{0};

    };

}


