#pragma once

#include <memory>

#include <hardware/i2c.h>

#include <homer2_i2c.hpp>

#include "homer2_bmp3xx_base.hpp"
#include "../bosch/bmp3_defs.h"

namespace homer2::sensor::bmp3xx::internal::sensor {

    class BMP3xxSensor {
    public:

        BMP3xxSensor& operator=(const BMP3xxSensor& other) noexcept = delete;

        BMP3xxSensor& operator=(BMP3xxSensor&& other) = delete;

        BMP3xxSensor(BMP3xxSensor&& other) = delete;

        BMP3xxSensor() = delete;

        BMP3xxSensor(const BMP3xxSensor& other) noexcept = delete;


        BMP3xxSensor(
            std::shared_ptr<i2c::Homer2I2c> i2c,
            bool useAltAddr,
            BMP3xxIirFilterSize iirFilterSize,
            BMP3xxOversampling temperatureOversampling,
            BMP3xxOversampling pressureOversampling,
            BMP3xxOutputDataRate outputDataRate
        );


        [[nodiscard]]
        bool measure(uint64_t nowMillis);

        [[nodiscard]]
        bool reset(uint64_t nowMillis);


        [[nodiscard]]
        double getTemperatureCelsius() const noexcept;

        [[nodiscard]]
        double getPressureHPa() const noexcept;


    private:

        void validateTrimmingParameters();

        // -----------------------------

        void doRequestMeasurement(uint64_t nowMillis);

        [[nodiscard]]
        bool doReadMeasurement();

        // -----------------------------

        void doReset(uint64_t nowMillis);

        [[nodiscard]]
        bool doReadReset();

        // -----------------------------

        i2c::I2cConnection _i2c;

        bmp3_dev _dev{};
        bmp3_data _data{};

        uint64_t _dataReadyAtMillis{0};

    };

}
