#pragma once

#include <memory>
#include <optional>

#include <hardware/i2c.h>

#include "../src/homer2_bmp3xx_sensor.hpp"

namespace homer2::sensor::bmp3xx {

    class BMP3xxData {
    public:

        BMP3xxData() = delete;


        BMP3xxData(const BMP3xxData& other) noexcept = default;

        BMP3xxData& operator=(const BMP3xxData& other) noexcept = default;


        BMP3xxData(BMP3xxData&& other) noexcept = default;

        BMP3xxData& operator=(BMP3xxData&& other) noexcept = default;


        BMP3xxData(
            float temperatureCelsius,
            float pressureHPa
        ) noexcept;


        [[maybe_unused]]
        [[nodiscard]]
        float getTemperatureCelsius() const noexcept;

        [[maybe_unused]]
        [[nodiscard]]
        float getPressureHPa() const noexcept;

        [[maybe_unused]]
        [[nodiscard]]
        float getAltitude(float seaLevelPressureHPa) const noexcept;

    private:

        float _temperatureCelsius;
        float _pressureHPa;

    };

}

namespace homer2::sensor::bmp3xx {

    using homer2::sensor::bmp3xx::BMP3xxIirFilterSize;
    using homer2::sensor::bmp3xx::BMP3xxOversampling;

    class BMP3xx {
    public:

        BMP3xx(const BMP3xx& other) noexcept = delete;

        BMP3xx(BMP3xx&& other) = delete;

        BMP3xx() = delete;

        BMP3xx& operator=(const BMP3xx& other) noexcept = delete;

        BMP3xx& operator=(BMP3xx&& other) = delete;


        explicit BMP3xx(
            std::shared_ptr<i2c::Homer2I2c> i2c
        ) noexcept;


        [[maybe_unused]]
        BMP3xx* setAltAddress(bool useAltAddress) noexcept;

        [[maybe_unused]]
        BMP3xx* setIirFilterSize(BMP3xxIirFilterSize fs) noexcept;

        [[maybe_unused]]
        BMP3xx* setTemperatureOversampling(BMP3xxOversampling os) noexcept;

        [[maybe_unused]]
        BMP3xx* setPressureOversampling(BMP3xxOversampling os) noexcept;

        [[maybe_unused]]
        BMP3xx* setOutputDataRate(BMP3xxOutputDataRate odr) noexcept;


        [[nodiscard]]
        std::optional<BMP3xxData> measure(uint64_t nowMillis);

        [[nodiscard]]
        std::optional<bool> reset(uint64_t nowMillis);


    private:

        void init();

        void setUninitialized() noexcept;

        void setTask(internal::BMP3Task task);

        void setIdle(internal::BMP3Task fromTask);

        BMP3xxIirFilterSize _iirFilterSize{BMP3xxIirFilterSize::off};
        BMP3xxOversampling _temperatureOversampling{BMP3xxOversampling::off};
        BMP3xxOversampling _pressureOversampling{BMP3xxOversampling::off};
        BMP3xxOutputDataRate _outputDataRate{BMP3xxOutputDataRate::hz_200_0};

        bool _useAltAddress{false};
        std::shared_ptr<i2c::Homer2I2c> _i2c;

        std::unique_ptr<internal::sensor::BMP3xxSensor> _sensor{nullptr};
        internal::BMP3Task _task{internal::BMP3Task::idle};

    };

}
