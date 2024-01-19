#pragma once

#include <memory>

#include <homer2_bme68x.hpp>
#include <homer2_sht4x.hpp>
#include <homer2_sgp40.hpp>
#include <homer2_bmp3xx.hpp>
#include <homer2_sunrise.hpp>
#include <homer2_pmsx00x.hpp>

#include "homer2_init.hpp"

namespace homer2 {

    using homer2::sensor::bme68x::BME68x;
    using homer2::sensor::bme68x::BME68xData;
    using homer2::sensor::bme68x::BME68xOversampling;
    using homer2::sensor::bme68x::BME68xIirFilterSize;

    using homer2::sensor::sht4x::SHT4x;
    using homer2::sensor::sht4x::SHT4xData;
    using homer2::sensor::sht4x::Precision;
    using homer2::sensor::sht4x::HeaterConf;

    using homer2::sensor::sgp40::SGP40;
    using homer2::sensor::sgp40::SGP40Data;

    using homer2::sensor::bmp3xx::BMP3xx;
    using homer2::sensor::bmp3xx::BMP3xxData;

    using homer2::sensor::sunrise::Sunrise;
    using homer2::sensor::sunrise::SunriseData;

    using homer2::sensor::pmsx00x::PMSx00x;
    using homer2::sensor::pmsx00x::PMSx00xData;


    class Homer2SensorsData {
    public:

        Homer2SensorsData() noexcept = default;


        Homer2SensorsData(const Homer2SensorsData& other) noexcept = default;

        Homer2SensorsData& operator=(const Homer2SensorsData& other) noexcept = default;


        Homer2SensorsData(Homer2SensorsData&& other) = default;

        Homer2SensorsData& operator=(Homer2SensorsData&& other) = default;


        [[nodiscard]]
        const std::optional<SGP40Data>& sgp40Data() const noexcept;

        [[nodiscard]]
        const std::optional<BME68xData>& bme68xData() const noexcept;

        [[nodiscard]]
        const std::optional<SHT4xData>& sht4xData() const noexcept;

        [[nodiscard]]
        const std::optional<BMP3xxData>& bmp3xxData() const noexcept;

        [[nodiscard]]
        const std::optional<SunriseData>& sunriseData() const noexcept;

        [[nodiscard]]
        const std::optional<PMSx00xData>& pmsx00xData() const noexcept;


        [[nodiscard]]
        Homer2SensorsData withSgp40Data(
            std::optional<SGP40Data> data
        ) const noexcept;

        [[nodiscard]]
        Homer2SensorsData withBme68xData(
            std::optional<BME68xData> data
        ) const noexcept;

        [[nodiscard]]
        Homer2SensorsData withSht4xData(
            std::optional<SHT4xData> data
        ) const noexcept;

        [[nodiscard]]
        Homer2SensorsData withBmp3xxData(
            std::optional<BMP3xxData> data
        ) const noexcept;

        [[nodiscard]]
        Homer2SensorsData withSunriseData(
            std::optional<SunriseData> data
        ) const noexcept;

        [[nodiscard]]
        Homer2SensorsData withPmsx00xData(
            std::optional<PMSx00xData> data
        ) const noexcept;


        [[nodiscard]]
        bool empty() const noexcept;

    private:

        std::optional<SGP40Data> _sgp40Data{std::nullopt};
        std::optional<BME68xData> _bme68xData{std::nullopt};
        std::optional<SHT4xData> _sht4xData{std::nullopt};
        std::optional<BMP3xxData> _bmp3xxData{std::nullopt};
        std::optional<SunriseData> _sunriseData{std::nullopt};
        std::optional<PMSx00xData> _pmsx00xData{std::nullopt};

    };

    class Homer2Sensors {
    public:

        Homer2Sensors& operator=(const Homer2Sensors& other) noexcept = delete;

        Homer2Sensors& operator=(Homer2Sensors&& other) = delete;

        Homer2Sensors(Homer2Sensors&& other) = delete;

        Homer2Sensors(const Homer2Sensors& other) noexcept = delete;


        Homer2Sensors(
            i2c_inst_t* i2c,
            uart_inst_t* uart
        );


        void connectSensors() noexcept;

        [[nodiscard]]
        bool hasAnySensor() const noexcept;

        void querySensors();

        [[nodiscard]]
        Homer2SensorsData data() const noexcept;

    private:

        [[nodiscard]]
        bool isSht4xDataExpired() const noexcept;

        [[nodiscard]]
        bool isBme68xDataExpired() const noexcept;

        [[nodiscard]]
        bool isSgp40DataExpired() const noexcept;

        [[nodiscard]]
        bool isBmp3xxDataExpired() const noexcept;

        [[nodiscard]]
        bool isSunriseDataExpired() const noexcept;

        [[nodiscard]]
        bool isPmsx00xDataExpired() const noexcept;


        [[nodiscard]]
        std::unique_ptr<BME68x> makeBme68x();

        [[nodiscard]]
        std::unique_ptr<SHT4x> makeSht4x();

        [[nodiscard]]
        std::unique_ptr<SGP40> makeSgp40();

        [[nodiscard]]
        std::unique_ptr<BMP3xx> makeBmp3xx();

        [[nodiscard]]
        std::unique_ptr<Sunrise> makeSunrise();

        [[nodiscard]]
        std::unique_ptr<PMSx00x> makePmsx00x();


        void querySgp40() noexcept;

        void querySht4x() noexcept;

        void queryBme68x() noexcept;

        void queryBmp3xx() noexcept;

        void querySunrise() noexcept;

        void queryPmsx00x() noexcept;


        void connectBme68x() noexcept;

        void connectSht4x() noexcept;

        void connectSgp40() noexcept;

        void connectBmp3xx() noexcept;

        void connectSunrise() noexcept;

        void connectPmsx00x() noexcept;


        [[nodiscard]]
        std::optional<HumiditySource> humiditySource(
            HumiditySource source,
            uint8_t level
        ) const noexcept;

        [[nodiscard]]
        std::optional<HumiditySource> humiditySource() const noexcept;

        [[nodiscard]]
        std::optional<TemperatureSource> temperatureSource(
            TemperatureSource source,
            uint8_t level
        ) const noexcept;

        [[nodiscard]]
        std::optional<TemperatureSource> temperatureSource() const noexcept;

        const std::shared_ptr<i2c::Homer2I2c> _i2c;
        uart_inst_t* const _uart;

        uint64_t _lastSgp40DataTime{0};
        uint64_t _lastBme68xDataTime{0};
        uint64_t _lastSht4xDataTime{0};
        uint64_t _lastBmp3xxDataTime{0};
        uint64_t _lastSunriseDataTime{0};
        uint64_t _lastPmsx00xDataTime{0};

        Homer2SensorsData _data;

        std::unique_ptr<BME68x> _bme68x{nullptr};
        std::unique_ptr<SHT4x> _sht4x{nullptr};
        std::unique_ptr<SGP40> _sgp40{nullptr};
        std::unique_ptr<BMP3xx> _bmp3xx{nullptr};
        std::unique_ptr<Sunrise> _sunrise{nullptr};
        std::unique_ptr<PMSx00x> _pmsx00x{nullptr};

        size_t _bme68xErrors{0};
        size_t _sht4xErrors{0};
        size_t _sgp40Errors{0};
        size_t _bmp3xxErrors{0};
        size_t _sunriseErrors{0};
        size_t _pmsx00xErrors{0};

    };

}
