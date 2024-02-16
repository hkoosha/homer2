#include <array>
#include <map>

#include <homer2_util.hpp>
#include <homer2_logging.hpp>

#include "homer2_config.h"
#include "homer2_init.hpp"
#include "homer2_sensor.hpp"

namespace homer2 {

    namespace {
        const char* const TAG = "Sensor";
    }

}

namespace homer2 {

    const std::optional<SGP40Data>& Homer2SensorsData::sgp40Data() const noexcept {

        return this->_sgp40Data;
    }

    const std::optional<BME68xData>& Homer2SensorsData::bme68xData() const noexcept {

        return this->_bme68xData;
    }

    const std::optional<SHT4xData>& Homer2SensorsData::sht4xData() const noexcept {

        return this->_sht4xData;
    }

    const std::optional<BMP3xxData>& Homer2SensorsData::bmp3xxData() const noexcept {

        return this->_bmp3xxData;
    }

    const std::optional<SunriseData>& Homer2SensorsData::sunriseData() const noexcept {

        return this->_sunriseData;
    }

    const std::optional<PMSx00xData>& Homer2SensorsData::pmsx00xData() const noexcept {

        return this->_pmsx00xData;
    }


    [[nodiscard]]
    Homer2SensorsData Homer2SensorsData::withSgp40Data(
        const std::optional<SGP40Data> data
    ) const noexcept {

        Homer2SensorsData newData{*this};
        newData._sgp40Data = data;
        return newData;
    }

    [[nodiscard]]
    Homer2SensorsData Homer2SensorsData::withBme68xData(
        const std::optional<BME68xData> data
    ) const noexcept {

        Homer2SensorsData newData{*this};
        newData._bme68xData = data;
        return newData;
    }

    [[nodiscard]]
    Homer2SensorsData Homer2SensorsData::withSht4xData(
        const std::optional<SHT4xData> data
    ) const noexcept {

        Homer2SensorsData newData{*this};
        newData._sht4xData = data;
        return newData;
    }

    [[nodiscard]]
    Homer2SensorsData Homer2SensorsData::withBmp3xxData(
        const std::optional<BMP3xxData> data
    ) const noexcept {

        Homer2SensorsData newData{*this};
        newData._bmp3xxData = data;
        return newData;
    }

    [[nodiscard]]
    Homer2SensorsData Homer2SensorsData::withSunriseData(
        const std::optional<SunriseData> data
    ) const noexcept {

        Homer2SensorsData newData{*this};
        newData._sunriseData = data;
        return newData;
    }

    [[nodiscard]]
    Homer2SensorsData Homer2SensorsData::withPmsx00xData(
        const std::optional<PMSx00xData> data
    ) const noexcept {

        Homer2SensorsData newData{*this};
        newData._pmsx00xData = data;
        return newData;
    }


    bool Homer2SensorsData::empty() const noexcept {

        return !this->sht4xData().has_value() &&
               !this->bme68xData().has_value() &&
               !this->bmp3xxData().has_value() &&
               !this->pmsx00xData().has_value() &&
               !this->sunriseData().has_value() &&
               !this->sgp40Data().has_value();
    }

}

namespace homer2 {

    using homer2::sensor::bme68x::BME68x;
    using homer2::sensor::bme68x::BME68xOversampling;
    using homer2::sensor::bme68x::BME68xIirFilterSize;

    using homer2::sensor::sht4x::SHT4x;
    using homer2::sensor::sht4x::Precision;
    using homer2::sensor::sht4x::HeaterConf;

    using homer2::sensor::sgp40::SGP40;

    using homer2::sensor::sunrise::Sunrise;

    namespace {

        constexpr size_t SENSOR_ERROR_THRESHOLD = 5;

    }

    Homer2Sensors::Homer2Sensors(
        i2c_inst_t* const i2c,
        uart_inst_t* const uart
    ) :
        _uart{uart},
        _i2c{std::make_shared<i2c::Homer2I2c>(i2c)} {

        if (nullptr == i2c)
            throw std::logic_error{"i2c not set"};

        if (nullptr == uart)
            throw std::logic_error{"uart not set"};
    }


    [[nodiscard]]
    std::unique_ptr<BME68x> Homer2Sensors::makeBme68x() {

        I(TAG, "making BME68x");

        auto sensor = std::make_unique<BME68x>(this->_i2c);

        sensor
            ->setAmbientTemperatureCelsius(HOMER2_BME68X_AMBIENT_TEMPERATURE)
            ->setGasHeaterDurationMillis(HOMER2_BME68X_GAS_HEATER_DURATION_MILLIS)
            ->setGasHeaterTemperatureCelsius(HOMER2_BME68X_GAS_HEATER_TEMPERATURE_CELSIUS)
            ->setTemperatureOversampling(HOMER2_BME68X_TEMPERATURE_OVERSAMPLING)
            ->setHumidityOversampling(HOMER2_BME68X_HUMIDITY_OVERSAMPLING)
            ->setPressureOversampling(HOMER2_BME68X_PRESSURE_OVERSAMPLING)
            ->setIirFilterSize(HOMER2_BME68X_IIR_FILTER_SIZE)
            ->setAltAddress(HOMER2_BME68X_I2C_ALT_ADDR);

        return sensor;
    }

    [[nodiscard]]
    std::unique_ptr<SHT4x> Homer2Sensors::makeSht4x() {

        I(TAG, "making SHT4x");

        auto sensor = std::make_unique<SHT4x>(this->_i2c);

        sensor
            ->setAltAddress(HOMER2_SHT4X_I2C_ALT_ADDR)
            ->setPrecision(HOMER2_SHT4X_PRECISION_CONF)
            ->setHeaterConf(HeaterConf::off);

        return sensor;
    }

    [[nodiscard]]
    std::unique_ptr<SGP40> Homer2Sensors::makeSgp40() {

        I(TAG, "making SGP40");

        auto sensor = std::make_unique<SGP40>(this->_i2c);

        return sensor;
    }

    [[nodiscard]]
    std::unique_ptr<BMP3xx> Homer2Sensors::makeBmp3xx() {

        I(TAG, "making BMP3xx");

        auto sensor = std::make_unique<BMP3xx>(this->_i2c);

        return sensor;
    }

    [[nodiscard]]
    std::unique_ptr<Sunrise> Homer2Sensors::makeSunrise() {

        I(TAG, "making Sunrise");

        auto sensor = std::make_unique<Sunrise>(this->_i2c);

        return sensor;
    }


    std::unique_ptr<PMSx00x> Homer2Sensors::makePmsx00x() {

        I(TAG, "making PMSx00x");

        auto sensor = std::make_unique<PMSx00x>(this->_uart);

        return sensor;
    }


    [[nodiscard]]
    std::optional<HumiditySource> Homer2Sensors::humiditySource(
        const HumiditySource source,
        const uint8_t level
    ) const noexcept {

        switch (source) {
            case HumiditySource::sht4x:
                if ((nullptr != this->_sht4x) &&
                    (!this->isSht4xDataExpired()) &&
                    (this->_data.sht4xData().has_value())
                    )
                    return source;

                D(3, TAG, "SHT4x selected as SGP40 data provider #"
                    << std::to_string(level)
                    << " but data is not available, ignoring | has last value: "
                    << (this->_data.sht4xData().has_value() ? "yes" : "no")
                    << ", is expired: "
                    << (is_expired(this->_lastSht4xDataTime, HOMER2_CACHED_DATA_EXPIRY_MILLIS)
                        ? "yes" : "no"));
                return std::nullopt;

            case HumiditySource::bme68x:
                if ((nullptr != this->_bme68x) &&
                    (!this->isBme68xDataExpired()) &&
                    (this->_data.bme68xData().has_value())
                    )
                    return source;

                D(3, TAG, "BME68x selected as SGP40 data provider #"
                    << std::to_string(level)
                    << " but data is not available, ignoring | has last value: "
                    << (this->_data.bmp3xxData().has_value() ? "yes" : "no")
                    << ", is expired: "
                    << (is_expired(this->_lastBme68xDataTime, HOMER2_CACHED_DATA_EXPIRY_MILLIS)
                        ? "yes" : "no"));
                return std::nullopt;

            case HumiditySource::constant:
                W(TAG, "constants selected as SGP40 data provider #"
                    << std::to_string(level)
                    << ", data will NOT be accurate");
                return source;

            case HumiditySource::disabled:
                D(3, TAG, "SGP40 is disabled on provider #"
                    << std::to_string(level)
                    << ", not checking further data providers");
                return source;

            default:
                assert(false);
        }
    }

    [[nodiscard]]
    std::optional<HumiditySource> Homer2Sensors::humiditySource() const noexcept {

        const auto& provider0 = this->humiditySource(homer2::humidity_source0(), 0);
        if (provider0.has_value()) {
            D(3, TAG, "humidity provider #0 selected: " << provider0.value());
            return provider0;
        }

        const auto& provider1 = this->humiditySource(homer2::humidity_source1(), 1);
        if (provider1.has_value()) {
            D(3, TAG, "humidity provider #1 selected: " << provider1.value());
            return provider1;
        }

        const auto& provider2 = this->humiditySource(homer2::humidity_source2(), 2);
        if (provider2.has_value()) {
            D(3, TAG, "humidity provider #2 selected: " << provider2.value());
            return provider2;
        }

        const auto& provider3 = this->humiditySource(homer2::humidity_source3(), 3);
        if (provider3.has_value()) {
            D(3, TAG, "humidity provider #3 selected: " << provider3.value());
            return provider3;
        }

        W(TAG, "no humidity provider available");
        return std::nullopt;
    }

    [[nodiscard]]
    std::optional<TemperatureSource> Homer2Sensors::temperatureSource(
        const TemperatureSource source,
        const uint8_t level
    ) const noexcept {

        switch (source) {
            case TemperatureSource::sht4x:
                if ((nullptr != this->_sht4x) &&
                    (!this->isSht4xDataExpired()) &&
                    (this->_data.sht4xData().has_value())
                    )
                    return source;

                D(3, TAG, "SHT4x selected as SGP40 data provider #"
                    << std::to_string(level)
                    << " but data is not available, ignoring | has last value: "
                    << (this->_data.sht4xData().has_value() ? "yes" : "no")
                    << ", is expired: "
                    << (is_expired(this->_lastSht4xDataTime, HOMER2_CACHED_DATA_EXPIRY_MILLIS)
                        ? "yes" : "no"));
                return std::nullopt;


            case TemperatureSource::bme68x:
                if ((nullptr != this->_bme68x) &&
                    (!this->isBme68xDataExpired()) &&
                    (this->_data.bme68xData().has_value())
                    )
                    return source;

                D(3, TAG, "BME68x selected as SGP40 data provider #"
                    << std::to_string(level)
                    << " but data is not available, ignoring | has last value: "
                    << (this->_data.bme68xData().has_value() ? "yes" : "no")
                    << ", is expired: "
                    << (is_expired(this->_lastBme68xDataTime, HOMER2_CACHED_DATA_EXPIRY_MILLIS)
                        ? "yes" : "no"));
                return std::nullopt;

            case TemperatureSource::bmp3xx:
                if ((nullptr != this->_bmp3xx) &&
                    (!this->isBmp3xxDataExpired()) &&
                    (this->_data.bmp3xxData().has_value())
                    )
                    return source;

                D(3, TAG, "BMP3xx selected as SGP40 data provider #"
                    << std::to_string(level)
                    << " but data is not available, ignoring | has last value: "
                    << (this->_data.bmp3xxData().has_value() ? "yes" : "no")
                    << ", is expired: "
                    << (is_expired(this->_lastBmp3xxDataTime, HOMER2_CACHED_DATA_EXPIRY_MILLIS)
                        ? "yes" : "no"));
                return std::nullopt;

            case TemperatureSource::constant:
                W(TAG, "constants selected as SGP40 data provider #"
                    << std::to_string(level)
                    << ", data will NOT be accurate");
                return source;

            case TemperatureSource::disabled:
                D(3, TAG, "SGP40 is disabled on provider #"
                    << std::to_string(level)
                    << ", not checking further data providers");
                return source;

            default:
                assert(false);
        }
    }

    [[nodiscard]]
    std::optional<TemperatureSource> Homer2Sensors::temperatureSource() const noexcept {

        const auto& provider0 = this->temperatureSource(homer2::temperature_source0(), 0);
        if (provider0.has_value()) {
            D(3, TAG, "temperature provider #0 selected: " << provider0.value());
            return provider0;
        }

        const auto& provider1 = this->temperatureSource(homer2::temperature_source1(), 1);
        if (provider1.has_value()) {
            D(3, TAG, "temperature provider #1 selected: " << provider1.value());
            return provider1;
        }

        const auto& provider2 = this->temperatureSource(homer2::temperature_source2(), 2);
        if (provider2.has_value()) {
            D(3, TAG, "temperature provider #2 selected: " << provider2.value());
            return provider2;
        }

        const auto& provider3 = this->temperatureSource(homer2::temperature_source3(), 3);
        if (provider3.has_value()) {
            D(3, TAG, "humidity provider #3 selected: " << provider3.value());
            return provider3;
        }

        W(TAG, "no temperature provider available");
        return std::nullopt;
    }


    void Homer2Sensors::connectSensors() noexcept {

        this->connectPmsx00x();
        this->connectBme68x();
        this->connectSht4x();
        this->connectSgp40();
        this->connectBmp3xx();
        this->connectSunrise();
    }


    void Homer2Sensors::connectBme68x() noexcept {

        if (!is_enabled_bme68x()) {
            this->_bme68x = nullptr;
            return;
        }

        if (SENSOR_ERROR_THRESHOLD <= this->_bme68xErrors) {
            W(TAG, "BME68x has encountered too many errors, reconnecting: "
                << this->_bme68xErrors);
            this->_bme68x = nullptr;
            this->_bme68xErrors = 0;
        }

        if (nullptr != this->_bme68x)
            return;

        try {
            this->_bme68x = this->makeBme68x();
        }
        catch (std::exception& e) {
            E(TAG, "failed to create BME68x sensor: " << e.what());
            this->_bme68x = nullptr;
        }
        catch (...) {
            E(TAG, "failed to create BME68x very badly! do not even know why");
            this->_bme68x = nullptr;
        }

        if (nullptr != this->_bme68x)
            I(TAG, "BME68x connected");
    }

    void Homer2Sensors::connectSht4x() noexcept {

        if (!is_enabled_sht4x()) {
            this->_sht4x = nullptr;
            return;
        }

        if (SENSOR_ERROR_THRESHOLD <= this->_sht4xErrors) {
            W(TAG, "SHT4x has encountered too many errors, reconnecting: "
                << this->_sht4xErrors);
            this->_sht4x = nullptr;
            this->_sht4xErrors = 0;
        }

        if (nullptr != this->_sht4x)
            return;

        try {
            this->_sht4x = this->makeSht4x();
        }
        catch (std::exception& e) {
            E(TAG, "failed to create SHT4x sensor: " << e.what());
            this->_sht4x = nullptr;
            return;
        }
        catch (...) {
            E(TAG, "failed to create SHT4x very badly! do not even know why");
            this->_sht4x = nullptr;
            return;
        }

        auto resetOk = false;
        for (uint32_t i = 0; i < HOMER2_SHT4X_MAX_RESET_RETRIES; i++) {

            std::optional<bool> reset;
            try {
                reset = this->_sht4x->reset(now());
            }
            catch (...) {
                continue;
            }

            if (reset.has_value() && reset.value()) {
                resetOk = true;
                break;
            }

            sleep_ms(HOMER2_SHT4X_RESET_DELAY_MILLIS);
        }

        if (!resetOk)
            W(TAG, "could not reset SHT4x");

        auto serialOk = false;
        for (uint32_t i = 0; i < HOMER2_SHT4X_MAX_READ_SERIAL_RETRIES; i++) {

            std::optional<uint32_t> serial;
            try {
                serial = this->_sht4x->readSerial(now());
            }
            catch (...) {
                continue;
            }

            if (serial.has_value()) {
                I(TAG, "SHT4x serial number: " << serial.value());
                serialOk = true;
                break;
            }

            sleep_ms(HOMER2_SHT4X_READ_SERIAL_DELAY_MILLIS);
        }

        if (!serialOk)
            E(TAG, "could not read SHT4x serial");

        I(TAG, "SHT4x connected");
    }

    void Homer2Sensors::connectSgp40() noexcept {

        if (!is_enabled_sgp40()) {
            this->_sgp40 = nullptr;
            return;
        }

        if (SENSOR_ERROR_THRESHOLD <= this->_sgp40Errors) {
            W(TAG, "SGP40 has encountered too many errors, reconnecting: "
                << this->_sgp40Errors);
            this->_sgp40 = nullptr;
            this->_sgp40Errors = 0;
        }

        if (nullptr != this->_sgp40)
            return;

        try {
            this->_sgp40 = this->makeSgp40();
        }
        catch (std::exception& e) {
            E(TAG, "failed to create SGP40 sensor: " << e.what());
            this->_sgp40 = nullptr;
            return;
        }
        catch (...) {
            E(TAG, "failed to create SGP40 very badly! do not even know why");
            this->_sgp40 = nullptr;
            return;
        }

        auto resetOk = false;
        for (uint32_t i = 0; i < HOMER2_SGP40_MAX_RESET_RETRIES; i++) {

            std::optional<bool> reset;
            try {
                reset = this->_sgp40->reset(now());
            }
            catch (...) {
                continue;
            }

            if (reset.has_value() && reset.value()) {
                resetOk = true;
                break;
            }

            sleep_ms(HOMER2_SGP40_RESET_DELAY_MILLIS);
        }

        if (!resetOk)
            E(TAG, "could not reset SGP40");

        auto serialOk = false;
        for (uint32_t i = 0; i < HOMER2_SGP40_MAX_READ_SERIAL_RETRIES; i++) {

            const std::array<uint8_t, 6>* serial;
            try {
                serial = this->_sgp40->readSerial(now());
            }
            catch (...) {
                continue;
            }

            if (nullptr != serial) {
                I(TAG, "SGP40 serial number: "
                    << std::hex << std::setw(2) << std::setfill('0')
                    << static_cast<uint64_t>((*serial)[0]) << '-'
                    << std::hex << std::setw(2) << std::setfill('0')
                    << static_cast<uint64_t>((*serial)[1]) << '-'
                    << std::hex << std::setw(2) << std::setfill('0')
                    << static_cast<uint64_t>((*serial)[2]) << '-'
                    << std::hex << std::setw(2) << std::setfill('0')
                    << static_cast<uint64_t>((*serial)[3]) << '-'
                    << std::hex << std::setw(2) << std::setfill('0')
                    << static_cast<uint64_t>((*serial)[4]) << '-'
                    << std::hex << std::setw(2) << std::setfill('0')
                    << static_cast<uint64_t>((*serial)[5]));
                serialOk = true;
                break;
            }

            sleep_ms(HOMER2_SGP40_READ_SERIAL_DELAY_MILLIS);
        }

        if (!serialOk)
            E(TAG, "could not read SGP40 serial");

        I(TAG, "SGP40 connected");
    }

    void Homer2Sensors::connectBmp3xx() noexcept {

        if (!is_enabled_bmp3xx()) {
            this->_bmp3xx = nullptr;
            return;
        }

        if (SENSOR_ERROR_THRESHOLD <= this->_bmp3xxErrors) {
            W(TAG, "BMP3xx has encountered too many errors, reconnecting: "
                << this->_bmp3xxErrors);
            this->_bmp3xx = nullptr;
            this->_bmp3xxErrors = 0;
        }

        if (nullptr != this->_bmp3xx)
            return;

        try {
            this->_bmp3xx = this->makeBmp3xx();
        }
        catch (std::exception& e) {
            E(TAG, "failed to create BMP3xx sensor: " << e.what());
            this->_bmp3xx = nullptr;
        }
        catch (...) {
            E(TAG, "failed to create BMP3xx very badly! do not even know why");
            this->_bmp3xx = nullptr;
        }

        if (nullptr != this->_bmp3xx)
            I(TAG, "BMP3xx connected");
    }

    void Homer2Sensors::connectSunrise() noexcept {

        if (!is_enabled_sunrise()) {
            this->_sunrise = nullptr;
            return;
        }

        if (SENSOR_ERROR_THRESHOLD <= this->_sunriseErrors) {
            W(TAG, "Sunrise has encountered too many errors, reconnecting: "
                << this->_sunriseErrors);
            this->_sunrise = nullptr;
            this->_sunriseErrors = 0;
        }

        if (nullptr != this->_sunrise)
            return;

        try {
            this->_sunrise = this->makeSunrise();
        }
        catch (std::exception& e) {
            E(TAG, "failed to create Sunrise sensor: " << e.what());
            this->_sunrise = nullptr;
        }
        catch (...) {
            E(TAG, "failed to create Sunrise very badly! do not even know why");
            this->_sunrise = nullptr;
        }

        if (nullptr != this->_sunrise)
            I(TAG, "Sunrise connected");
    }

    void Homer2Sensors::connectPmsx00x() noexcept {

        if (!is_enabled_pmsx00x()) {
            this->_pmsx00x = nullptr;
            return;
        }

        if (SENSOR_ERROR_THRESHOLD <= this->_pmsx00xErrors) {
            W(TAG, "PMSx00x has encountered too many errors, reconnecting: "
                << this->_pmsx00xErrors);
            this->_pmsx00x = nullptr;
            this->_pmsx00xErrors = 0;
        }

        if (nullptr != this->_pmsx00x)
            return;

        try {
            this->_pmsx00x = this->makePmsx00x();
        }
        catch (std::exception& e) {
            E(TAG, "failed to create PMSx00x sensor: " << e.what());
            this->_pmsx00x = nullptr;
        }
        catch (...) {
            E(TAG, "failed to create PMSx00x very badly! do not even know why");
            this->_pmsx00x = nullptr;
        }

        if (nullptr != this->_pmsx00x)
            I(TAG, "PMSx00x connected");
    }


    [[nodiscard]]
    bool Homer2Sensors::isSht4xDataExpired() const noexcept {

        return is_expired(this->_lastSht4xDataTime, HOMER2_CACHED_DATA_EXPIRY_MILLIS);
    }

    [[nodiscard]]
    bool Homer2Sensors::isBme68xDataExpired() const noexcept {

        return is_expired(this->_lastBme68xDataTime, HOMER2_CACHED_DATA_EXPIRY_MILLIS);
    }

    [[nodiscard]]
    bool Homer2Sensors::isSgp40DataExpired() const noexcept {

        return is_expired(this->_lastSgp40DataTime, HOMER2_CACHED_DATA_EXPIRY_MILLIS);
    }

    [[nodiscard]]
    bool Homer2Sensors::isBmp3xxDataExpired() const noexcept {

        return is_expired(this->_lastBmp3xxDataTime, HOMER2_CACHED_DATA_EXPIRY_MILLIS);
    }

    [[nodiscard]]
    bool Homer2Sensors::isSunriseDataExpired() const noexcept {

        return is_expired(this->_lastSunriseDataTime, HOMER2_CACHED_DATA_EXPIRY_MILLIS);
    }

    [[nodiscard]]
    bool Homer2Sensors::isPmsx00xDataExpired() const noexcept {

        return is_expired(this->_lastPmsx00xDataTime, HOMER2_CACHED_DATA_EXPIRY_MILLIS);
    }


    [[nodiscard]]
    bool Homer2Sensors::hasAnySensor() const noexcept {

        return (nullptr != this->_sgp40) ||
               (nullptr != this->_sht4x) ||
               (nullptr != this->_sunrise) ||
               (nullptr != this->_bme68x) ||
               (nullptr != this->_bmp3xx) ||
               (nullptr != this->_pmsx00x);
    }


    void Homer2Sensors::querySgp40() noexcept {

        if (nullptr == this->_sgp40)
            return;

        const std::optional<HumiditySource> humiditySource = this->humiditySource();
        if (!humiditySource.has_value()) {
            D(4, TAG, "humidity source is unavailable, not querying SGP40");
            return;
        }

        const std::optional<TemperatureSource> temperatureSource = this->temperatureSource();
        if (!temperatureSource.has_value()) {
            D(4, TAG, "temperature source is unavailable, not querying SGP40");
            return;
        }

        float temperatureCelsius;
        switch (temperatureSource.value()) {
            case TemperatureSource::constant:
                temperatureCelsius = HOMER2_SOURCE_CONST_TEMPERATURE_CELSIUS;
                break;

            case TemperatureSource::bme68x:
                temperatureCelsius = this->_data.bme68xData()->getTemperatureCelsius();
                break;

            case TemperatureSource::sht4x:
                temperatureCelsius = this->_data.sht4xData()->getTemperatureCelsius();
                break;

            case TemperatureSource::bmp3xx:
                temperatureCelsius = this->_data.bmp3xxData()->getTemperatureCelsius();
                break;

            case TemperatureSource::disabled:
                D(4, TAG, "temperature source disabled for sgp40, not querying sensor");
                return;
        }

        float relativeHumidityPercent;
        switch (humiditySource.value()) {
            case HumiditySource::constant:
                relativeHumidityPercent = HOMER2_SOURCE_CONST_RELATIVE_HUMIDITY_PERCENT;
                break;

            case HumiditySource::bme68x:
                relativeHumidityPercent = this->_data.bme68xData()->getRelativeHumidityPercent();
                break;

            case HumiditySource::sht4x:
                relativeHumidityPercent = this->_data.sht4xData()->getRelativeHumidityPercent();
                break;

            case HumiditySource::disabled:
                D(4, TAG, "humidity source disabled for sgp40, not querying sensor");
                return;
        }

        const auto now = now();

        try {
            auto value = this->_sgp40->measure(
                now,
                temperatureCelsius,
                relativeHumidityPercent
            );

            if (value.has_value()) {
                this->_data = this->_data.withSgp40Data(value.value());
                this->_lastSgp40DataTime = now;
            }

            this->_sgp40Errors = 0;
        }
        catch (const std::exception& ex) {
            E(TAG, "SGP40 failure: " << ex.what());
            this->_sgp40Errors++;
        }
        catch (...) {
            E(TAG, "SGP40 failed, very badly! do not even know how");
            this->_sgp40Errors++;
        }
    }

    void Homer2Sensors::querySht4x() noexcept {

        if (nullptr == this->_sht4x)
            return;

        const auto now = now();

        try {
            auto value = this->_sht4x->measure(now);

            if (value.has_value()) {
                this->_data = this->_data.withSht4xData(value.value());
                this->_lastSht4xDataTime = now;
            }

            this->_sht4xErrors = 0;
        }
        catch (const std::exception& ex) {
            E(TAG, "SHT4x failure: " << ex.what());
            this->_sht4xErrors++;
        }
        catch (...) {
            E(TAG, "SHT4x failed, very badly! do not even know how");
            this->_sht4xErrors++;
        }
    }

    void Homer2Sensors::querySunrise() noexcept {

        if (nullptr == this->_sunrise)
            return;

        const auto now = now();

        try {
            auto value = this->_sunrise->measure(now);

            if (value.has_value()) {
                this->_data = this->_data.withSunriseData(value.value());
                this->_lastSunriseDataTime = now;
            }

            this->_sunriseErrors = 0;
        }
        catch (const std::exception& ex) {
            E(TAG, "Sunrise failure: " << ex.what());
            this->_sunriseErrors++;
        }
        catch (...) {
            E(TAG, "Sunrise failed, very badly! do not even know how");
            this->_sunriseErrors++;
        }
    }

    void Homer2Sensors::queryBme68x() noexcept {

        if (nullptr == this->_bme68x)
            return;

        const auto now = now();

        try {
            auto value = this->_bme68x->measure(now);

            if (value.has_value()) {
                this->_data = this->_data.withBme68xData(value.value());
                this->_lastBme68xDataTime = now;
            }

            this->_bme68xErrors = 0;
        }
        catch (const std::exception& ex) {
            E(TAG, "BME68x failure: " << ex.what());
            this->_bme68xErrors++;
        }
        catch (...) {
            E(TAG, "BME68x failed, very badly! do not even know how");
            this->_bme68xErrors++;
        }
    }

    void Homer2Sensors::queryBmp3xx() noexcept {

        if (nullptr == this->_bmp3xx)
            return;

        const auto now = now();

        try {
            auto value = this->_bmp3xx->measure(now);

            if (value.has_value()) {
                this->_data = this->_data.withBmp3xxData(value.value());
                this->_lastBmp3xxDataTime = now;
            }

            this->_bmp3xxErrors = 0;
        }
        catch (const std::exception& ex) {
            E(TAG, "BMP3xx failure: " << ex.what());
            this->_bmp3xxErrors++;
        }
        catch (...) {
            E(TAG, "BMP3xx failed, very badly! do not even know how");
            this->_bmp3xxErrors++;
        }
    }

    void Homer2Sensors::queryPmsx00x() noexcept {

        if (nullptr == this->_bmp3xx)
            return;

        const auto now = now();

        try {
            auto value = this->_pmsx00x->measure(now);

            if (value.has_value()) {
                this->_data = this->_data.withPmsx00xData(value.value());
                this->_lastPmsx00xDataTime = now;
            }

            this->_pmsx00xErrors = 0;
        }
        catch (const std::exception& ex) {
            E(TAG, "PMSx00x failure: " << ex.what());
            this->_pmsx00xErrors++;
        }
        catch (...) {
            E(TAG, "PMSx00x failed, very badly! do not even know how");
            this->_pmsx00xErrors++;
        }
    }

    void Homer2Sensors::querySensors() {

        if (!this->hasAnySensor())
            throw std::logic_error{"no sensor available to query"};

        this->queryPmsx00x();

        this->querySunrise();

        this->queryBmp3xx();

        this->querySht4x();

        this->queryBme68x();

        // Must be after Sht4x & Bme68x as depends on their data.
        this->querySgp40();
    }

    Homer2SensorsData Homer2Sensors::data() const noexcept {

        Homer2SensorsData data = this->_data;

        if (this->isBme68xDataExpired())
            data = data.withBme68xData(std::nullopt);

        if (this->isSht4xDataExpired())
            data = data.withSht4xData(std::nullopt);

        if (this->isBmp3xxDataExpired())
            data = data.withBmp3xxData(std::nullopt);

        if (this->isPmsx00xDataExpired())
            data = data.withPmsx00xData(std::nullopt);

        if (this->isSgp40DataExpired())
            data = data.withSgp40Data(std::nullopt);

        if (this->isSunriseDataExpired())
            data = data.withSunriseData(std::nullopt);

        return data;
    }

}
