#include <utility>

#include <homer2_logging.hpp>

#include "homer2_bme68x_sensor.hpp"
#include "../bosch/bme68x.h"

using homer2::i2c::Homer2I2c;
using homer2::i2c::I2cConnection;

namespace homer2::sensor::bme68x::internal::sensor {

    namespace {

        constexpr uint64_t I2C_TIMEOUT_MILLIS = 500;

        constexpr uint8_t I2C_ADDR_MAIN = 0x77;
        constexpr uint8_t I2C_ADDR_ALT = 0x76;

        [[nodiscard]]
        const char* translate(
            const int8_t error
        ) noexcept {
            switch (error) {
                case BME68X_OK:
                    return "native - OK";

                case BME68X_E_NULL_PTR:
                    return "native - NPE";

                case BME68X_E_COM_FAIL:
                    return "native - communication failure";

                case BME68X_E_DEV_NOT_FOUND:
                    return "native - device not found";

                case BME68X_E_INVALID_LENGTH:
                    return "native - bad data length";

                case BME68X_E_SELF_TEST:
                    return "native - self test failure";

                case BME68X_W_DEFINE_OP_MODE:
                    return "native - w - define op mode";

                case BME68X_W_NO_NEW_DATA:
                    return "native - w - no new data";

                case BME68X_W_DEFINE_SHD_HEATR_DUR:
                    return "native - w - define shared heater duration";

                default:
                    return "native - unknown error";
            }
        }

    }

    BME68xSensor::BME68xSensor(
        std::shared_ptr<i2c::Homer2I2c> i2c,
        const bool useAltAddr,
        const BME68xIirFilterSize iirFilterSize,
        const BME68xOversampling temperatureOversampling,
        const BME68xOversampling pressureOversampling,
        const BME68xOversampling humidityOversampling,
        const uint16_t gasHeaterTemperatureCelsius,
        const uint16_t gasHeaterDurationMillis,
        const int8_t ambientTemperatureCelsius
    ) :
        _i2c{
            I2cConnection{
                std::move(i2c),
                useAltAddr ? I2C_ADDR_ALT : I2C_ADDR_MAIN,
                I2C_TIMEOUT_MILLIS,
            }
        } {

        this->_dev.chip_id = useAltAddr ? I2C_ADDR_ALT : I2C_ADDR_MAIN;
        this->_dev.intf = bme68x_intf::BME68X_I2C_INTF;
        this->_dev.intf_ptr = (void*) this;
        this->_dev.read = i2c::Helper::write_and_read_helper;
        this->_dev.write = i2c::Helper::write_helper;
        this->_dev.delay_us = i2c::Helper::delay_us;
        this->_dev.amb_temp = ambientTemperatureCelsius;

        if (gasHeaterDurationMillis == 0 || gasHeaterTemperatureCelsius == 0) {
            this->_heaterConf.enable = BME68X_DISABLE;
        }
        else {
            this->_heaterConf.enable = BME68X_ENABLE;
            this->_heaterConf.heatr_temp = gasHeaterTemperatureCelsius;
            this->_heaterConf.heatr_dur = gasHeaterDurationMillis;
        }

        this->_conf.os_temp = static_cast<uint8_t>(temperatureOversampling);
        this->_conf.os_hum = static_cast<uint8_t>(humidityOversampling);
        this->_conf.os_pres = static_cast<uint8_t>(pressureOversampling);
        this->_conf.filter = static_cast<uint8_t>(iirFilterSize);

        I(TAG, "i2c addr: 0x" << std::hex << static_cast<uint64_t>(this->_dev.chip_id));
        I(TAG, "heater enabled: " << (this->_heaterConf.enable ? "yes" : "no"));
        I(TAG, "heater duration millis: " << gasHeaterDurationMillis);
        I(TAG, "heater temperature Celsius: " << gasHeaterTemperatureCelsius);
        I(TAG, "temperature oversampling: " << temperatureOversampling);
        I(TAG, "humidity oversampling: " << humidityOversampling);
        I(TAG, "pressure oversampling: " << pressureOversampling);
        I(TAG, "IIR filter size: " << iirFilterSize);

        int8_t result;

        D(0, TAG, "i2c init...");
        result = bme68x_init(&this->_dev);
        if (BME68X_OK != result) {
            E(TAG, "i2c initialization failed: " << translate(result));
            throw std::runtime_error{"BME68x: initialization over i2c failed"};
        }
        D(2, TAG, "i2c init ok");

        D(0, TAG, "configuring sensor...");
        result = bme68x_set_conf(&this->_conf, &this->_dev);
        if (BME68X_OK != result) {
            E(TAG, "sensor configuration failed: " << translate(result));
            throw std::runtime_error{"BME68x: sensor configuration failed"};
        }
        D(2, TAG, "sensor configuration ok");

        D(0, TAG, "configuring heater...");
        result = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &this->_heaterConf, &this->_dev);
        if (BME68X_OK != result) {
            E(TAG, "heater configuration failed: " << translate(result));
            throw std::runtime_error{"BME68x: heater configuration failed"};
        }
        D(2, TAG, "heater configuration ok");

        D(0, TAG, "heater range: " << std::to_string(this->_dev.calib.res_heat_range));
        D(0, TAG, "heater value: " << std::to_string(this->_dev.calib.res_heat_val));
        D(0, TAG, "SW error coefficient: " << std::to_string(this->_dev.calib.range_sw_err));
    }

    [[nodiscard]]
    bool BME68xSensor::measure(
        const uint64_t nowMillis
    ) {
        assert(nowMillis > 0);

        if (this->_dataReadyAtMillis == 0) {

            this->doRequestMeasurement(nowMillis);
            return false;

        }
        else if (this->_dataReadyAtMillis > nowMillis) {

            D(3, TAG, "data not ready yet, to be ready at: "
                << this->_dataReadyAtMillis << "ms (" <<
                (this->_dataReadyAtMillis - nowMillis) << "ms left)");
            return false;

        }
        else {

            return this->doReadMeasurement();

        }
    }

    void BME68xSensor::doRequestMeasurement(
        const uint64_t nowMillis
    ) {
        D(1, TAG, "requesting measurement");
        this->_dataReadyAtMillis = 0;

        D(4, TAG, "setting operation mode to forced");
        const int8_t result = bme68x_set_op_mode(BME68X_FORCED_MODE, &this->_dev);
        if (BME68X_OK != result) {
            E(TAG, "failed to set sensor operation mode to forced: " << translate(result));
            throw std::runtime_error{"BME68x: failed to set sensor operation mode to forced"};
        }
        D(4, TAG, "set operating mode to forced");

        D(3, TAG, "reading measurement duration");
        uint32_t duration_us = bme68x_get_meas_dur(BME68X_FORCED_MODE, &this->_conf, &this->_dev);
        if (duration_us == 0) {
            throw std::runtime_error{"BME68x: failed to get measurement duration"};
        }

        duration_us += this->_heaterConf.heatr_dur * 1000;
        this->_dataReadyAtMillis = nowMillis + (duration_us / 1000) + 1;

        D(3, TAG, "measurement to be ready at: " << nowMillis << " + "
                                                 << (duration_us / 1000) << " = "
                                                 << this->_dataReadyAtMillis << "ms");
    }

    bool BME68xSensor::doReadMeasurement() {

        D(3, TAG, "measurement ready, reading");
        this->_dataReadyAtMillis = 0;

        bme68x_data data{};
        uint8_t n_fields;
        const int8_t result = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &this->_dev);

        if (BME68X_W_NO_NEW_DATA == result) {
            D(2, TAG, "no new data");
            return false;
        }

        if (BME68X_OK != result) {
            E(TAG, "failed to read sensor measurement: "
                << translate(result) << ", "
                << std::to_string(result));
            throw std::runtime_error{"BME68x: failed to read sensor measurement"};
        }

        if (n_fields) {
            this->_temperatureCelsius = data.temperature;
            this->_relativeHumidityPercent = data.humidity;
            this->_pressureHPa = data.pressure;
            this->_gasResistanceOhms = data.status & (BME68X_HEAT_STAB_MSK | BME68X_GASM_VALID_MSK)
                                       ? data.gas_resistance
                                       : 0;

            D(5, TAG, "temperature (Celsius): " << this->_temperatureCelsius);
            D(5, TAG, "relative humidity (percent): " << this->_relativeHumidityPercent);
            D(5, TAG, "pressure (hPA): " << this->_pressureHPa);
            D(5, TAG, "gas resistance (Ohms): " << this->_gasResistanceOhms);
            return true;
        }
        else {
            throw std::runtime_error{"BME68x: no measurement from sensor"};
        }
    }


    [[nodiscard]]
    float BME68xSensor::getTemperatureCelsius() const noexcept {

        return this->_temperatureCelsius;
    }

    [[nodiscard]]
    float BME68xSensor::getPressureHPa() const noexcept {

        return this->_pressureHPa;
    }

    [[nodiscard]]
    float BME68xSensor::getRelativeHumidityPercent() const noexcept {

        return this->_relativeHumidityPercent;
    }

    [[nodiscard]]
    float BME68xSensor::getGasResistanceOhms() const noexcept {

        return this->_gasResistanceOhms;
    }

}
