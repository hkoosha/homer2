#include <limits>
#include <utility>

#include <homer2_logging.hpp>

#include "homer2_bmp3xx_sensor.hpp"
#include "../bosch/bmp3.h"

using homer2::i2c::Homer2I2c;
using homer2::i2c::I2cConnection;

namespace homer2::sensor::bmp3xx::internal::sensor {

    namespace {

        constexpr uint64_t I2C_TIMEOUT_MILLIS = 200;

        constexpr uint8_t I2C_ADDR_MAIN = 0x77;
        constexpr uint8_t I2C_ADDR_ALT = 0x76;

        constexpr uint64_t RESET_DURATION_MS = 1;

        [[nodiscard]]
        const char* translate(
            const int8_t error
        ) noexcept {
            switch (error) {
                case BMP3_OK:
                    return "native - OK";

                case BMP3_E_NULL_PTR:
                    return "native - NPE";

                case BMP3_E_DEV_NOT_FOUND:
                    return "native - device not found";

                case BMP3_E_INVALID_ODR_OSR_SETTINGS:
                    return "native - invalid odr settings";

                case BMP3_E_CMD_EXEC_FAILED:
                    return "native - command execution failed";

                case BMP3_E_CONFIGURATION_ERR:
                    return "native - configuration error";

                case BMP3_E_INVALID_LEN:
                    return "native - invalid data length";

                case BMP3_E_COMM_FAIL:
                    return "native - communication failure";

                case BMP3_W_SENSOR_NOT_ENABLED:
                    return "native - w - sensor not enabled";

                default:
                    return "native - unknown error";
            }
        }

        [[nodiscard]]
        int8_t calculate_crc(
            uint8_t seed,
            uint8_t data
        ) {
            for (uint8_t i = 0; i < 8; i++) {
                const int8_t v = (seed & 0x80) ^ (data & 0x80) ? 1 : 0;
                seed = (seed & 0x7F) << 1;
                data = (data & 0x7F) << 1;
                seed = seed ^ (uint8_t) (0x1D * v);
            }

            return static_cast<int8_t>(seed);
        }

    }

    BMP3xxSensor::BMP3xxSensor(
        std::shared_ptr<i2c::Homer2I2c> i2c,
        const bool useAltAddr,
        const BMP3xxIirFilterSize iirFilterSize,
        const BMP3xxOversampling temperatureOversampling,
        const BMP3xxOversampling pressureOversampling,
        const BMP3xxOutputDataRate outputDataRate
    ) :
        _i2c{
            I2cConnection{
                std::move(i2c),
                useAltAddr ? I2C_ADDR_ALT : I2C_ADDR_MAIN,
                I2C_TIMEOUT_MILLIS,
            }
        } {

        this->_data.pressure = std::numeric_limits<float>::quiet_NaN();
        this->_data.temperature = std::numeric_limits<float>::quiet_NaN();

        this->_dev.chip_id = useAltAddr ? I2C_ADDR_ALT : I2C_ADDR_MAIN;
        this->_dev.intf = bmp3_intf::BMP3_I2C_INTF;
        this->_dev.intf_ptr = static_cast<void*>(&this->_i2c);
        this->_dev.read = i2c::Helper::write_and_read_helper;
        this->_dev.write = i2c::Helper::write_helper;
        this->_dev.delay_us = i2c::Helper::delay_us;
        this->_dev.dummy_byte = 0;

        auto result = bmp3_soft_reset(&this->_dev);
        if (BMP3_OK != result) {
            E(TAG, "could not reset sensor: " << translate(result));
            throw std::runtime_error{"BMP3xx: could not reset sensor"};
        }

        result = bmp3_init(&this->_dev);
        if (BMP3_OK != result) {
            E(TAG, "could not initialize sensor: " << translate(result));
            throw std::runtime_error{"BMP3xx: could not initialize sensor"};
        }

        this->validateTrimmingParameters();

        D(4, TAG, "T01 = " << std::to_string(this->_dev.calib_data.reg_calib_data.par_t1));
        D(4, TAG, "T02 = " << std::to_string(this->_dev.calib_data.reg_calib_data.par_t2));
        D(4, TAG, "T03 = " << std::to_string(this->_dev.calib_data.reg_calib_data.par_t3));
        D(4, TAG, "P01 = " << std::to_string(this->_dev.calib_data.reg_calib_data.par_p1));
        D(4, TAG, "P02 = " << std::to_string(this->_dev.calib_data.reg_calib_data.par_p2));
        D(4, TAG, "P03 = " << std::to_string(this->_dev.calib_data.reg_calib_data.par_p3));
        D(4, TAG, "P04 = " << std::to_string(this->_dev.calib_data.reg_calib_data.par_p4));
        D(4, TAG, "P05 = " << std::to_string(this->_dev.calib_data.reg_calib_data.par_p5));
        D(4, TAG, "P06 = " << std::to_string(this->_dev.calib_data.reg_calib_data.par_p6));
        D(4, TAG, "P07 = " << std::to_string(this->_dev.calib_data.reg_calib_data.par_p7));
        D(4, TAG, "P08 = " << std::to_string(this->_dev.calib_data.reg_calib_data.par_p8));
        D(4, TAG, "P09 = " << std::to_string(this->_dev.calib_data.reg_calib_data.par_p9));
        D(4, TAG, "P10 = " << std::to_string(this->_dev.calib_data.reg_calib_data.par_p10));
        D(4, TAG, "P11 = " << std::to_string(this->_dev.calib_data.reg_calib_data.par_p11));
        D(4, TAG, "T LIN = " << std::to_string(this->_dev.calib_data.reg_calib_data.t_lin));

        this->_dev.settings.odr_filter.temp_os = static_cast<uint8_t>(temperatureOversampling);
        this->_dev.settings.odr_filter.press_os = static_cast<uint8_t>(pressureOversampling);
        this->_dev.settings.odr_filter.iir_filter = static_cast<uint8_t>(iirFilterSize);
        this->_dev.settings.odr_filter.odr = static_cast<uint8_t>(outputDataRate);

        this->_dev.settings.op_mode = BMP3_MODE_FORCED;
    }

    [[nodiscard]]
    bool BMP3xxSensor::measure(
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

    void BMP3xxSensor::doRequestMeasurement(
        const uint64_t nowMillis
    ) {
        D(1, TAG, "requesting measurement");
        this->_dataReadyAtMillis = 0;

        const auto components = BMP3_TEMP | BMP3_PRESS;

        this->_dev.settings.temp_en = BMP3_ENABLE;
        this->_dev.settings.press_en = BMP3_ENABLE;
        this->_dev.settings.op_mode = BMP3_MODE_FORCED;

        uint32_t settings = BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_EN;
        if (this->_dev.settings.odr_filter.temp_os != static_cast<uint8_t>(BMP3xxOversampling::off))
            settings |= BMP3_SEL_TEMP_OS;
        if (this->_dev.settings.odr_filter.press_os != static_cast<uint8_t>(BMP3xxOversampling::off))
            settings |= BMP3_SEL_PRESS_OS;
        if (this->_dev.settings.odr_filter.iir_filter != static_cast<uint8_t>(BMP3xxIirFilterSize::off))
            settings |= BMP3_SEL_IIR_FILTER;
        if (this->_dev.settings.odr_filter.odr != static_cast<uint8_t>(BMP3xxOutputDataRate::off))
            settings |= BMP3_SEL_ODR;

        auto result = bmp3_set_sensor_settings(settings, &this->_dev);
        if (BMP3_OK != result) {
            E(TAG, "failed to set sensor settings: " << translate(result));
            throw std::runtime_error{"BMP3xx: failed to set sensor settings"};
        }

        result = bmp3_set_op_mode(&this->_dev);
        if (BMP3_OK != result) {
            E(TAG, "failed to set sensor operation mode: " << translate(result));
            throw std::runtime_error{"BMP3xx: failed to set sensor operation mode"};
        }

        result = bmp3_get_sensor_data(components, &this->_data, &this->_dev);
        if (BMP3_OK != result) {
            E(TAG, "failed to get sensor data: " << translate(result));
            throw std::runtime_error{"BMP3xx: failed to get sensor data"};
        }

        this->_dataReadyAtMillis = nowMillis;
        D(3, TAG, "measurement to be ready at: " << nowMillis << " + "
                                                 << 0 << " = "
                                                 << this->_dataReadyAtMillis << "ms");
    }

    bool BMP3xxSensor::doReadMeasurement() {

        D(3, TAG, "measurement ready, reading");
        this->_dataReadyAtMillis = 0;

        D(5, TAG, "temperature (Celsius): " << this->_data.temperature);
        D(5, TAG, "pressure (hPa): " << this->_data.pressure);

        return true;
    }

    // ---------------------------------

    [[nodiscard]]
    bool BMP3xxSensor::reset(
        const uint64_t nowMillis
    ) {
        assert(nowMillis > 0);

        if (this->_dataReadyAtMillis == 0) {

            this->doReset(nowMillis);
            return false;

        }
        else if (this->_dataReadyAtMillis > nowMillis) {

            D(3, TAG, "sensor is restarting, to be ready at: "
                << this->_dataReadyAtMillis << "ms (" <<
                (this->_dataReadyAtMillis - nowMillis) << "ms left)");
            return false;

        }
        else {

            return this->doReadReset();

        }
    }

    void BMP3xxSensor::doReset(
        const uint64_t nowMillis
    ) {
        D(1, TAG, "resetting sensor");
        this->_dataReadyAtMillis = 0;

        const auto result = bmp3_soft_reset(&this->_dev);
        if (BMP3_OK != result) {
            E(TAG, "could not reset sensor: " << translate(result));
            throw std::runtime_error{"BMP3xx: could not reset sensor"};
        }

        this->_dataReadyAtMillis = nowMillis + RESET_DURATION_MS;

        D(3, TAG, "sensor to be ready after restart at: " << nowMillis << " +"
                                                          << RESET_DURATION_MS << " = "
                                                          << this->_dataReadyAtMillis << "ms");
    }

    [[nodiscard]]
    bool BMP3xxSensor::doReadReset() {

        D(1, TAG, "sensor should be restarted by now");
        return true;
    }

    // ---------------------------------

    void BMP3xxSensor::validateTrimmingParameters() {

        uint8_t trim_param[21];
        auto result = bmp3_get_regs(
            BMP3_REG_CALIB_DATA,
            trim_param,
            21,
            &this->_dev
        );

        if (BMP3_OK != result) {
            E(TAG, "trim parameters validation failed");
            throw std::runtime_error{"BMP3: trim parameters validation failed"};
        }

        uint8_t expected_crc = 0xFF;
        for (unsigned char i: trim_param)
            expected_crc = (uint8_t) calculate_crc(expected_crc, i);
        expected_crc = (expected_crc ^ 0xFF);

        uint8_t crc = 0;
        bmp3_get_regs(0x30, &crc, 1, &this->_dev);

        if (expected_crc != crc) {
            E(TAG, "trim parameters validation failed, crc mismatch: " << expected_crc << " != " << crc);
            throw std::runtime_error{"BMP3: trim parameters validation failed, crc mismatch"};
        }
    }

    // ---------------------------------

    [[nodiscard]]
    double BMP3xxSensor::getTemperatureCelsius() const noexcept {

        return this->_data.temperature;
    }

    [[nodiscard]]
    double BMP3xxSensor::getPressureHPa() const noexcept {

        return this->_data.pressure;
    }


}
