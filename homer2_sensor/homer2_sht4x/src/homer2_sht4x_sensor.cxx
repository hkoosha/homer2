#include <map>
#include <string_view>
#include <utility>

#include <homer2_logging.hpp>
#include <homer2_i2c.hpp>

#include "homer2_sht4x_base.hpp"
#include "homer2_sht4x_sensor.hpp"

using homer2::i2c::Homer2I2c;
using homer2::i2c::I2cConnection;
using homer2::i2c::Homer2I2cError;

namespace homer2::sensor::sht4x::internal::sensor {

    namespace {

        constexpr uint64_t I2C_TIMEOUT_MILLIS = 500;

        constexpr uint8_t I2C_ADDR_MAIN = 0x44;
        constexpr uint8_t I2C_ADDR_ALT = 0x45;

        constexpr uint64_t READ_SERIAL_DURATION_MS = 10;
        constexpr uint64_t RESET_DURATION_MS = 10;

        enum class Command : uint8_t {
            read_serial = 0x89,
            reset = 0x95,
            read_no_heater_high_precision = 0xFD,
            read_no_heater_medium_precision = 0xF6,
            read_no_heater_low_precision = 0xE0,
            read_1000ms_high_heater_high_precision = 0x39,
            read_100ms_high_heater_high_precision = 0x32,
            read_1000ms_medium_heater_high_precision = 0x2F,
            read_100ms_medium_heater_high_precision = 0x24,
            read_1000ms_low_heater_high_precision = 0x1E,
            read_100ms_low_heater_high_precision = 0x15,
        };

        std::ostream& operator<<(
            std::ostream& out,
            Command value
        ) {

            static std::map<Command, std::string_view> strings{
                {Command::read_serial,                              "read_serial"},
                {Command::reset,                                    "reset"},
                {Command::read_no_heater_high_precision,            "read_no_heater_high_precision"},
                {Command::read_no_heater_medium_precision,          "read_no_heater_medium_precision"},
                {Command::read_no_heater_low_precision,             "read_no_heater_low_precision"},
                {Command::read_1000ms_high_heater_high_precision,   "read_1000ms_high_heater_high_precision"},
                {Command::read_100ms_high_heater_high_precision,    "read_100ms_high_heater_high_precision"},
                {Command::read_1000ms_medium_heater_high_precision, "read_1000ms_medium_heater_high_precision"},
                {Command::read_100ms_medium_heater_high_precision,  "read_100ms_medium_heater_high_precision"},
                {Command::read_1000ms_low_heater_high_precision,    "read_1000ms_low_heater_high_precision"},
                {Command::read_100ms_low_heater_high_precision,     "read_100ms_low_heater_high_precision"},
            };

            return out << strings[value];
        }


        /**
         * From SHT4x datasheet.
         */
        [[nodiscard]]
        bool crc_matches(
            const uint8_t msb,
            const uint8_t lsb,
            const uint8_t expected_crc
        ) noexcept {

            uint8_t crc = 0xFF ^ msb;

            for (uint8_t i = 0; i < 8; i++)
                crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;

            crc ^= lsb;
            for (uint8_t i = 0; i < 8; i++)
                crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;

            return crc == expected_crc;
        }

        [[nodiscard]]
        uint64_t measurementDuration(
            HeaterConf heaterConf,
            Precision precision
        ) {

            if (HeaterConf::off == heaterConf) {
                switch (precision) {
                    case Precision::low:
                        return 2;

                    case Precision::medium:
                        return 5;

                    case Precision::high:
                        return 10;
                }
            }
            else {
                    hard_assert(Precision::high == precision);

                switch (heaterConf) {
#pragma clang diagnostic push
#pragma ide diagnostic ignored "UnreachableCode"
                    case HeaterConf::off:
                        panic("unreachable");
#pragma clang diagnostic pop

                    case HeaterConf::low_100ms:
                        return 110;

                    case HeaterConf::low_1000ms:
                        return 1100;

                    case HeaterConf::medium_100ms:
                        return 110;

                    case HeaterConf::medium_1000ms:
                        return 1100;

                    case HeaterConf::high_100ms:
                        return 110;

                    case HeaterConf::high_1000ms:
                        return 1100;
                }
            }

            throw std::logic_error{"unknown state in measurementDuration()"};
        }

        [[nodiscard]]
        Command command(
            HeaterConf heaterConf,
            Precision precision
        ) {

            if (HeaterConf::off == heaterConf) {
                switch (precision) {
                    case Precision::low:
                        return Command::read_no_heater_low_precision;

                    case Precision::medium:
                        return Command::read_no_heater_medium_precision;

                    case Precision::high:
                        return Command::read_no_heater_high_precision;
                }
            }
            else {
                    hard_assert(Precision::high == precision);

                switch (heaterConf) {
#pragma clang diagnostic push
#pragma ide diagnostic ignored "UnreachableCode"
                    case HeaterConf::off:
                        panic("unreachable");
#pragma clang diagnostic pop

                    case HeaterConf::low_100ms:
                        return Command::read_100ms_low_heater_high_precision;

                    case HeaterConf::low_1000ms:
                        return Command::read_1000ms_low_heater_high_precision;

                    case HeaterConf::medium_100ms:
                        return Command::read_100ms_medium_heater_high_precision;

                    case HeaterConf::medium_1000ms:
                        return Command::read_1000ms_medium_heater_high_precision;

                    case HeaterConf::high_100ms:
                        return Command::read_100ms_high_heater_high_precision;

                    case HeaterConf::high_1000ms:
                        return Command::read_1000ms_high_heater_high_precision;
                }
            }

            throw std::logic_error{"unknown state in command()"};
        }

    }

    SHT4xSensor::SHT4xSensor(
        std::shared_ptr<Homer2I2c> i2c,
        const bool useAltAddr,
        const Precision precision,
        const HeaterConf heaterConf
    ) :
        _heaterConf{heaterConf},
        _precision{precision},
        _i2c{
            I2cConnection{
                std::move(i2c),
                useAltAddr ? I2C_ADDR_ALT : I2C_ADDR_MAIN,
                I2C_TIMEOUT_MILLIS
            }
        } {

        if (heaterConf != HeaterConf::off && precision != Precision::high) {
            E(TAG, "invalid combination of heater conf and precision, when heater is on precision should be high"
                << " heater conf: " << heaterConf << ", precision: " << precision);
            throw std::logic_error{
                "invalid combination of heater conf and precision, when heater is on precision should be high"
            };
        }

        I(TAG, "i2c addr: 0x" << std::hex << static_cast<uint64_t>(useAltAddr ? I2C_ADDR_ALT : I2C_ADDR_MAIN));
        I(TAG, "heater conf: " << this->_heaterConf);
        I(TAG, "precision: " << this->_precision);
        I(TAG, "measurement duration: "
            <<
            std::to_string(measurementDuration(this->_heaterConf, this->_precision))
            << "ms");
        I(TAG, "i2c command: " << command(this->_heaterConf, this->_precision));
    }

    // =================================

    [[nodiscard]]
    bool SHT4xSensor::measure(
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

    void SHT4xSensor::doRequestMeasurement(
        const uint64_t nowMillis
    ) {
        D(1, TAG, "requesting measurement");
        this->_dataReadyAtMillis = 0;

        const Command cmd = command(this->_heaterConf, this->_precision);
        this->_i2c[0] = static_cast<uint8_t>(cmd);

        const auto result = this->_i2c.write(1);
        if (Homer2I2cError::no_error != result) {
            E(TAG, "failed to request measurement: " << result);
            throw std::runtime_error{"SHT4x: failed to request measurement"};
        }

        const uint64_t measurementDurationMillis = measurementDuration(this->_heaterConf, this->_precision);
        this->_dataReadyAtMillis = nowMillis + measurementDurationMillis;

        D(3, TAG, "measurement to be ready at: " << nowMillis << " + "
                                                 << measurementDurationMillis << " = "
                                                 << this->_dataReadyAtMillis << "ms");
    }

    [[nodiscard]]
    bool SHT4xSensor::doReadMeasurement() {

        D(3, TAG, "measurement ready, reading");
        this->_dataReadyAtMillis = 0;

        const auto result = this->_i2c.read(6);
        if (Homer2I2cError::no_error != result) {
            E(TAG, "failed to read measurement: " << result);
            throw std::runtime_error{"SHT4x: failed to read measurement"};
        }

        if (!crc_matches(this->_i2c[0], this->_i2c[1], this->_i2c[2])) {
            W(TAG, "crc mismatch while reading temperature");
            throw std::runtime_error{"SHT4x: CRC mismatch for temperature"};
        }
        if (!crc_matches(this->_i2c[3], this->_i2c[4], this->_i2c[5])) {
            W(TAG, "crc mismatch while reading relative humidity");
            throw std::runtime_error{"SHT4x: CRC mismatch for humidity"};
        }

        const uint16_t temperatureCelsius = (this->_i2c[0] << 8) + this->_i2c[1];
        const uint16_t relativeHumidityPercent = (this->_i2c[3] << 8) + this->_i2c[4];

        this->_temperatureCelsius = static_cast<float>(temperatureCelsius) * 0.00267033f - 45.f;
        this->_relativeHumidityPercent = static_cast<float>(relativeHumidityPercent) * 0.0015259f;

        D(5, TAG, "temperature celsius: " << std::to_string(this->_temperatureCelsius));
        D(5, TAG, "relative humidity percent: " << std::to_string(this->_relativeHumidityPercent));
        return true;
    }

    // =================================

    [[nodiscard]]
    bool SHT4xSensor::readSerial(
        const uint64_t nowMillis
    ) {
        assert(nowMillis > 0);

        if (this->_dataReadyAtMillis == 0) {

            this->doRequestSerial(nowMillis);
            return false;

        }
        else if (this->_dataReadyAtMillis > nowMillis) {

            D(3, TAG, "serial number not ready yet, to be ready at: "
                << this->_dataReadyAtMillis << "ms (" <<
                (this->_dataReadyAtMillis - nowMillis) << "ms left)");
            return false;

        }
        else {

            return this->doReadSerial();

        }
    }

    void SHT4xSensor::doRequestSerial(
        const uint64_t nowMillis
    ) {
        D(1, TAG, "requesting serial number");
        this->_dataReadyAtMillis = 0;

        this->_i2c[0] = static_cast<uint8_t>(Command::read_serial);

        const auto result = this->_i2c.write(1);
        if (result != Homer2I2cError::no_error) {
            E(TAG, "failed to request serial number: " << result);
            throw std::runtime_error{"SHT4x: failed to request serial number"};
        }

        this->_dataReadyAtMillis = nowMillis + READ_SERIAL_DURATION_MS;

        D(3, TAG, "serial number to be ready at: " << nowMillis << " + "
                                                   << READ_SERIAL_DURATION_MS << " = "
                                                   << this->_dataReadyAtMillis << "ms");
    }

    [[nodiscard]]
    bool SHT4xSensor::doReadSerial() {

        D(1, TAG, "reading serial number");
        this->_dataReadyAtMillis = 0;

        const auto result = this->_i2c.read(6);
        if (Homer2I2cError::no_error != result) {
            E(TAG, "failed to read serial number: " << result);
            throw std::runtime_error{"SHT4x: failed to read serial number"};
        }

        if (!crc_matches(this->_i2c[0], this->_i2c[1], this->_i2c[2]) ||
            !crc_matches(this->_i2c[3], this->_i2c[4], this->_i2c[5])) {
            W(TAG, "crc mismatch for serialNumber");
            throw std::runtime_error{"SHT4x: crc mismatch while reading serial number"};
        }

        this->_serial = 0;
        this->_serial = this->_i2c[0];
        this->_serial <<= 8;
        this->_serial |= this->_i2c[1];
        this->_serial <<= 8;
        this->_serial |= this->_i2c[3];
        this->_serial <<= 8;
        this->_serial |= this->_i2c[4];

        D(5, TAG, "serial: " << this->_serial);

        return true;
    }

    // =================================

    [[nodiscard]]
    bool SHT4xSensor::reset(
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

    void SHT4xSensor::doReset(
        const uint64_t nowMillis
    ) {
        D(1, TAG, "resetting sensor");
        this->_dataReadyAtMillis = 0;

        this->_i2c[0] = static_cast<uint8_t>(Command::reset);

        const auto result = this->_i2c.write(1);
        if (result != Homer2I2cError::no_error) {
            E(TAG, "failed to reset sensor: " << result);
            throw std::runtime_error{"SHT4x: failed to reset sensor"};
        }

        this->_dataReadyAtMillis = nowMillis + RESET_DURATION_MS;

        D(3, TAG, "sensor to be ready after restart at: " << nowMillis << " +"
                                                          << RESET_DURATION_MS << " = "
                                                          << this->_dataReadyAtMillis << "ms");
    }

    [[nodiscard]]
    bool SHT4xSensor::doReadReset() {

        D(1, TAG, "sensor should be restarted by now");
        return true;
    }

    // =================================

    [[nodiscard]]
    float SHT4xSensor::getTemperatureCelsius() const noexcept {

        return this->_temperatureCelsius;
    }

    [[nodiscard]]
    float SHT4xSensor::getRelativeHumidityPercent() const noexcept {

        return this->_relativeHumidityPercent;
    }

    [[nodiscard]]
    uint32_t SHT4xSensor::getSerial() const noexcept {

        return this->_serial;
    }

}
