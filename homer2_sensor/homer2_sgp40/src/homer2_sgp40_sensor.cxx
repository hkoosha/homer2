#include <cmath>
#include <utility>

#include <homer2_logging.hpp>
#include <homer2_i2c.hpp>

#include "homer2_sgp40_sensor.hpp"

using homer2::i2c::Homer2I2c;
using homer2::i2c::I2cConnection;

namespace homer2::sensor::sgp40::internal::sensor {

    using i2c::Homer2I2cError;

    namespace {

        constexpr uint64_t I2C_TIMEOUT_MILLIS = 500;
        constexpr uint8_t I2C_ADDR = 0x59;

        constexpr uint16_t U_INT_16_HI_BITS = 0xFF00;
        constexpr uint16_t U_INT_16_LO_BITS = 0x00FF;

        constexpr uint8_t RESET_CMD_HI = 0x00;
        constexpr uint8_t RESET_CMD_LO = 0x06;
        constexpr uint64_t RESET_DURATION_MILLIS = 10;

        constexpr uint8_t HEATER_OFF_CMD_HI = 0x36;
        constexpr uint8_t HEATER_OFF_CMD_LO = 0x15;
        constexpr uint64_t HEATER_OFF_DURATION_MILLIS = 50;

        constexpr uint8_t SELF_TEST_CMD_HI = 0x28;
        constexpr uint8_t SELF_TEST_CMD_LO = 0x0E;
        constexpr uint64_t SELF_TEST_DURATION_MILLIS = 500;
        constexpr uint16_t SELF_TEST_REPLY = 0xD400;

        constexpr uint8_t MEASURE_CMD_HI = 0x26;
        constexpr uint8_t MEASURE_CMD_LO = 0x0F;
        constexpr uint64_t MEASUREMENT_DURATION_MILLIS = 250;

        constexpr uint8_t SERIAL_NUMBER_CMD_HI = 0x36;
        constexpr uint8_t SERIAL_NUMBER_CMD_LO = 0x82;
        constexpr uint64_t SERIAL_NUMBER_DURATION_MILLIS = 10;

        constexpr uint8_t FEATURE_SET_CMD_HI = 0x20;
        constexpr uint8_t FEATURE_SET_CMD_LO = 0x2F;
        constexpr uint64_t FEATURE_SET_DURATION_MILLIS = 10;

        uint8_t calculate_crc(
            const uint8_t data0,
            const uint8_t data1
        ) {
            uint8_t crc = 0xFF;

            crc ^= data0;
            for (uint8_t b = 0; b < 8; b++)
                if (crc & 0x80)
                    crc = (crc << 1) ^ 0x31;
                else
                    crc <<= 1;

            crc ^= data1;
            for (uint8_t b = 0; b < 8; b++)
                if (crc & 0x80)
                    crc = (crc << 1) ^ 0x31;
                else
                    crc <<= 1;

            return crc;
        }

        template<typename T>
        bool check_crc(
            const T& buffer,
            const uint8_t len
        ) {
            for (int i = 0; i < len; ++i) {
                const auto j = i * 3;

                const uint8_t crc = calculate_crc(buffer[j], buffer[j + 1]);
                if (crc != buffer[j + 2]) {
                    W(TAG, "crc mismatch at index: " << j);
                    return false;
                }
            }

            return true;
        }

    }

    SGP40Sensor::SGP40Sensor(
#pragma clang diagnostic push
#pragma ide diagnostic ignored "UnusedParameter"
        std::shared_ptr<i2c::Homer2I2c> i2c
#pragma clang diagnostic pop
    ) noexcept:
        _i2c{
            I2cConnection{
                std::move(i2c),
                I2C_ADDR,
                I2C_TIMEOUT_MILLIS,
            }
        } {

        VocAlgorithm_init(&this->_vocAlgorithmParams);
    }


    [[nodiscard]]
    uint16_t SGP40Sensor::getRawIndex() const noexcept {

        return this->_rawIndex;
    }

    [[nodiscard]]
    int32_t SGP40Sensor::getVocIndex() const noexcept {

        return this->_vocIndex;
    }

    [[nodiscard]]
    const std::array<uint8_t, 6>& SGP40Sensor::getSerialNumber() const noexcept {

        return this->_serialNumber;
    }

    [[nodiscard]]
    uint16_t SGP40Sensor::getFeatureSet() const noexcept {

        return this->_featureSet;
    }

    [[nodiscard]]
    bool SGP40Sensor::getSelfTest() const noexcept {

        return this->_selfTestOk;
    }


    [[nodiscard]]
    bool SGP40Sensor::measure(
        const uint64_t nowMillis,
        const float temperatureCelsius,
        const float relativeHumidityPercent
    ) {
        assert(nowMillis > 0);

        if (this->_dataReadyAtMillis == 0) {

            this->doRequestMeasurement(nowMillis, temperatureCelsius, relativeHumidityPercent);
            return false;

        }
        else if (this->_dataReadyAtMillis > nowMillis) {

            D(3, TAG, "measurement is not ready yet, to be ready at: "
                << this->_dataReadyAtMillis << "ms (" <<
                (this->_dataReadyAtMillis - nowMillis) << "ms left)");
            return false;

        }
        else {

            return this->doReadMeasurement();

        }
    }

    void SGP40Sensor::doRequestMeasurement(
        const uint64_t nowMillis,
        const float temperatureCelsius,
        const float relativeHumidityPercent
    ) {
        D(1, TAG, "requesting measurement");

        this->_i2c[0] = MEASURE_CMD_HI;
        this->_i2c[1] = MEASURE_CMD_LO;

        const auto relativeHumidityTicks = static_cast<uint16_t>( std::lround(
            relativeHumidityPercent * 65535 / 100 + 0.5
        ));
        this->_i2c[2] = relativeHumidityTicks >> 8;
        this->_i2c[3] = relativeHumidityTicks & 0xFF;
        this->_i2c[4] = calculate_crc(this->_i2c[2], this->_i2c[3]);

        const auto temperatureTicks = static_cast<uint16_t>(((temperatureCelsius + 45) * 65535) / 175);
        this->_i2c[5] = temperatureTicks >> 8;
        this->_i2c[6] = temperatureTicks & 0xFF;
        this->_i2c[7] = calculate_crc(this->_i2c[5], this->_i2c[6]);

        const Homer2I2cError result = this->_i2c.write(8);
        if (Homer2I2cError::no_error != result) {
            this->_dataReadyAtMillis = 0;
            E(TAG, "failed to request measurement: " << result);
            throw std::runtime_error{"SGP40: failed to request measurement"};
        }

        this->_dataReadyAtMillis = nowMillis + MEASUREMENT_DURATION_MILLIS + 1;

        D(3, TAG, "measurement to be ready at: " << std::to_string(nowMillis) << " + "
                                                 << std::to_string(MEASUREMENT_DURATION_MILLIS + 1)
                                                 << " = "
                                                 << std::to_string(this->_dataReadyAtMillis)
                                                 << "ms");
    }

    bool SGP40Sensor::doReadMeasurement() {

        D(3, TAG, "measurement ready, reading");
        this->_dataReadyAtMillis = 0;

        const auto result = this->read(1);
        if (Homer2I2cError::no_error != result) {
            this->_dataReadyAtMillis = 0;
            E(TAG, "failed to read measurement: " << result);
            throw std::runtime_error{"SGP40: failed to read measurement measurement"};
        }

        this->_rawIndex = this->convertToUInt16(0);

        VocAlgorithm_process(
            &this->_vocAlgorithmParams,
            static_cast<int32_t>(this->_rawIndex),
            &this->_vocIndex
        );

        D(4, TAG, "raw index: " << this->_rawIndex << ", voc index: " << this->_vocIndex);
        return true;
    }


    [[nodiscard]]
    bool SGP40Sensor::reset(const uint64_t nowMillis) {

        assert(nowMillis > 0);

        if (this->_dataReadyAtMillis == 0) {

            this->doRequestReset(nowMillis);
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


    void SGP40Sensor::doRequestReset(const uint64_t nowMillis) {

        D(1, TAG, "requesting reset");

        this->_i2c[0] = RESET_CMD_HI;
        this->_i2c[1] = RESET_CMD_LO;
        const auto result = this->_i2c.write(2);
        if (Homer2I2cError::no_error != result) {
            this->_dataReadyAtMillis = 0;
            E(TAG, "failed to request reset: " << result);
            throw std::runtime_error{"SGP40: failed to request reset"};
        }

        this->_dataReadyAtMillis = nowMillis + RESET_DURATION_MILLIS;

        D(3, TAG, "sensor restarting, to be ready at: " << nowMillis << " + "
                                                        << RESET_DURATION_MILLIS << " = "
                                                        << this->_dataReadyAtMillis << "ms");
    }

    [[nodiscard]]
    bool SGP40Sensor::doReadReset() {

        D(1, TAG, "sensor should be restarted by now");
        this->_dataReadyAtMillis = 0;
        return true;
    }


    [[nodiscard]]
    bool SGP40Sensor::turnHeaterOff(const uint64_t nowMillis) {

        assert(nowMillis > 0);

        if (this->_dataReadyAtMillis == 0) {

            this->doRequestHeaterOff(nowMillis);
            return false;

        }
        else if (this->_dataReadyAtMillis > nowMillis) {

            D(3, TAG, "sensor is switching heater off, to be ready at: "
                << this->_dataReadyAtMillis << "ms (" <<
                (this->_dataReadyAtMillis - nowMillis) << "ms left)");
            return false;

        }
        else {

            return this->doReadHeaterOff();

        }
    }

    void SGP40Sensor::doRequestHeaterOff(const uint64_t nowMillis) {

        D(1, TAG, "requesting heater off");

        this->_i2c[0] = HEATER_OFF_CMD_HI;
        this->_i2c[1] = HEATER_OFF_CMD_LO;
        const auto result = this->_i2c.write(2);
        if (Homer2I2cError::no_error != result) {
            this->_dataReadyAtMillis = 0;
            E(TAG, "failed to request heater off: " << result);
            throw std::runtime_error{"SGP40: failed to request heater off"};
        }

        this->_dataReadyAtMillis = nowMillis + HEATER_OFF_DURATION_MILLIS;

        D(3, TAG, "heater to be off at: " << nowMillis << " + "
                                          << HEATER_OFF_DURATION_MILLIS << " = "
                                          << this->_dataReadyAtMillis << "ms");
    }

    [[nodiscard]]
    bool SGP40Sensor::doReadHeaterOff() {

        D(1, TAG, "heater should be off by now");
        this->_dataReadyAtMillis = 0;
        return true;
    }


    [[nodiscard]]
    bool SGP40Sensor::selfTest(const uint64_t nowMillis) {

        assert(nowMillis > 0);

        if (this->_dataReadyAtMillis == 0) {

            this->doRequestSelfTest(nowMillis);
            return false;

        }
        else if (this->_dataReadyAtMillis > nowMillis) {

            D(3, TAG, "sensor is running self test, to be ready at: "
                << this->_dataReadyAtMillis << "ms (" <<
                (this->_dataReadyAtMillis - nowMillis) << "ms left)");
            return false;

        }
        else {

            return this->doReadSelfTest();

        }
    }

    void SGP40Sensor::doRequestSelfTest(const uint64_t nowMillis) {

        D(1, TAG, "requesting self test");

        this->_selfTestOk = false;

        this->_i2c[0] = SELF_TEST_CMD_HI;
        this->_i2c[1] = SELF_TEST_CMD_LO;
        const auto result = this->_i2c.write(2);
        if (Homer2I2cError::no_error != result) {
            this->_dataReadyAtMillis = 0;
            E(TAG, "failed to request self test: " << result);
            throw std::runtime_error{"SGP40: failed to request self test"};
        }

        this->_dataReadyAtMillis = nowMillis + SELF_TEST_DURATION_MILLIS;

        D(3, TAG, "self test to be ready at: " << nowMillis << " + "
                                               << SELF_TEST_DURATION_MILLIS << " = "
                                               << this->_dataReadyAtMillis << "ms");
    }

    [[nodiscard]]
    bool SGP40Sensor::doReadSelfTest() {

        D(3, TAG, "self test ready, reading");
        this->_dataReadyAtMillis = 0;

        const auto result = this->read(1);
        if (Homer2I2cError::no_error != result) {
            E(TAG, "failed to read self test: " << result);
            throw std::runtime_error{"SGP40: failed to read self test"};
        }

        const auto readBack = this->convertToUInt16(0);
        this->_selfTestOk = readBack == SELF_TEST_REPLY;

        return true;
    }


    [[nodiscard]]
    bool SGP40Sensor::readSerialNumber(const uint64_t nowMillis) {

        assert(nowMillis > 0);

        if (this->_dataReadyAtMillis == 0) {

            this->doRequestSerialNumber(nowMillis);
            return false;

        }
        else if (this->_dataReadyAtMillis > nowMillis) {

            D(3, TAG, "serial number is not ready yet, to be ready at: "
                << this->_dataReadyAtMillis << "ms (" <<
                (this->_dataReadyAtMillis - nowMillis) << "ms left)");
            return false;

        }
        else {

            return this->doReadSerialNumber();

        }
    }

    void SGP40Sensor::doRequestSerialNumber(const uint64_t nowMillis) {

        D(1, TAG, "requesting serial number");

        this->_serialNumber[0] = 0;
        this->_serialNumber[1] = 0;
        this->_serialNumber[2] = 0;

        this->_i2c[0] = SERIAL_NUMBER_CMD_HI;
        this->_i2c[1] = SERIAL_NUMBER_CMD_LO;
        const auto result = this->_i2c.write(2);
        if (Homer2I2cError::no_error != result) {
            this->_dataReadyAtMillis = 0;
            E(TAG, "failed to request serial number: " << result);
            throw std::runtime_error{"SGP40: failed to request serial number"};
        }

        this->_dataReadyAtMillis = nowMillis + SERIAL_NUMBER_DURATION_MILLIS;

        D(3, TAG, "serial number to be ready at: " << nowMillis << " + "
                                                   << SERIAL_NUMBER_DURATION_MILLIS << " = "
                                                   << this->_dataReadyAtMillis << "ms");
    }

    [[nodiscard]]
    bool SGP40Sensor::doReadSerialNumber() {

        D(3, TAG, "serial number ready, reading");
        this->_dataReadyAtMillis = 0;

        const auto result = this->read(3);
        if (Homer2I2cError::no_error != result) {
            E(TAG, "failed to read serial number: " << result);
            throw std::runtime_error{"SGP40: failed to read serial number"};
        }

        const auto v0 = this->convertToUInt16(0);
        const auto v1 = this->convertToUInt16(1);
        const auto v2 = this->convertToUInt16(2);

        this->_serialNumber[0] = static_cast<uint8_t>(v0 >> 8);
        this->_serialNumber[1] = static_cast<uint8_t>(v0 & 0xFF);
        this->_serialNumber[2] = static_cast<uint8_t>(v1 >> 8);
        this->_serialNumber[3] = static_cast<uint8_t>(v1 & 0xFF);
        this->_serialNumber[4] = static_cast<uint8_t>(v2 >> 8);
        this->_serialNumber[5] = static_cast<uint8_t>(v2 & 0xFF);

        D(5, TAG, "serial number: 0x"
            << std::hex
            << std::uppercase
            << static_cast<uint64_t>(this->_serialNumber[0])
            << '-'
            << std::hex
            << std::uppercase
            << static_cast<uint64_t>(this->_serialNumber[1])
            << '-'
            << std::hex
            << std::uppercase
            << static_cast<uint64_t>(this->_serialNumber[2])
            << '-'
            << std::hex
            << std::uppercase
            << static_cast<uint64_t>(this->_serialNumber[3])
            << '-'
            << std::hex
            << std::uppercase
            << static_cast<uint64_t>(this->_serialNumber[4])
            << '-'
            << std::hex
            << std::uppercase
            << static_cast<uint64_t>(this->_serialNumber[5]));

        return true;
    }


    [[nodiscard]]
    bool SGP40Sensor::readFeatureSet(const uint64_t nowMillis) {

        assert(nowMillis > 0);

        if (this->_dataReadyAtMillis == 0) {

            this->doRequestFeatureSet(nowMillis);
            return false;

        }
        else if (this->_dataReadyAtMillis > nowMillis) {

            D(3, TAG, "feature set is not ready yet, to be ready at: "
                << this->_dataReadyAtMillis << "ms (" <<
                (this->_dataReadyAtMillis - nowMillis) << "ms left)");
            return false;

        }
        else {

            return this->doReadFeatureSet();

        }
    }

    void SGP40Sensor::doRequestFeatureSet(const uint64_t nowMillis) {

        D(1, TAG, "requesting feature set");

        this->_featureSet = 0;

        this->_i2c[0] = FEATURE_SET_CMD_HI;
        this->_i2c[1] = FEATURE_SET_CMD_LO;
        const auto result = this->_i2c.write(2);
        if (Homer2I2cError::no_error != result) {
            this->_dataReadyAtMillis = 0;
            E(TAG, "failed to request feature set: " << result);
            throw std::runtime_error{"SGP40: failed to request feature set"};
        }

        this->_dataReadyAtMillis = nowMillis + FEATURE_SET_DURATION_MILLIS;

        D(3, TAG, "feature set to be ready at: " << nowMillis << " + "
                                                 << FEATURE_SET_DURATION_MILLIS << " = "
                                                 << this->_dataReadyAtMillis << "ms");
    }

    [[nodiscard]]
    bool SGP40Sensor::doReadFeatureSet() {

        D(3, TAG, "feature set ready, reading");
        this->_dataReadyAtMillis = 0;

        const auto result = this->read(1);
        if (Homer2I2cError::no_error != result) {
            this->_dataReadyAtMillis = 0;
            E(TAG, "failed to read feature set: " << result);
            throw std::runtime_error{"SGP40: failed to read feature set"};
        }

        this->_featureSet = this->convertToUInt16(0);

        D(5, TAG, "feature set: " << std::hex << this->_featureSet);
        return true;
    }


    [[nodiscard]]
    uint16_t SGP40Sensor::convertToUInt16(const size_t index) {

        const auto hi = static_cast<uint16_t>(this->_i2c[index * 3]);
        const auto lo = static_cast<uint16_t>(this->_i2c[index * 3 + 1]);

        auto value = static_cast<uint16_t>(static_cast<uint16_t>(hi << 8) & U_INT_16_HI_BITS);
        value |= static_cast<uint16_t>(lo & U_INT_16_LO_BITS);

        return value;
    }

    i2c::Homer2I2cError SGP40Sensor::read(const size_t len) noexcept {

        const auto result = this->_i2c.read(len * 3);
        if (Homer2I2cError::no_error != result)
            return result;

        if (!check_crc(this->_i2c, len))
            return Homer2I2cError::read_corrupt_data;

        return Homer2I2cError::no_error;
    }

}
