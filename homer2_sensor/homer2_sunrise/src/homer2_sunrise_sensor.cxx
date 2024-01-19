#include <map>
#include <utility>

#include <homer2_util.hpp>
#include <homer2_logging.hpp>
#include <homer2_i2c.hpp>

#include "homer2_sunrise_base.hpp"
#include "homer2_sunrise_sensor.hpp"

using homer2::i2c::Homer2I2c;
using homer2::i2c::Homer2I2cError;

namespace homer2::sensor::sunrise::internal::sensor {

    namespace {

        constexpr uint64_t I2C_TIMEOUT_MILLIS = 35;
        constexpr uint8_t I2C_ADDR = 0x68;

        constexpr uint64_t EEPROM_WRITE_DURATION_MILLIS = 25;
        constexpr uint64_t STABILIZATION_DURATION_MILLIS = 35;

        constexpr uint8_t ERROR_STATUS_REG = 0x01;
        constexpr uint8_t MEASUREMENT_MODE_REG = 0x95;
        constexpr uint8_t METER_CONTROL_REG = 0xA5;

        constexpr uint16_t MODE_CONTINUOUS = 0x0000;
        // constexpr uint16_t MODE_SINGLE = 0x0001;

        constexpr size_t WAKEUP_ATTEMPTS = 5;

    }

    SunriseSensor::SunriseSensor(
        std::shared_ptr<Homer2I2c> i2c
    ) :
        _i2c{
            i2c::I2cConnection{
                std::move(i2c),
                I2C_ADDR,
                I2C_TIMEOUT_MILLIS,
            }
        } {

        I(TAG, "i2c addr: 0x" << std::hex << std::uppercase << static_cast<uint64_t>(I2C_ADDR));

        sleep_ms(STABILIZATION_DURATION_MILLIS);
    }

    // =================================

    void SunriseSensor::measure() {

        this->wakeup();

        this->_i2c[0] = ERROR_STATUS_REG;
        auto result = this->_i2c.writeNonStop(1);
        if (result != Homer2I2cError::no_error) {
            E(TAG, "failed to request measurement: " << result);
            throw std::runtime_error{"Sunrise: failed to request measurement"};
        }
        result = this->_i2c.read(7);
        if (result != Homer2I2cError::no_error) {
            E(TAG, "failed to read measurement: " << result);
            throw std::runtime_error{"Sunrise: failed to read measurement"};
        }

        this->_errorStatus = this->_i2c[0];
        this->_co2Ppm = merge(this->_i2c[5], this->_i2c[6]);
    }

    void SunriseSensor::doRequestMeasurement(
        const uint64_t nowMillis
    ) {
        D(1, TAG, "requesting measurement");
        this->_dataReadyAtMillis = 0;

        this->wakeup();

        this->_dataReadyAtMillis = nowMillis + 2000;

        D(3, TAG, "measurement to be ready at: " << nowMillis << " + "
                                                 << 2000 << " = "
                                                 << this->_dataReadyAtMillis << "ms");
    }

    [[nodiscard]]
    bool SunriseSensor::doReadMeasurement() {

        D(3, TAG, "measurement ready, reading");
        this->_dataReadyAtMillis = 0;

        D(5, TAG, "co2 concentration (ppm): " << this->_co2Ppm);
        D(5, TAG, "error: 0x" << std::hex << std::uppercase << this->_errorStatus);
        return true;
    }

    // =================================

    [[maybe_unused]]
    void SunriseSensor::readSensorConfig() {

        this->wakeup();

        this->_i2c[0] = MEASUREMENT_MODE_REG;
        auto result = this->_i2c.writeNonStop(1);
        if (result != Homer2I2cError::no_error) {
            E(TAG, "failed to request measurement mode: " << result);
            throw std::runtime_error{"Sunrise: failed to request measurement mode"};
        }
        result = this->_i2c.read(7);
        if (result != Homer2I2cError::no_error) {
            E(TAG, "failed to read measurement mode: " << result);
            throw std::runtime_error{"Sunrise: failed to read measurement mode"};
        }

        this->_measurementMode = this->_i2c[0];
        this->_measurementPeriodMillis = merge(this->_i2c[1], this->_i2c[2]);
        this->_numberOfSamples = merge(this->_i2c[3], this->_i2c[4]);
        this->_abcPeriodHours = merge(this->_i2c[5], this->_i2c[6]);

        // -----------------------------

        this->wakeup();

        this->_i2c[0] = METER_CONTROL_REG;
        result = this->_i2c.writeNonStop(1);
        if (result != Homer2I2cError::no_error) {
            E(TAG, "failed to request meter control: " << result);
            throw std::runtime_error{"Sunrise: failed to request meter control"};
        }
        result = this->_i2c.read(1);
        if (result != Homer2I2cError::no_error) {
            E(TAG, "failed to read meter control: " << result);
            throw std::runtime_error{"Sunrise: failed to read meter control"};
        }

        this->_meterControl = this->_i2c[0];

        // -----------------------------

        I(TAG, "measurement mode: " << std::to_string(this->_measurementMode));
        I(TAG, "measurement period: " << std::to_string(this->_measurementPeriodMillis) << "ms");
        I(TAG, "number of samples: " << std::to_string(this->_numberOfSamples));
        I(TAG, "meter control: " << std::hex << std::uppercase
                                 << static_cast<uint64_t>(this->_meterControl));
        if ((0U == this->_abcPeriodHours) ||
            (0xFFFFU == this->_abcPeriodHours) ||
            (this->_meterControl & 0x02U)
            )
            I(TAG, "ABC period: disabled");
        else
            I(TAG, "ABC period: " << std::to_string(this->_abcPeriodHours) << " hours");
    }

    [[maybe_unused]]
    void SunriseSensor::disableABC() {
        this->setABC(false);
    }

    [[maybe_unused]]
    void SunriseSensor::enableABC() {
        this->setABC(true);
    }

    void SunriseSensor::setABC(const bool enabled) {

        this->wakeup();

        this->_i2c[0] = METER_CONTROL_REG;
        auto result = this->_i2c.writeNonStop(1);
        if (result != Homer2I2cError::no_error) {
            E(TAG, "failed to request meter control: " << result);
            throw std::runtime_error{"Sunrise: failed to request meter control"};
        }
        result = this->_i2c.read(1);
        if (result != Homer2I2cError::no_error) {
            E(TAG, "failed to read meter control: " << result);
            throw std::runtime_error{"Sunrise: failed to read meter control"};
        }

        const uint8_t current_mode = this->_i2c[0];

        const uint8_t new_mode =
            enabled
            ? (current_mode & static_cast<uint8_t>(~0x02U))
            : (current_mode | static_cast<uint8_t>(0x02U));

        if (enabled)
            I(TAG, "enabling ABC...");
        else
            I(TAG, "disabling ABC...");

        if (new_mode == current_mode) {
            I(TAG, "current ABC mode matches, bailing out: "
                << std::hex << std::uppercase << static_cast<uint64_t>(current_mode));
            this->_meterControl = current_mode;
            return;
        }
        else {
            I(TAG, "current ABC mode does not matches, writing new mode: CURR#"
                << std::uppercase << std::hex << static_cast<uint64_t>(current_mode)
                << " != NEW#"
                << std::uppercase << std::hex << static_cast<uint64_t>(new_mode));
        }

        this->wakeup();

        this->_i2c[0] = METER_CONTROL_REG;
        this->_i2c[1] = new_mode;
        result = this->_i2c.write(2);

        sleep_ms(EEPROM_WRITE_DURATION_MILLIS);

        if (result != Homer2I2cError::no_error) {
            E(TAG, "failed to set meter control: " << result);
            throw std::runtime_error{"Sunrise: failed to set meter control"};
        }

        this->_i2c[0] = METER_CONTROL_REG;
        result = this->_i2c.writeNonStop(1);
        if (result != Homer2I2cError::no_error) {
            E(TAG, "failed to request meter control: " << result);
            throw std::runtime_error{"Sunrise: failed to request meter control"};
        }
        result = this->_i2c.read(1);
        if (result != Homer2I2cError::no_error) {
            E(TAG, "failed to read meter control: " << result);
            throw std::runtime_error{"Sunrise: failed to read meter control"};
        }

        this->_meterControl = this->_i2c[0];

        I(TAG, "final mode: "
            << std::hex << std::uppercase << static_cast<uint64_t>(this->_meterControl));
    }

    void SunriseSensor::setToContinuousMode() {

        this->wakeup();

        this->_i2c[0] = MEASUREMENT_MODE_REG;
        auto result = this->_i2c.writeNonStop(1);
        if (result != Homer2I2cError::no_error) {
            E(TAG, "failed to request measurement mode: " << result);
            throw std::runtime_error{"Sunrise: failed to request measurement mode"};
        }
        result = this->_i2c.read(1);
        if (result != Homer2I2cError::no_error) {
            E(TAG, "failed to read measurement mode: " << result);
            throw std::runtime_error{"Sunrise: failed to read measurement mode"};
        }

        this->_measurementMode = this->_i2c[0];

        if (this->_measurementMode != MODE_CONTINUOUS) {

            I(TAG, "measurement mode is not continuous, changing...");

            this->wakeup();

            this->_i2c[0] = MEASUREMENT_MODE_REG;
            this->_i2c[1] = MODE_CONTINUOUS;
            result = this->_i2c.write(2);

            sleep_ms(EEPROM_WRITE_DURATION_MILLIS);

            if (result != Homer2I2cError::no_error) {
                E(TAG, "failed to set measurement mode: " << result);
                throw std::runtime_error{"Sunrise: failed to set measurement mode"};
            }

            W(TAG, "sensor restart is required to apply the changes, "
                << "you need to power cycle the system");
            this->haltSystem();
        }
    }

    // =================================

    void SunriseSensor::wakeup() {

        size_t attempt = 0;
        Homer2I2cError err;
        do {

            err = this->_i2c.write(0);
            attempt++;

            if (Homer2I2cError::no_error != err && attempt < WAKEUP_ATTEMPTS)
                D(4, TAG, "sensor did not ACK the wakeup attempt, will retry, "
                    << attempt << '/' << WAKEUP_ATTEMPTS);
            else
                break;

        } while (true);

        if (Homer2I2cError::no_error != err) {
            E(TAG, "failed to wake the sensor up, attempts: " << attempt << ", error: " << err);
            throw std::runtime_error{"Sunrise: failed to wake the sensor up"};
        }
    }

    [[noreturn]]
    void SunriseSensor::haltSystem() const noexcept {

        W(TAG, "halting the system! power cycle needed for correct function");
        while (true)
            sleep_ms(10'000);
    }

    // =================================

    [[nodiscard]]
    [[maybe_unused]]
    uint16_t SunriseSensor::getCo2Ppm() const noexcept {

        return this->_co2Ppm;
    }

    [[nodiscard]]
    [[maybe_unused]]
    uint8_t SunriseSensor::getErrorStatus() const noexcept {

        return this->_errorStatus;
    }

}
