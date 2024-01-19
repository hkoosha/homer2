#include <hardware/timer.h>

#include <homer2_logging.hpp>

#include "homer2_pmsx00x_base.hpp"
#include "homer2_pmsx00x_sensor.hpp"

namespace homer2::sensor::pmsx00x::internal::sensor {

    namespace {

        constexpr uint8_t START0 = 0x42;
        constexpr uint8_t START1 = 0x4d;

        constexpr uint64_t STABILIZATION_DURATION_MILLIS = 30'000;
    }

    PMSx00xSensor::PMSx00xSensor(uart_inst_t* const uart) :
        _uart{uart} {

        if (nullptr == uart)
            throw std::runtime_error{"PMSx00x: uart is not set"};
    }

    // =================================

    [[nodiscard]]
    bool PMSx00xSensor::measure(
        const uint64_t nowMillis
    ) {
        assert(nowMillis > 0);

        if (this->_stabilizedAt == 0)
            this->_stabilizedAt = nowMillis + STABILIZATION_DURATION_MILLIS;

        if (nowMillis < this->_stabilizedAt) {
            D(1, TAG, "data not stabilized at, to be ready at: "
                << this->_stabilizedAt << "ms ("
                << (this->_stabilizedAt - nowMillis)
                << "ms left)");
            return false;
        }

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

            return this->doReadMeasurement(nowMillis);

        }
    }

    void PMSx00xSensor::doRequestMeasurement(
        const uint64_t nowMillis
    ) {
        D(1, TAG, "requesting measurement");
        this->_dataReadyAtMillis = nowMillis + 1;

        D(3, TAG, "measurement to be ready at: " << nowMillis << " + "
                                                 << 1 << " = "
                                                 << this->_dataReadyAtMillis << "ms");
    }

    [[nodiscard]]
    bool PMSx00xSensor::doReadMeasurement(
        const uint64_t nowMillis
    ) {
        if (!uart_is_readable(this->_uart))
            D(5, TAG, "no data on uart, index=" << std::to_string(this->_bufIndex));

        while ((uart_is_readable(this->_uart)) && ((this->_bufIndex) < 32)) {
            D(5, TAG, "data available, reading, index=" << std::to_string(this->_bufIndex));
            uart_read_blocking(this->_uart, &this->_buffer[this->_bufIndex], 1);

            switch (this->_bufIndex) {
                case 0: {
                    if (this->_buffer[0] != START0) {
                        D(2, TAG, "bad data at index=0, discarding: "
                            << std::hex << static_cast<uint64_t>(this->_buffer[0]));
                        this->_bufIndex = 0;
                    }
                    else {
                        D(5, TAG, "index=0 read correctly");
                        this->_bufIndex = 1;
                    }
                }
                    break;

                case 1: {
                    if (this->_buffer[1] != START1) {
                        D(2, TAG, "bad data at index=1, discarding: "
                            << std::hex << static_cast<uint64_t>(this->_buffer[1]));
                        this->_bufIndex = 0;
                    }
                    else {
                        D(5, TAG, "index=1 read correctly");
                        this->_bufIndex = 2;
                    }
                }
                    break;

                default: {
                    assert(this->_bufIndex < 32);
                    D(5, TAG, "data read, "
                        << std::to_string(this->_bufIndex)
                        << " = 0x"
                        << std::hex
                        << static_cast<uint64_t>(this->_buffer[this->_bufIndex]));
                    this->_bufIndex++;
                }
                    break;
            }
        }

        if (32 == this->_bufIndex) {
            uint16_t sum = 0;
            for (uint8_t i = 0; i < 30; i++)
                sum += this->_buffer[i];

            const auto checksum = static_cast<uint16_t>(this->_buffer[30] * 256) + this->_buffer[31];

            if (sum != checksum) {
                D(2, TAG, "bad checksum, discarding buffers: "
                    << std::hex
                    << static_cast<uint64_t>(sum)
                    << " != "
                    << std::hex
                    << static_cast<uint64_t>(checksum));

                while (uart_is_readable(this->_uart))
                    uart_read_blocking(this->_uart, &this->_buffer[0], 1);

                this->_bufIndex = 0;
            }
        }

        if (32 == this->_bufIndex) {
            D(3, TAG, "data fully read, extracting measurements");
            this->_bufIndex = 0;
            this->_dataReadyAtMillis = 0;

            this->_pm10Std = static_cast<uint16_t>(this->_buffer[4] * 256) + this->_buffer[5];
            this->_pm25Std = static_cast<uint16_t>(this->_buffer[6] * 256) + this->_buffer[7];
            this->_pm100Std = static_cast<uint16_t>(this->_buffer[8] * 256) + this->_buffer[9];
            this->_pm10Env = static_cast<uint16_t>(this->_buffer[10] * 256) + this->_buffer[11];
            this->_pm25Env = static_cast<uint16_t>(this->_buffer[12] * 256) + this->_buffer[13];
            this->_pm100Env = static_cast<uint16_t>(this->_buffer[14] * 256) + this->_buffer[15];
            this->_particles03 = static_cast<uint16_t>(this->_buffer[16] * 256) + this->_buffer[17];
            this->_particles05 = static_cast<uint16_t>(this->_buffer[18] * 256) + this->_buffer[19];
            this->_particles10 = static_cast<uint16_t>(this->_buffer[20] * 256) + this->_buffer[21];
            this->_particles25 = static_cast<uint16_t>(this->_buffer[22] * 256) + this->_buffer[23];
            this->_particles50 = static_cast<uint16_t>(this->_buffer[24] * 256) + this->_buffer[25];
            this->_particles100 = static_cast<uint16_t>(this->_buffer[26] * 256) + this->_buffer[27];

            D(5, TAG, "discarding buffers after successful read");
            while (uart_is_readable(this->_uart))
                uart_read_blocking(this->_uart, &this->_buffer[0], 1);

            D(5, TAG, "pm-10 standard  (ppm): " << std::to_string(this->_pm10Std));
            D(5, TAG, "pm-25 standard  (ppm): " << std::to_string(this->_pm25Std));
            D(5, TAG, "pm-100 standard (ppm): " << std::to_string(this->_pm100Std));
            D(5, TAG, "pm-10 env       (ppm): " << std::to_string(this->_pm10Env));
            D(5, TAG, "pm-25 env       (ppm): " << std::to_string(this->_pm25Env));
            D(5, TAG, "pm-100 env      (ppm): " << std::to_string(this->_pm100Env));
            D(5, TAG, "particle-03     (ppm): " << std::to_string(this->_particles03));
            D(5, TAG, "particle-05     (ppm): " << std::to_string(this->_particles05));
            D(5, TAG, "particle-10     (ppm): " << std::to_string(this->_particles10));
            D(5, TAG, "particle-25     (ppm): " << std::to_string(this->_particles25));
            D(5, TAG, "particle-50     (ppm): " << std::to_string(this->_particles50));
            D(5, TAG, "particle-100    (ppm): " << std::to_string(this->_particles100));

            return true;

        }
        else {

            D(5, TAG, "data not fully read yet");
            this->_dataReadyAtMillis = nowMillis + 1;
            return false;

        }
    }

    // =================================

    [[maybe_unused]]
    [[nodiscard]]
    uint16_t PMSx00xSensor::getPm10Std() const noexcept {

        return this->_pm10Std;
    }

    [[maybe_unused]]
    [[nodiscard]]
    uint16_t PMSx00xSensor::getPm25Std() const noexcept {

        return this->_pm25Std;
    }

    [[maybe_unused]]
    [[nodiscard]]
    uint16_t PMSx00xSensor::getPm100Std() const noexcept {

        return this->_pm100Std;
    }

    [[maybe_unused]]
    [[nodiscard]]
    uint16_t PMSx00xSensor::getPm10Env() const noexcept {

        return this->_pm10Env;
    }

    [[maybe_unused]]
    [[nodiscard]]
    uint16_t PMSx00xSensor::getPm25Env() const noexcept {

        return this->_pm25Env;
    }

    [[maybe_unused]]
    [[nodiscard]]
    uint16_t PMSx00xSensor::getPm100Env() const noexcept {

        return this->_pm100Env;
    }

    [[maybe_unused]]
    [[nodiscard]]
    uint16_t PMSx00xSensor::getParticles03() const noexcept {

        return this->_particles03;
    }

    [[maybe_unused]]
    [[nodiscard]]
    uint16_t PMSx00xSensor::getParticles05() const noexcept {

        return this->_particles05;
    }

    [[maybe_unused]]
    [[nodiscard]]
    uint16_t PMSx00xSensor::getParticles10() const noexcept {

        return this->_particles10;
    }

    [[maybe_unused]]
    [[nodiscard]]
    uint16_t PMSx00xSensor::getParticles25() const noexcept {

        return this->_particles25;
    }

    [[maybe_unused]]
    [[nodiscard]]
    uint16_t PMSx00xSensor::getParticles50() const noexcept {

        return this->_particles50;
    }

    [[maybe_unused]]
    [[nodiscard]]
    uint16_t PMSx00xSensor::getParticles100() const noexcept {

        return this->_particles100;
    }

}
