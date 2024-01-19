#include <ostream>
#include <map>
#include <array>
#include <memory>
#include <utility>

#include <homer2_util.hpp>
#include <homer2_logging.hpp>

#include "homer2_i2c.hpp"

namespace homer2::i2c {

    namespace {

        const char* const TAG = "I2C";

    }

    namespace Helper {

        void delay_us(
            const uint32_t delay,
            void* intf_ptr
        ) noexcept {

            (void) intf_ptr;

            D(4, TAG, "delay_us: " << delay);
            sleep_us(delay);
        }

        [[nodiscard]]
        int8_t read_helper(
            const uint8_t reg,
            uint8_t* const into,
            const uint32_t len,
            void* connectionPtr
        ) noexcept {

            D(5, TAG, "ignoring reg: " << std::hex << static_cast<uint64_t>(reg));
            (void) reg;

            auto* const connection = static_cast<I2cConnection*>(connectionPtr);

            if (len >= connection->getBufferCapacity()) {
                E(TAG, "read data will not fit into i2c buffer: "
                    << connection->getBufferCapacity() << " < " << len);
                return static_cast<int8_t>(i2c::Homer2I2cError::buffer_too_small);
            }

            const auto err = connection->read(len);

            if (i2c::Homer2I2cError::no_error == err)
                for (uint8_t i = 0; i < len; ++i)
                    into[i] = (*connection)[i];

            return static_cast<int8_t>(err);
        }

        [[nodiscard]]
        int8_t write_helper(
            const uint8_t reg,
            const uint8_t* const data,
            const uint32_t len,
            void* connectionPtr
        ) noexcept {

            auto* const connection = static_cast<I2cConnection*>(connectionPtr);

            if ((len + 1) >= connection->getBufferCapacity()) {
                E(TAG, "write data will not fit into i2c buffer: "
                    << connection->getBufferCapacity()
                    << " < " << (len + 1));
                return static_cast<int8_t>(i2c::Homer2I2cError::buffer_too_small);
            }

            (*connection)[0] = reg;
            if (nullptr != data)
                for (uint32_t i = 0; i < len; ++i)
                    (*connection)[i + 1] = data[i];

            const auto err = connection->write(len + 1);
            return static_cast<int8_t>(err);
        }

        [[nodiscard]]
        int8_t write_and_read_helper(
            const uint8_t reg,
            uint8_t* const into,
            const uint32_t len,
            void* connectionPtr
        ) noexcept {

            const int8_t err = write_helper(
                reg,
                nullptr,
                0,
                connectionPtr
            );

            if (0 != err)
                return err;

            return read_helper(0, into, len, connectionPtr);
        }

    }

    std::ostream& operator<<(
        std::ostream& out,
        const Homer2I2cError value
    ) {
        static std::map<Homer2I2cError, std::string_view> strings{
            {Homer2I2cError::no_error,            "no_error"},
            {Homer2I2cError::read_timeout,        "read_timeout"},
            {Homer2I2cError::read_generic_error,  "read_generic_error"},
            {Homer2I2cError::read_unknown_error,  "read_unknown_error"},
            {Homer2I2cError::read_missing_data,   "read_missing_data"},
            {Homer2I2cError::read_too_much_data,  "read_too_much_data"},
            {Homer2I2cError::read_corrupt_data,   "read_corrupt_data"},
            {Homer2I2cError::write_timeout,       "write_timeout"},
            {Homer2I2cError::write_generic_error, "write_generic_error"},
            {Homer2I2cError::write_unknown_error, "write_unknown_error"},
            {Homer2I2cError::write_missing_data,  "write_missing_data"},
            {Homer2I2cError::write_too_much_data, "write_too_much_data"},
            {Homer2I2cError::buffer_too_small,    "buffer_too_small"},
        };

        return out << strings[value];
    }

}

namespace homer2::i2c {

    I2cConnection::I2cConnection(
        std::shared_ptr<i2c::Homer2I2c> i2c,
        const uint8_t addr,
        const uint64_t timeoutMillis
    ) :
        _i2c{std::move(i2c)},
        _addr{addr},
        _timeoutMillis{timeoutMillis} {
    }

    [[nodiscard]]
    Homer2I2cError I2cConnection::read(const uint8_t len) noexcept {

        return this->_i2c->read(this->_timeoutMillis, this->_addr, len);
    }

    [[nodiscard]]
    Homer2I2cError I2cConnection::write(const uint8_t len) const noexcept {

        return this->_i2c->write(this->_timeoutMillis, this->_addr, len);
    }

    [[nodiscard]]
    Homer2I2cError I2cConnection::writeNonStop(const uint8_t len) const noexcept {

        return this->_i2c->writeNonStop(this->_timeoutMillis, this->_addr, len);
    }

    [[nodiscard]]
    uint8_t I2cConnection::operator[](const size_t index) const {

        return (*this->_i2c)[index];
    }

    [[nodiscard]]
    uint8_t& I2cConnection::operator[](const size_t index) {

        return (*this->_i2c)[index];
    }

#pragma clang diagnostic push
#pragma ide diagnostic ignored "ConstantFunctionResult"

    [[nodiscard]]
    size_t I2cConnection::getBufferCapacity() const {

        return this->_i2c->getBufferCapacity();
    }

#pragma clang diagnostic pop

}

namespace homer2::i2c {

    Homer2I2c::Homer2I2c(
        i2c_inst_t* const i2c
    ) :
        _i2c{i2c},
        _buffer{std::array<uint8_t, BUFFER_CAPACITY>{0}} {

        if (nullptr == i2c)
            throw std::logic_error{"Homer2I2c: i2c not set"};
    }


    [[nodiscard]]
    Homer2I2cError Homer2I2c::read(
        const uint64_t timeoutMillis,
        const uint8_t addr,
        const uint8_t len
    ) noexcept {

        D(5, TAG, std::hex << "i2c read"
                           << ", addr: 0x" << static_cast<uint64_t>(addr)
                           << ", len: " << len);

        auto err = Homer2I2cError::no_error;

        if (len <= 0) {
            W(TAG, "nothing to read");
            return err;
        }

        const auto read = i2c_read_blocking_until(
            this->_i2c,
            addr,
            this->_buffer.data(),
            len,
            false,
            a_until(timeoutMillis)
        );

        if (PICO_ERROR_TIMEOUT == read)
            err = Homer2I2cError::read_timeout;
        else if (PICO_ERROR_GENERIC == read)
            err = Homer2I2cError::read_generic_error;
        else if (read < 0)
            err = Homer2I2cError::read_unknown_error;
        else if (read != len)
            err = Homer2I2cError::read_missing_data;
        else if (read > 255)
            err = Homer2I2cError::read_too_much_data;
        if (Homer2I2cError::no_error != err) {
            E(TAG, "could not read from i2c: " << err);
            return err;
        }

#if DEBUG_ENABLED_AT_LEVEL(6)
        for (int i = 0; i < len; ++i)
                D(6, TAG, std::hex << "i2c read" << std::hex
                                   << ", addr: 0x" << static_cast<uint64_t>(addr)
                                   << ", i: " << std::dec << static_cast<uint64_t>(i)
                                   << ", d: " << std::hex << static_cast<uint64_t>(this->_buffer[i]));
#endif

        return err;
    }

    [[nodiscard]]
    Homer2I2cError Homer2I2c::write(
        const uint64_t timeoutMillis,
        const uint8_t addr,
        const uint8_t len
    ) const noexcept {

        return this->doWrite(timeoutMillis, addr, len, false);
    }

    [[nodiscard]]
    Homer2I2cError Homer2I2c::writeNonStop(
        const uint64_t timeoutMillis,
        const uint8_t addr,
        const uint8_t len
    ) const noexcept {

        return this->doWrite(timeoutMillis, addr, len, true);
    }

    [[nodiscard]]
    Homer2I2cError Homer2I2c::doWrite(
        const uint64_t timeoutMillis,
        const uint8_t addr,
        const uint8_t len,
        const bool nonStop
    ) const noexcept {

#if DEBUG_ENABLED_AT_LEVEL(6)
        for (int i = 0; i < len; ++i)
            D(6, TAG, std::hex << "i2c write" << std::hex
                               << ", addr: 0x" << static_cast<uint64_t>(addr)
                               << ", nonStop: " << (nonStop ? "yes" : "no")
                               << ", i: " << std::dec << static_cast<uint64_t>(i)
                               << ", d: " << std::hex << static_cast<uint64_t>(this->_buffer[i]));
#else
        D(5, TAG, std::hex << "i2c write"
                           << ", addr: 0x" << static_cast<uint64_t>(this->_i2cAddr)
                           << ", nonStop: " << (nonStop ? "yes" : "no")
                           << ", len: " << len);
#endif

        const int result = i2c_write_blocking_until(
            this->_i2c,
            addr,
            this->_buffer.data(),
            len,
            nonStop,
            a_until(timeoutMillis)
        );

        auto err = Homer2I2cError::no_error;
        if (PICO_ERROR_TIMEOUT == result) {
            err = Homer2I2cError::write_timeout;
            E(TAG, "could not write all data: write timeout, timeout set to: "
                << static_cast<uint64_t>(timeoutMillis) << "ms");
        }
        else if (PICO_ERROR_GENERIC == result) {
            err = Homer2I2cError::write_generic_error;
            E(TAG, "could not write to i2c: " << err);
        }
        else if (result < 0) {
            err = Homer2I2cError::write_unknown_error;
            E(TAG, "could not write to i2c: " << err);
        }
        else if (result < len) {
            err = Homer2I2cError::write_missing_data;
            E(TAG, "could not write all data: "
                << static_cast<uint64_t>(len) << " != " << static_cast<uint64_t>(result));
        }
        else if (result > len) {
            err = Homer2I2cError::write_too_much_data;
            E(TAG, "could not write data, too much was written: "
                << static_cast<uint64_t>(len) << " != " << static_cast<uint64_t>(result));
        }

        return err;
    }

    size_t Homer2I2c::getBufferCapacity() const {

        return BUFFER_CAPACITY;
    }

    uint8_t Homer2I2c::operator[](const size_t index) const {

        if (index >= BUFFER_CAPACITY)
            throw std::out_of_range{"read index out of range"};

        return this->_buffer[index];
    }

    uint8_t& Homer2I2c::operator[](const size_t index) {

        if (index >= BUFFER_CAPACITY)
            throw std::out_of_range{"write index out of range"};

        return this->_buffer[index];
    }

}

