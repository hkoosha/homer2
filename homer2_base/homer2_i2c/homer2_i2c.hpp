#pragma once

#include <cstdint>
#include <array>
#include <memory>

#include <hardware/i2c.h>

namespace homer2::i2c {

    constexpr size_t BUFFER_CAPACITY = 64;

    enum class Homer2I2cError : int8_t {
        no_error = 0,
        buffer_too_small = -1,
        read_timeout = -2,
        read_generic_error = -3,
        read_unknown_error = -4,
        read_missing_data = -5,
        read_too_much_data = -6,
        read_corrupt_data = -7,
        write_timeout = -8,
        write_generic_error = -9,
        write_unknown_error = -10,
        write_missing_data = -11,
        write_too_much_data = -12,
    };

    std::ostream& operator<<(
        std::ostream& out,
        Homer2I2cError value
    );


    class Homer2I2c {
    public:

        Homer2I2c& operator=(const Homer2I2c& other) noexcept = delete;

        Homer2I2c& operator=(Homer2I2c&& other) = delete;

        Homer2I2c(Homer2I2c&& other) = delete;

        Homer2I2c() = delete;

        Homer2I2c(const Homer2I2c& other) noexcept = delete;


        explicit Homer2I2c(i2c_inst_t* i2c);

        [[nodiscard]]
        Homer2I2cError read(
            uint64_t timeoutMillis,
            uint8_t addr,
            uint8_t len
        ) noexcept;

        [[nodiscard]]
        Homer2I2cError write(
            uint64_t timeoutMillis,
            uint8_t addr,
            uint8_t len
        ) const noexcept;

        [[nodiscard]]
        Homer2I2cError writeNonStop(
            uint64_t timeoutMillis,
            uint8_t addr,
            uint8_t len
        ) const noexcept;

        [[nodiscard]]
        uint8_t operator[](size_t index) const;

        uint8_t& operator[](size_t index);

        [[nodiscard]]
        size_t getBufferCapacity() const;

    private:

        [[nodiscard]]
        Homer2I2cError doWrite(
            uint64_t timeoutMillis,
            uint8_t addr,
            uint8_t len,
            bool nonStop
        ) const noexcept;

        i2c_inst_t* const _i2c;
        std::array<uint8_t, BUFFER_CAPACITY> _buffer;

    };

    class I2cConnection {
    public:

        I2cConnection(
            std::shared_ptr<i2c::Homer2I2c> _i2c,
            uint8_t addr,
            uint64_t timeoutMillis
        );

        [[nodiscard]]
        Homer2I2cError read(uint8_t len) noexcept;

        [[nodiscard]]
        Homer2I2cError write(uint8_t len) const noexcept;

        [[nodiscard]]
        Homer2I2cError writeNonStop(uint8_t len) const noexcept;

        [[nodiscard]]
        uint8_t operator[](size_t index) const;

        uint8_t& operator[](size_t index);

        [[nodiscard]]
        size_t getBufferCapacity() const;

    private:

        const std::shared_ptr<i2c::Homer2I2c> _i2c;
        const uint8_t _addr;
        const uint64_t _timeoutMillis;

    };


    namespace Helper {

        void delay_us(
            uint32_t delay,
            void* any
        ) noexcept;

        [[nodiscard]]
        int8_t read_helper(
            uint8_t reg,
            uint8_t* into,
            uint32_t len,
            void* connection
        ) noexcept;

        [[nodiscard]]
        int8_t write_helper(
            uint8_t reg,
            const uint8_t* data,
            uint32_t len,
            void* connection
        ) noexcept;

        [[nodiscard]]
        int8_t write_and_read_helper(
            uint8_t reg,
            uint8_t* into,
            uint32_t len,
            void* connection
        ) noexcept;

    }

}
