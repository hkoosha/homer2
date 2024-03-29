#pragma once

#include <string>
#include <array>

#include <lwip/tcp.h>
#include <lwip/ip_addr.h>

#include "homer2_sensor.hpp"

namespace homer2 {

    namespace internal {

        enum class ConnectionStatus {
            DISCONNECTED,
            CONNECTING,
            CONNECTED,
        };

    }

    class Homer2Pusher {
    public:

        Homer2Pusher(
            const char* addr,
            uint16_t port,
            uint64_t pushFrequencyMillis,
            uint64_t pushStartAfterTimestamp
        );

        void push(
            const Homer2SensorsData& data
        );


    private:

        void fillBuffer(
            const Homer2SensorsData& data
        ) noexcept;

        void tryPush() noexcept;

        [[nodiscard]]
        bool resolve() noexcept;

        void resolve_ip() noexcept;

        void resolve_addr() noexcept;

        [[nodiscard]]
        bool open() noexcept;

        [[nodiscard]]
        bool connect() noexcept;

        [[nodiscard]]
        bool write() noexcept;

        err_t close() noexcept;


        void waitForConnection() const noexcept;

        void incTcpErr() noexcept;

        void incDnsErr() noexcept;

        void resetTcpErr() noexcept;

        void resetDnsErr() noexcept;

        [[nodiscard]]
        std::string ipAddr() const noexcept;

        const uint64_t _pushStartAfterTimestamp;
        const uint64_t _pushFrequencyMillis;
        uint64_t _lastPushMillis{0};
        // uint8_t _httpErrors{0};
        uint8_t _tcpErrors{0};
        uint8_t _dnsErrors{0};

        const char* _addr;
        const uint16_t _port;
        ip_addr_t _ip{.addr = IPADDR_ANY};

        struct tcp_pcb* _tcpPcb{nullptr};
        internal::ConnectionStatus _connection{internal::ConnectionStatus::DISCONNECTED};

        const std::string _header;
        std::string _body{};
        std::string _writeBuffer{};

    };

}
