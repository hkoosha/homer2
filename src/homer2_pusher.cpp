#include <pico/cyw43_arch.h>
#include <lwip/dns.h>

#include <homer2_logging.hpp>
#include <homer2_util.hpp>

#include "homer2_pusher.hpp"

namespace homer2 {

    namespace {

        const char* const TAG = "Pusher";

        const char* translate(
            const err_t err
        ) {
            switch (err) {
                case ERR_OK:
                    return "tcp - no error";

                case ERR_MEM:
                    return "tcp - out of memory";

                case ERR_BUF:
                    return "tcp - buffer error";

                case ERR_TIMEOUT:
                    return "tcp - timeout";

                case ERR_RTE:
                    return "tcp - routing problem";

                case ERR_INPROGRESS:
                    return "tcp - in progress";

                case ERR_VAL:
                    return "tcp - illegal value";

                case ERR_WOULDBLOCK:
                    return "tcp - operation would block";

                case ERR_USE:
                    return "tcp - address in use";

                case ERR_ALREADY:
                    return "tcp - already connecting";

                case ERR_ISCONN:
                    return "tcp - connection already established";

                case ERR_CONN:
                    return "tcp - not connected";

                case ERR_IF:
                    return "tcp - low-level netif error";

                case ERR_ABRT:
                    return "tcp - connection aborted";

                case ERR_RST:
                    return "tcp - connection reset";

                case ERR_CLSD:
                    return "tcp - connection closed";

                case ERR_ARG:
                    return "tcp - illegal argument";

                default:
                    return "tcp - unknown error";
            }
        }

        std::string header(
            const char* addr,
            const uint16_t port
        ) {

            std::string buffer{};
            buffer += "POST /api/put HTTP/1.1\nHost: ";
            buffer += addr;
            buffer += ':';
            buffer += std::to_string(port);
            buffer += "\nUser-Agent: homer2/";
            buffer += std::to_string(HOMER2_VERSION_MAJOR);
            buffer += '.';
            buffer += std::to_string(HOMER2_VERSION_MINOR);
            buffer +=
                "\nAccept: */*\nContent-Type: application/json\nContent-Length: ";

            return std::move(buffer);
        }

        bool addr_is_already_resolved(
            const char* addr
        ) {
            if (strlen(addr) == 0)
                return false;

            ip_addr_t ip{};
            return ip4addr_aton(addr, &ip) == 1;
        }

    }

}

namespace homer2 {

    Homer2Pusher::Homer2Pusher(
        const char* addr,
        const uint16_t port,
        uint64_t pushFrequencyMillis
    ) : _addr{addr},
        _port{port},
        _pushFrequencyMillis{pushFrequencyMillis},
        _header{header(addr, port)} {
    }

    void Homer2Pusher::push(
        const Homer2SensorsData& data
    ) {
        D(4, TAG, "pushing data...");

        if (this->_tcpErrors >= net::tcp_max_tries()) {
            E(TAG, "tcp max tries exhausted");
            this->close();
            throw std::runtime_error{"tcp max tries exhausted"};
        }

        if (this->_dnsErrors >= net::dns_max_tries()) {
            E(TAG, "dns max tries exhausted");
            this->close();
            throw std::runtime_error{"dns max tries exhausted"};
        }

        if (this->_connectionStartMillis > 0) {
            D(4, TAG, "connection in progress");

            // if (is_expired(this->_connectionStartMillis, net::tcp_write_timeout_millis())) {
            //     E(TAG, "tcp write timeout");
            //     this->incTcpErr();
            //     this->close();
            // }

            return;
        }

        if (!is_expired(this->_lastPushMillis, this->_pushFrequencyMillis)) {
            D(4, TAG, "next push time not arrived yet, ignoring cycle, remaining: "
                << remaining(this->_lastPushMillis, this->_pushFrequencyMillis)
                << "ms");
            return;
        }

        if (!this->resolve())
            return;

        this->fillBuffer(data);

        this->tryPush();
    }

    void Homer2Pusher::fillBuffer(
        const Homer2SensorsData& data
    ) noexcept {

        D(5, TAG, "filling data");

        this->_body = "[";

        if (data.sgp40Data().has_value()) {
            this->_body += R"({"metric":"voc_index","tags":{"agent":"homer2","sensor":"sgp40"},"value":)";
            this->_body += std::to_string(data.sgp40Data()->getVocIndex());
            this->_body += "},";
        }

        if (data.sunriseData().has_value()) {
            this->_body += R"({"metric":"co2","tags":{"agent":"homer2","sensor":"sunrise"},"value":)";
            this->_body += std::to_string(data.sunriseData()->getCo2Ppm());
            this->_body += "},";
        }

        if (data.bmp3xxData().has_value()) {
            this->_body += R"({"metric":"pressure","tags":{"agent":"homer2","sensor":"bmp3xx"},"value":)";
            this->_body += std::to_string(data.bmp3xxData()->getPressureHPa());
            this->_body += "},";
            this->_body += R"({"metric":"temperature","tags":{"agent":"homer2","sensor":"bmp3xx"},"value":)";
            this->_body += std::to_string(data.bmp3xxData()->getTemperatureCelsius());
            this->_body += "},";
        }

        if (data.bme68xData().has_value()) {
            this->_body += R"({"metric":"pressure","tags":{"agent":"homer2","sensor":"bme68x"},"value":)";
            this->_body += std::to_string(data.bme68xData()->getPressureHPa());
            this->_body += "},";
            this->_body += R"({"metric":"temperature","tags":{"agent":"homer2","sensor":"bme68x"},"value":)";
            this->_body += std::to_string(data.bme68xData()->getTemperatureCelsius());
            this->_body += "},";
            this->_body += R"({"metric":"humidity","tags":{"agent":"homer2","sensor":"bme68x"},"value":)";
            this->_body += std::to_string(data.bme68xData()->getRelativeHumidityPercent());
            this->_body += "},";
            this->_body += R"({"metric":"gas_resistance","tags":{"agent":"homer2","sensor":"bme68x"},"value":)";
            this->_body += std::to_string(data.bme68xData()->getGasResistanceOhms());
            this->_body += "},";
        }

        if (data.sht4xData().has_value()) {
            this->_body += R"({"metric":"temperature","tags":{"agent":"homer2","sensor":"sht4x"},"value":)";
            this->_body += std::to_string(data.sht4xData()->getTemperatureCelsius());
            this->_body += "},";
            this->_body += R"({"metric":"humidity","tags":{"agent":"homer2","sensor":"sht4x"},"value":)";
            this->_body += std::to_string(data.sht4xData()->getRelativeHumidityPercent());
            this->_body += "},";
        }

        if (data.pmsx00xData().has_value()) {
            this->_body += R"({"metric":"pm1_0","tags":{"agent":"homer2","sensor":"pmsx00x","cat":"env"},"value":)";
            this->_body += std::to_string(data.pmsx00xData()->getPm10Env());
            this->_body += "},";
            this->_body += R"({"metric":"pm2_5","tags":{"agent":"homer2","sensor":"pmsx00x","cat":"env"},"value":)";
            this->_body += std::to_string(data.pmsx00xData()->getPm25Env());
            this->_body += "},";
            this->_body += R"({"metric":"pm10_0","tags":{"agent":"homer2","sensor":"pmsx00x","cat":"env"},"value":)";
            this->_body += std::to_string(data.pmsx00xData()->getPm100Env());
            this->_body += "},";
            this->_body += R"({"metric":"ptc0_3","tags":{"agent":"homer2","sensor":"pmsx00x","cat":"env"},"value":)";
            this->_body += std::to_string(data.pmsx00xData()->getParticles03());
            this->_body += "},";
            this->_body += R"({"metric":"ptc0_5","tags":{"agent":"homer2","sensor":"pmsx00x","cat":"env"},"value":)";
            this->_body += std::to_string(data.pmsx00xData()->getParticles05());
            this->_body += "},";
            this->_body += R"({"metric":"ptc1_0","tags":{"agent":"homer2","sensor":"pmsx00x","cat":"env"},"value":)";
            this->_body += std::to_string(data.pmsx00xData()->getParticles10());
            this->_body += "},";
            this->_body += R"({"metric":"ptc2_5","tags":{"agent":"homer2","sensor":"pmsx00x","cat":"env"},"value":)";
            this->_body += std::to_string(data.pmsx00xData()->getParticles25());
            this->_body += "},";
            this->_body += R"({"metric":"ptc5_0","tags":{"agent":"homer2","sensor":"pmsx00x","cat":"env"},"value":)";
            this->_body += std::to_string(data.pmsx00xData()->getParticles50());
            this->_body += "},";
            this->_body += R"({"metric":"ptc10_0","tags":{"agent":"homer2","sensor":"pmsx00x","cat":"env"},"value":)";
            this->_body += std::to_string(data.pmsx00xData()->getParticles100());
            this->_body += "},";
        }

        if (this->_body.back() == ',')
            this->_body.pop_back();

        this->_body += ']';

        D(5, TAG, "data filled, len: " << this->_body.size());
    }

    void Homer2Pusher::tryPush() noexcept {

        if (!this->open())
            return;

        if (!this->connect())
            return;

        if (!this->write())
            return;
    }

    [[nodiscard]]
    bool Homer2Pusher::resolve() noexcept {

        if (this->_ip.addr != IPADDR_ANY && this->_ip.addr != IPADDR_NONE) {
            D(5, TAG, "previously resolved: " << this->ipAddr());
            return true;
        }

        I(TAG, "resolving: " << this->_addr);
        if (addr_is_already_resolved(this->_addr))
            this->resolve_ip();
        else
            this->resolve_addr();

        return false;
    }

    void Homer2Pusher::resolve_addr() noexcept {

        D(4, TAG, "address is domain, resolving: " << this->_addr);

        const auto start = now();

        cyw43_arch_lwip_begin();
        const err_t lwip_err = dns_gethostbyname(
            this->_addr,
            &this->_ip,
            [](
                const char* const name,
                const ip_addr_t* const ip_addr,
                void* const arg
            ) {
                D(4, TAG, "on resolved: " << name);
                auto that = static_cast<Homer2Pusher*>(arg);
                if (ip_addr) {
                    that->_ip.addr = ip_addr->addr;
                    that->resetDnsErr();
                }
                else {
                    E(TAG, "could not resolve");
                    that->_ip.addr = IPADDR_NONE;
                    that->incDnsErr();
                }
            },
            this
        );
        cyw43_arch_lwip_end();

        switch (lwip_err) {
            case ERR_OK: {
                I(TAG, "resolved in "
                    << std::to_string(now() - start) << "ms: "
                    << this->_addr << " => " << this->ipAddr());
                this->resetDnsErr();
            }
                break;

            case ERR_ARG: {
                E(TAG, "invalid domain: " << this->_addr);
                this->incDnsErr();
            }
                break;

            case ERR_INPROGRESS: {
                D(4, TAG, "dns resolver in progress");

                while (IPADDR_ANY == this->_ip.addr)
                    sleep_ms(1000);

                if (IPADDR_NONE == this->_ip.addr)
                    E(TAG, "failed to resolve address: " << this->_addr);
                else
                    I(TAG, "resolved in "
                        << std::to_string(now() - start) << "ms: "
                        << this->_addr << " => " << this->ipAddr());
            }
                break;

            default: {
                E(TAG, "unknown dns resolver error: " << std::to_string(lwip_err));
                this->incDnsErr();
            }
                break;
        }
    }

    void Homer2Pusher::resolve_ip() noexcept {

        D(4, TAG, "address is ip, parsing: " << this->_addr);

        ip_addr_t already{};
        auto ok = ipaddr_aton(this->_addr, &already);

        if (ok == 0) {
            E(TAG, "bad ip: " << this->_addr);
            return;
        }

        this->_ip.addr = already.addr;
    }

    [[nodiscard]]
    bool Homer2Pusher::open() noexcept {

        D(5, TAG, "opening tcp");

        this->_tcpPcb = tcp_new_ip_type(IP_GET_TYPE(&this->_ip));
        if (!this->_tcpPcb) {
            E(TAG, "failed to create tcp pcb");
            this->incTcpErr();
            this->close();
            return false;
        }

        tcp_arg(this->_tcpPcb, this);

        tcp_poll(
            this->_tcpPcb,
            [](
                void* const arg,
                struct tcp_pcb* const tcpPcb
            ) {
                E(TAG, "connection timeout");

                auto that = static_cast<Homer2Pusher*>(arg);
                that->incTcpErr();
                return that->close();
            },
            10
        );

        tcp_sent(
            this->_tcpPcb,
            [](
                void* const arg,
                struct tcp_pcb* const tcpPcb,
                const u16_t len
            ) {
                I(TAG, "tcp sent, wrote to sever: " << std::to_string(len));
                auto that = static_cast<Homer2Pusher*>(arg);


                if (len == that->_writeBuffer.size()) {
                    that->resetTcpErr();
                    return that->close();
                }

                return static_cast<err_t>(ERR_OK);
            }
        );

        tcp_recv(
            this->_tcpPcb,
            [](
                void* const arg,
                struct tcp_pcb* const tcpPcb,
                struct pbuf* const pBuf,
                const err_t err
            ) {
                D(4, TAG, "tcp recv");
                // TODO check response for HTTP 2xx.
                return static_cast<err_t>(ERR_OK);
            }
        );

        tcp_err(
            this->_tcpPcb,
            [](
                void* const arg,
                const err_t err
            ) {
                E(TAG, "TCP fatal error: " << translate(err));
                if (ERR_ABRT != err)
                    static_cast<Homer2Pusher*>(arg)->close();
            }
        );

        return true;
    }

    [[nodiscard]]
    bool Homer2Pusher::connect() noexcept {

        D(5, TAG, "connecting tcp");

        this->_connectionStartMillis = now();

        cyw43_arch_lwip_begin();
        this->_connection = internal::ConnectionStatus::CONNECTING;
        const err_t err = tcp_connect(
            this->_tcpPcb,
            &this->_ip,
            this->_port,
            [](
                void* const arg,
                struct tcp_pcb* const tcpPcb,
                const err_t err
            ) {
                auto that = static_cast<Homer2Pusher*>(arg);

                if (ERR_OK != err) {
                    E(TAG, "connection failed: " << translate(err));
                    that->incTcpErr();
                    const auto closeErr = that->close();
                    return ERR_OK == closeErr ? err : closeErr;
                }
                else {
                    D(4, TAG, "connection successful");
                    that->_connection = internal::ConnectionStatus::CONNECTED;
                    return static_cast<err_t>(ERR_OK);
                }
            }
        );
        cyw43_arch_lwip_end();

        if (ERR_OK != err) {
            E(TAG, "connection failed: " << translate(err));
            this->incTcpErr();
            this->close();
            return false;
        }

        this->waitForConnection();

        return true;
    }

    void Homer2Pusher::waitForConnection() const noexcept {

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
#pragma ide diagnostic ignored "ConstantConditionsOC"
        while (internal::ConnectionStatus::CONNECTING == this->_connection)
            sleep_us(100);
#pragma clang diagnostic pop
    }

    [[nodiscard]]
    bool Homer2Pusher::write() noexcept {

        D(5, TAG, "writing tcp");

        this->_writeBuffer = this->_header;
        this->_writeBuffer += std::to_string(this->_body.size());
        this->_writeBuffer += "\n\n";
        this->_writeBuffer += this->_body;

        const auto available = tcp_sndbuf(this->_tcpPcb);

        if (available < this->_writeBuffer.size()) {
            W(TAG, "TCP buffer full, skipping");
            this->close();
            return false;
        }

        // err_t tcpErr;

        cyw43_arch_lwip_begin();
        const auto bodyErr = tcp_write(
            this->_tcpPcb,
            this->_writeBuffer.data(),
            this->_writeBuffer.size(),
            TCP_WRITE_FLAG_COPY
        );
        // if (ERR_OK == bodyErr)
        //     tcpErr = tcp_output(this->_tcpPcb);
        cyw43_arch_lwip_end();

        if (ERR_OK != bodyErr) {
            E(TAG, "could not write body: " << std::to_string(bodyErr));
            this->close();
            this->incTcpErr();
            return false;
        }
        else {
            D(0, TAG, "wrote to server");
        }

        // if (ERR_OK != tcpErr) {
        //     E(TAG, "could not flush tcp: " << std::to_string(bodyErr));
        //     this->close();
        //     this->incTcpErr();
        //     return false;
        // }
        // else {
        //     D(4, TAG, "flushed tcp");
        // }

        this->close();

        return true;
    }

    err_t Homer2Pusher::close() noexcept {

        D(5, TAG, "tcp close");

        if (this->_tcpPcb != nullptr) {

            tcp_arg(this->_tcpPcb, nullptr);
            tcp_poll(this->_tcpPcb, nullptr, 0);
            tcp_sent(this->_tcpPcb, nullptr);
            tcp_recv(this->_tcpPcb, nullptr);
            tcp_err(this->_tcpPcb, nullptr);

            D(4, TAG, "closing tcp");
            const auto err = tcp_close(this->_tcpPcb);

            if (ERR_OK != err) {
                E(TAG, "failed to close tcp connection, aborting: " << translate(err));
                this->incTcpErr();
                tcp_abort(this->_tcpPcb);
                this->_tcpPcb = nullptr;
                return ERR_ABRT;
            }
            else {
                this->_tcpPcb = nullptr;
            }
        }

        this->_connectionStartMillis = 0;
        this->_lastPushMillis = now();
        this->_connection = internal::ConnectionStatus::DISCONNECTED;
        return ERR_OK;
    }


    [[nodiscard]]
    const std::string& Homer2Pusher::ipAddr() noexcept {

        if (this->_ipStr.empty()) {
            cyw43_arch_lwip_begin();
            const char* const addr = ipaddr_ntoa(&this->_ip);
            this->_ipStr = std::move(std::string{addr});
            cyw43_arch_lwip_end();
        }

        return this->_ipStr;
    }

    void Homer2Pusher::incTcpErr() noexcept {

        if (this->_tcpErrors < std::numeric_limits<uint8_t>::max())
            this->_tcpErrors++;
    }

    void Homer2Pusher::incDnsErr() noexcept {

        if (this->_dnsErrors < std::numeric_limits<uint8_t>::max())
            this->_dnsErrors++;
    }

    void Homer2Pusher::resetTcpErr() noexcept {

        if (this->_tcpErrors > 0)
            this->_tcpErrors--;
    }

    void Homer2Pusher::resetDnsErr() noexcept {

        if (this->_dnsErrors > 0)
            this->_dnsErrors--;
    }

}
