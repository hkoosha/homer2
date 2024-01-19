#pragma once

#include <string_view>

#include "homer2_config.h"

namespace homer2::net {

    uint8_t dns_max_tries() noexcept;


    uint8_t tcp_max_tries() noexcept;

    uint64_t tcp_write_timeout_millis() noexcept;


    const char* victoria_addr() noexcept;

    uint16_t victoria_port();

    uint64_t victoria_metrics_frequency_millis() noexcept;

    bool is_victoria_metrics_enabled() noexcept;

}

namespace homer2 {

    void init_delay();

    void init_i2c1();

    void init_uart1();

    void init_dns();

    [[nodiscard]]
    bool init();

}

namespace homer2 {

    [[nodiscard]]
    bool terminate_on_no_sensor() noexcept;


    [[nodiscard]]
    bool is_enabled_bmp3xx() noexcept;

    [[nodiscard]]
    bool is_enabled_bme68x() noexcept;

    [[nodiscard]]
    bool is_enabled_sht4x() noexcept;

    [[nodiscard]]
    bool is_enabled_sgp40() noexcept;

    [[nodiscard]]
    bool is_enabled_sunrise() noexcept;

    [[nodiscard]]
    bool is_enabled_pmsx00x() noexcept;

}

namespace homer2 {

    enum class TemperatureSource {
        disabled,
        constant,
        bme68x,
        sht4x,
        bmp3xx,
    };

    std::ostream& operator<<(
        std::ostream& out,
        TemperatureSource value
    );


    enum class HumiditySource {
        disabled,
        constant,
        bme68x,
        sht4x,
    };

    std::ostream& operator<<(
        std::ostream& out,
        HumiditySource value
    );

    [[nodiscard]]
    HumiditySource humidity_source0() noexcept;

    [[nodiscard]]
    HumiditySource humidity_source1() noexcept;

    [[nodiscard]]
    HumiditySource humidity_source2() noexcept;

    [[nodiscard]]
    HumiditySource humidity_source3() noexcept;


    [[nodiscard]]
    TemperatureSource temperature_source0() noexcept;

    [[nodiscard]]
    TemperatureSource temperature_source1() noexcept;

    [[nodiscard]]
    TemperatureSource temperature_source2() noexcept;

    [[nodiscard]]
    TemperatureSource temperature_source3() noexcept;

}

namespace homer2 {

    bool wifi_connect() noexcept;

}
