#include <map>
#include <iomanip>
#include <iostream>

#include <pico/stdio.h>
#include <pico/cyw43_arch.h>
#include <pico/binary_info/code.h>
#include <pico/time.h>
#include <hardware/gpio.h>
#include <hardware/i2c.h>
#include <hardware/uart.h>

#include <homer2_logging.hpp>

#include "homer2_config.h"
#include "homer2_init.hpp"

namespace {

    const char* const TAG = "Init";

}

namespace homer2::net {

    uint8_t dns_max_tries() noexcept {

        return static_cast<uint8_t>(HOMER2_DNS_MAX_TRIES);
    }


    uint8_t tcp_max_tries() noexcept {

        return static_cast<uint8_t>(HOMER2_TCP_MAX_TRIES);
    }

    uint64_t tcp_write_timeout_millis() noexcept {

        return static_cast<uint64_t>(HOMER2_TCP_WRITE_TIMEOUT_MILLIS);
    }


    const char* victoria_addr() noexcept {

        return HOMER2_VICTORIA_ADDR;
    }

    uint16_t victoria_port() {

        char* endPtr;
        const auto value = std::strtol(HOMER2_VICTORIA_PORT, &endPtr, 10);

        if (*endPtr != '\0' || value < 0 || value > 65535) {
            E(TAG, "invalid victoria metrics port: " << HOMER2_VICTORIA_PORT);
            throw std::runtime_error{"invalid victoria metrics port"};
        }

        return static_cast<uint16_t>(value);
    }

    uint64_t victoria_metrics_frequency_millis() noexcept {

        return static_cast<uint64_t>(HOMER2_VICTORIA_FREQUENCY_MILLIS);
    }


    bool is_victoria_metrics_enabled() noexcept {

#pragma clang diagnostic push
#pragma ide diagnostic ignored "Simplify"
#pragma ide diagnostic ignored "UnreachableCode"
#pragma ide diagnostic ignored "ConstantConditionsOC"
        return HOMER2_WIFI &&
               victoria_metrics_frequency_millis() > 0 &&
               strlen(victoria_addr()) > 0;
#pragma clang diagnostic pop
    }

}

namespace homer2 {

    void init_delay() {

#pragma clang diagnostic push
#pragma ide diagnostic ignored "UnreachableCode"
#pragma clang diagnostic push
#pragma ide diagnostic ignored "Simplify"

        if (0 == HOMER2_INITIAL_DELAY_MILLIS)
            return;

        // So that when UART is connected, then homer2 version can be read in terminal.
        I("Init", "sleeping for " << HOMER2_INITIAL_DELAY_MILLIS << " milliseconds...");

        uint32_t i;

        const auto seconds = HOMER2_INITIAL_DELAY_MILLIS / 1000;
        for (i = 0; i < seconds; ++i) {
            I("Init", "sleep #" << i);
            sleep_ms(1000);
        }

        const auto millis = HOMER2_INITIAL_DELAY_MILLIS % 1000;
        if (millis > 0) {
            I("Init", "sleep fin #" << i);
            sleep_ms(millis);
        }

        std::cout << std::endl;

#pragma clang diagnostic pop
#pragma clang diagnostic pop
    }

    void init_i2c1() {

#pragma clang diagnostic push
#pragma ide diagnostic ignored "ConstantConditionsOC"
#pragma clang diagnostic push
#pragma ide diagnostic ignored "UnreachableCode"
        if (!is_enabled_sunrise() &&
            !is_enabled_bmp3xx() &&
            !is_enabled_sgp40() &&
            !is_enabled_sht4x() &&
            !is_enabled_bme68x()
            ) {
            I("Init", "not initializing I2C as non of sensors on I2C bus are enabled");
            return;
        }
#pragma clang diagnostic pop
#pragma clang diagnostic pop

        i2c_init(i2c1, HOMER2_I2C1_BAUDRATE);
        gpio_set_function(HOMER2_I2C1_PIN_SDA, gpio_function::GPIO_FUNC_I2C);
        gpio_set_function(HOMER2_I2C1_PIN_SCL, gpio_function::GPIO_FUNC_I2C);
        gpio_pull_up(HOMER2_I2C1_PIN_SDA);
        gpio_pull_up(HOMER2_I2C1_PIN_SCL);
        bi_decl(bi_2pins_with_func(HOMER2_I2C1_PIN_SDA, HOMER2_I2C1_PIN_SCL, gpio_function::GPIO_FUNC_I2C));
    }

    void init_uart1() {

#pragma clang diagnostic push
#pragma ide diagnostic ignored "ConstantConditionsOC"
#pragma clang diagnostic push
#pragma ide diagnostic ignored "UnreachableCode"
        if (!is_enabled_pmsx00x()) {
            I("Init", "not initializing UART as PMSx00x is not enabled");
            return;
        }
#pragma clang diagnostic pop
#pragma clang diagnostic pop

        uart_init(uart1, 9600);

        gpio_set_function(HOMER2_UART1_PIN_TX, GPIO_FUNC_UART);
        gpio_set_function(HOMER2_UART1_PIN_RX, GPIO_FUNC_UART);

        uart_set_hw_flow(uart1, false, false);
        uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
        uart_set_fifo_enabled(uart1, true);
    }

    void init_dns() {

        if (!net::is_victoria_metrics_enabled()) {
            W(TAG, "victoria metrics not enabled, not initializing dns");
            return;
        }

        I("Init", "init dns, server: " << HOMER2_DNS_SERVER);
        auto server = new ip_addr_t;
        server->addr = ipaddr_aton(HOMER2_DNS_SERVER, nullptr);
    }

    bool init() {

        if (!stdio_init_all())
            return false;

        homer2::logging::init();

        init_delay();

        I("Init", "homer2 v" << HOMER2_VERSION_MAJOR << '.' << HOMER2_VERSION_MINOR);

        if (!wifi_connect())
            return false;

        init_dns();

        init_i2c1();
        init_uart1();

        return true;
    }

}

namespace homer2 {

    [[nodiscard]]
    bool terminate_on_no_sensor() noexcept {

        return HOMER2_TERMINATE_ON_NO_SENSOR;
    }


    [[nodiscard]]
    bool is_enabled_bmp3xx() noexcept {

        return HOMER2_SENSOR_ENABLED_BMP3XX;
    }

    [[nodiscard]]
    bool is_enabled_bme68x() noexcept {

        return HOMER2_SENSOR_ENABLED_BME68X;
    }

    [[nodiscard]]
    bool is_enabled_sht4x() noexcept {

        return HOMER2_SENSOR_ENABLED_SHT4X;
    }

    [[nodiscard]]
    bool is_enabled_sgp40() noexcept {

        return HOMER2_SENSOR_ENABLED_SGP40;
    }

    [[nodiscard]]
    bool is_enabled_sunrise() noexcept {

        return HOMER2_SENSOR_ENABLED_SUNRISE;
    }

    [[nodiscard]]
    bool is_enabled_pmsx00x() noexcept {

        return HOMER2_SENSOR_ENABLED_PMSX00X;
    }

}

namespace homer2 {

    std::ostream& operator<<(
        std::ostream& out,
        TemperatureSource value
    ) {
        static std::map<TemperatureSource, std::string_view> strings{
            {TemperatureSource::disabled, "disabled"},
            {TemperatureSource::constant, "constant"},
            {TemperatureSource::bme68x,   "bme68x"},
            {TemperatureSource::sht4x,    "sht4x"},
            {TemperatureSource::bmp3xx,   "bmp3xx"},
        };

        return out << strings[value];
    }

    std::ostream& operator<<(
        std::ostream& out,
        HumiditySource value
    ) {
        static std::map<HumiditySource, std::string_view> strings{
            {HumiditySource::disabled, "disabled"},
            {HumiditySource::constant, "constant"},
            {HumiditySource::bme68x,   "bme68x"},
            {HumiditySource::sht4x,    "sht4x"},
        };

        return out << strings[value];
    }


    [[nodiscard]]
    HumiditySource humidity_source0() noexcept {

        return HOMER2_SOURCE_0_HUMIDITY;
    }

    [[nodiscard]]
    HumiditySource humidity_source1() noexcept {

        return HOMER2_SOURCE_1_HUMIDITY;
    }

    [[nodiscard]]
    HumiditySource humidity_source2() noexcept {

        return HOMER2_SOURCE_2_HUMIDITY;
    }

    [[nodiscard]]
    HumiditySource humidity_source3() noexcept {

        return HOMER2_SOURCE_3_HUMIDITY;
    }


    [[nodiscard]]
    TemperatureSource temperature_source0() noexcept {

        return HOMER2_SOURCE_0_TEMPERATURE;
    }

    [[nodiscard]]
    TemperatureSource temperature_source1() noexcept {

        return HOMER2_SOURCE_1_TEMPERATURE;
    }

    [[nodiscard]]
    TemperatureSource temperature_source2() noexcept {

        return HOMER2_SOURCE_2_TEMPERATURE;
    }

    [[nodiscard]]
    TemperatureSource temperature_source3() noexcept {

        return HOMER2_SOURCE_3_TEMPERATURE;
    }

}

namespace homer2 {

    namespace {

        void print_hwaddr() noexcept {

            for (int i = 0; i < 32; i++) {
                const struct netif* net = netif_get_by_index(++i);
                if (net)
                    I("wifi", "hw-addr at interface#"
                        << i << ": " << std::hex << std::setw(2) << std::setfill('0') << std::uppercase
                        << static_cast<uint64_t>(net->hwaddr[0]) << ':'
                        << static_cast<uint64_t>(net->hwaddr[1]) << ':'
                        << static_cast<uint64_t>(net->hwaddr[2]) << ':'
                        << static_cast<uint64_t>(net->hwaddr[3]) << ':'
                        << static_cast<uint64_t>(net->hwaddr[4]) << ':'
                        << static_cast<uint64_t>(net->hwaddr[5]));

            }

            I("wifi", "hw-addr by cy43: "
                << std::hex << std::setw(2) << std::setfill('0') << std::uppercase
                << static_cast<uint64_t>(cyw43_state.mac[0]) << ':'
                << static_cast<uint64_t>(cyw43_state.mac[1]) << ':'
                << static_cast<uint64_t>(cyw43_state.mac[2]) << ':'
                << static_cast<uint64_t>(cyw43_state.mac[3]) << ':'
                << static_cast<uint64_t>(cyw43_state.mac[4]) << ':'
                << static_cast<uint64_t>(cyw43_state.mac[5]));
        }

    }

    bool wifi_connect() noexcept {

#if !HOMER2_WIFI
        return true;
#endif

        if (PICO_OK != cyw43_arch_init_with_country(CYW43_COUNTRY(WIFI_COUNTRY[0], WIFI_COUNTRY[1], 0))) {
            E("wifi", "wifi initialization failed, country=" << WIFI_COUNTRY);
            return false;
        }

        cyw43_arch_enable_sta_mode();

        print_hwaddr();

        I("wifi", "connecting to: " << WIFI_SSID);
        if (PICO_OK != cyw43_arch_wifi_connect_timeout_ms(
            WIFI_SSID,
            WIFI_PASSWORD,
            CYW43_AUTH_WPA2_AES_PSK,
            HOMER2_WIFI_CONNECTION_TIMEOUT_MS)
            ) {
            E("wifi", "connection failed");
            return false;
        }

        I("wifi", "connected");
        return true;
    }

}
