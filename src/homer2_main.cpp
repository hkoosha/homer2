#include <memory>

#include <hardware/watchdog.h>
#include <hardware/uart.h>
#include <hardware/i2c.h>

#include <homer2_util.hpp>
#include <homer2_logging.hpp>

#include "homer2_config.h"
#include "homer2_init.hpp"
#include "homer2_sensor.hpp"
#include "homer2_pusher.hpp"
#include "homer2_main.h"

using homer2::sensor::bme68x::BME68xOversampling;
using homer2::sensor::bme68x::BME68xIirFilterSize;
using homer2::sensor::bme68x::BME68x;
using homer2::sensor::bme68x::BME68xData;

using homer2::sensor::sht4x::SHT4x;
using homer2::sensor::sht4x::SHT4xData;

using homer2::sensor::sgp40::SGP40;

namespace {

    const char* const TAG = "Main";

    const char* const ppm = " ppm";
    const char* const hPa = " hPa";
    const char* const meters = " meters";

#if HOMER2_CONSOLE_UTF
    const char* const ugPerM3 = " µg/m³";
    const char* const celsius = " °C";
    const char* const percent = " %";
    const char* const ohms = " Ω";
#else
    const char* const ugPerM3 = " ug/m3";
    const char* const celsius = " Celsius";
    const char* const percent = " Percent";
    const char* const ohms = " Ohms";
#endif

    // Values are the longest in their category, selected for their length. The value itself is
    // irrelevant.
    const size_t TAG_WIDTH = strlen("PMSx00x");
    const size_t TITLE_WIDTH = strlen("Relative Humidity");

    void homer2_main_loop_delay() {

#pragma clang diagnostic push
#pragma ide diagnostic ignored "Simplify"
        if (HOMER2_SENSOR_LOOP_DELAY_MILLIS > 0)
            sleep_ms(HOMER2_SENSOR_LOOP_DELAY_MILLIS);
#pragma clang diagnostic pop

        tight_loop_contents();
    }

    template<typename T>
    void print(
        const char* const tag,
        const char* const title,
        const char* const unit,
        const T value
    ) {
        I(TAG, ""
            << std::setw(TAG_WIDTH) << std::setfill(' ') << std::left << tag
            << " | "
            << std::setw(TITLE_WIDTH) << std::setfill(' ') << std::left << title
            << ": " << value << unit
        );
    }

    void print(
        const homer2::Homer2SensorsData& data
    ) {
        if (data.bme68xData().has_value()) {
            const auto sData = data.bme68xData().value();
            print("BME68x", "Temperature", celsius, sData.getTemperatureCelsius());
            print("BME68x", "Relative Humidity", percent, sData.getRelativeHumidityPercent());
            print("BME68x", "Pressure", hPa, sData.getPressureHPa());
            print("BME68x", "Gas Resistance", ohms, sData.getGasResistanceOhms());
        }
        else if (homer2::is_enabled_bme68x()) {
            print("BME68x", "Temperature", "", '?');
            print("BME68x", "Relative Humidity", "", '?');
            print("BME68x", "Pressure", "", '?');
            print("BME68x", "Gas Resistance", "", '?');
        }

        if (data.sht4xData().has_value()) {
            const auto sData = data.sht4xData().value();
            print("SHT4x", "Temperature", celsius, sData.getTemperatureCelsius());
            print("SHT4x", "Relative Humidity", percent, sData.getRelativeHumidityPercent());
        }
        else if (homer2::is_enabled_sht4x()) {
            print("SHT4x", "Temperature", "", '?');
            print("SHT4x", "Relative Humidity", "", '?');
        }

        if (data.sgp40Data().has_value()) {
            const auto sData = data.sgp40Data().value();
            print("SGP40", "VOC Index", "", sData.getVocIndex());
        }
        else if (homer2::is_enabled_sgp40()) {
            print("SGP40", "VOC Index", "", '?');
        }

        if (data.bmp3xxData().has_value()) {
            const auto sData = data.bmp3xxData().value();
            print("BMP3xx", "Temperature", celsius, sData.getTemperatureCelsius());
            print("BMP3xx", "Pressure", hPa, sData.getPressureHPa());
            print("BMP3xx", "Altitude", meters, sData.getAltitude(1013.25F));
        }
        else if (homer2::is_enabled_bmp3xx()) {
            print("BMP3xx", "Temperature", "", '?');
            print("BMP3xx", "Pressure", "", '?');
            print("BMP3xx", "Altitude", "", '?');
        }

        if (data.sunriseData().has_value()) {
            const auto sData = data.sunriseData().value();
            print("Sunrise", "CO2", "ppm", sData.getCo2Ppm());
        }
        else if (homer2::is_enabled_sunrise()) {
            print("Sunrise", "CO2", "", '?');
        }

        if (data.pmsx00xData().has_value()) {
            const auto sData = data.pmsx00xData().value();
            print("PMSx00x", "PM 1.0", ugPerM3, sData.getPm10Env());
            print("PMSx00x", "PM 2.5 ", ugPerM3, sData.getPm25Env());
            print("PMSx00x", "PM 10.0", ugPerM3, sData.getPm100Env());
            print("PMSx00x", "PTC 0.3", ppm, sData.getParticles03());
            print("PMSx00x", "PTC 0.5", ppm, sData.getParticles05());
            print("PMSx00x", "PTC 1.0", ppm, sData.getParticles10());
            print("PMSx00x", "PTC 2.5", ppm, sData.getParticles25());
            print("PMSx00x", "PTC 5.0", ppm, sData.getParticles50());
            print("PMSx00x", "PTC 10.0", ppm, sData.getParticles100());
        }
        else if (homer2::is_enabled_pmsx00x()) {
            print("PMSx00x", "PM 1.0", "", "?");
            print("PMSx00x", "PM 2.5", "", "?");
            print("PMSx00x", "PM 10.0", "", "?");
            print("PMSx00x", "PTC 0.3", "", '?');
            print("PMSx00x", "PTC 0.5", "", '?');
            print("PMSx00x", "PTC 1.0", "", '?');
            print("PMSx00x", "PTC 2.5", "", '?');
            print("PMSx00x", "PTC 5.0", "", '?');
            print("PMSx00x", "PTC 10.0", "", '?');
        }
    }

    void ring0(
        const std::unique_ptr<homer2::Homer2Sensors>& sensors,
        const std::unique_ptr<homer2::Homer2Pusher>& pusher
    ) {
        for (uint64_t i = 0; i < std::numeric_limits<uint64_t>::max(); i++) {

            sensors->connectSensors();
            if (!sensors->hasAnySensor()) {
                if (homer2::terminate_on_no_sensor()) {
                    W(TAG, "no sensor was found, terminating");
                    return;
                }
                else {
                    continue;
                }
            }

            sensors->querySensors();

            const auto data = sensors->data();

            if (homer2::net::is_victoria_metrics_enabled())
                pusher->push(data);

            print(data);

            homer2_main_loop_delay();
        }
    }

    void ring1() {

        if (!homer2::init()) {
            E(TAG, "initialization failed");
            return;
        }
        I(TAG, "initialization finished successfully, entering homer2");

        // Homer2 sensors can't handle time=0 (they throw exception). Also, we have to wait for
        // the sensors to wake up and initialize.
        while (now() <= 100) {
            I(TAG, "waiting for current time to go after 100ms");
            sleep_ms(100);
        }

        auto sensors = std::make_unique<homer2::Homer2Sensors>(
            i2c1,
            uart1
        );

        auto pusher = homer2::net::is_victoria_metrics_enabled()
                      ?
                      std::make_unique<homer2::Homer2Pusher>(
                          homer2::net::victoria_addr(),
                          homer2::net::victoria_port(),
                          homer2::net::victoria_metrics_frequency_millis(),
                          homer2::net::victoria_metrics_write_initial_delay_millis() + now()
                      )
                      : nullptr;

        ring0(sensors, pusher);
    }

    void ring2() {

        // watchdog_enable(HOMER2_WATCHDOG_TIMEOUT_MILLIS, true);

        try {
            ring1();
        }
        catch (...) {
            E(TAG, "failed very badly, going to restart...");
        }

        W(TAG, "exited, using watchdog hack to restart" << std::endl << std::endl);
        sleep_ms(100);

        watchdog_enable(1, true);
        for (uint64_t i = 0; i < std::numeric_limits<uint64_t>::max(); i++)
            for (uint64_t j = 0; j < std::numeric_limits<uint64_t>::max(); j++)
                for (uint64_t k = 0; k < std::numeric_limits<uint64_t>::max(); k++);
    }

}

void homer2_main() {

    ring2();
}
