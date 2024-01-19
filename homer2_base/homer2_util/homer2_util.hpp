#pragma once

#include <cstdint>

#include <pico/time.h>

#define now() time_us_64() / 1000

#define a_until(DELAY_MS) from_us_since_boot(time_us_64() + ((DELAY_MS) * 1000))

#define is_expired(START_TIME_MILLIS, EXPIRY_DURATION_MILLIS) (((START_TIME_MILLIS) + (EXPIRY_DURATION_MILLIS)) <= (now()))

#define remaining(START_TIME_MILLIS, EXPIRY_DURATION_MILLIS) (((START_TIME_MILLIS) + (EXPIRY_DURATION_MILLIS)) - (now()))

inline uint16_t merge(
    const uint8_t hi,
    const uint8_t lo
) noexcept {

    const auto hi16 = static_cast<uint16_t>(static_cast<uint16_t>(hi) << 8);
    const auto lo16 = static_cast<uint16_t>(lo);
    const auto combine = static_cast<uint16_t>(hi16 | lo16);
    return combine;
}
