// Extend one hardware clock for exposure and IMU metadata.

#ifndef TIMESTAMP_UNWRAP_H
#define TIMESTAMP_UNWRAP_H

#include "camera_sdk/embedded_metadata.h"

#include <limits>
#include <stdexcept>

namespace camera_sdk::detail
{

inline std::uint64_t unwrap_near(std::uint64_t reference, std::uint32_t raw)
{
    constexpr std::uint64_t wrap = std::uint64_t{1} << 32;
    constexpr std::uint64_t half = wrap >> 1;
    std::uint64_t value = (reference & ~(wrap - 1)) | raw;
    if (value > reference && value - reference > half)
    {
        if (value < wrap)
        {
            throw std::overflow_error("camera timestamp precedes the extended clock epoch");
        }
        value -= wrap;
    }
    else if (reference > value && reference - value > half)
    {
        if (value > std::numeric_limits<std::uint64_t>::max() - wrap)
        {
            throw std::overflow_error("camera timestamp exceeds the extended clock range");
        }
        value += wrap;
    }
    return value;
}

inline std::uint64_t unwrap_metadata_timestamps(
    EmbeddedMetadata& metadata, std::uint64_t last_timestamp, bool have_timestamp)
{
    constexpr std::uint64_t wrap = std::uint64_t{1} << 32;
    // Reserve an earlier cycle for samples captured just before the first frame's wrap.
    const std::uint64_t unwrapped = have_timestamp
        ? unwrap_near(last_timestamp, metadata.exposure_end_raw_us)
        : wrap + metadata.exposure_end_raw_us;
    metadata.exposure_end_us = unwrapped;
    metadata.exposure_start_us = unwrap_near(unwrapped, metadata.exposure_start_raw_us);
    for (std::size_t index = 0; index < metadata.imu_count; ++index)
    {
        metadata.imu[index].timestamp_us =
            unwrap_near(unwrapped, metadata.imu[index].timestamp_raw_us);
    }
    return unwrapped;
}

} // namespace camera_sdk::detail

#endif // TIMESTAMP_UNWRAP_H
