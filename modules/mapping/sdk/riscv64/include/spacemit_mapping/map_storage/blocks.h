// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_MAPPING_MAP_STORAGE_BLOCKS_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_MAPPING_MAP_STORAGE_BLOCKS_H_

#include "spacemit_mapping/core/types.h"

#include <array>

namespace spacemit_mapping
{

inline constexpr int kVoxelsPerSide = 8;
inline constexpr int kVoxelsPerBlock = 512;

// z is contiguous; linear = x*64 + y*8 + z. Distances are metres.
struct alignas(128) TsdfBlock
{
    std::array<float, kVoxelsPerBlock> distance_m{};
    std::array<float, kVoxelsPerBlock> weight{};
    std::array<std::uint8_t, kVoxelsPerBlock> red{}, green{}, blue{};
    std::array<float, kVoxelsPerBlock> color_weight{};
};

struct ColorVoxel
{
    Rgb8 color{127, 127, 127};
    float weight = 0;
};

struct alignas(128) ColorBlock
{
    std::array<ColorVoxel, kVoxelsPerBlock> voxels{};
};

struct alignas(128) OccupancyBlock
{
    std::array<float, kVoxelsPerBlock> log_odds{};
};

// Squared distance is in voxel units; parent offsets point toward the site.
// New storage is unobserved. Algorithms initialize active distance bounds.
struct alignas(128) EsdfBlock
{
    std::array<float, kVoxelsPerBlock> squared_distance_vox{};
    std::array<std::int16_t, kVoxelsPerBlock> parent_x{}, parent_y{}, parent_z{};
    std::array<std::uint8_t, kVoxelsPerBlock> observed{}, inside{}, site{};
};

struct alignas(128) FreespaceBlock
{
    std::array<TimestampMs, kVoxelsPerBlock> last_occupied_timestamp_ms{};
    std::array<TimestampMs, kVoxelsPerBlock> consecutive_occupancy_duration_ms{};
    std::array<std::uint8_t, kVoxelsPerBlock> high_confidence{};
};

} // namespace spacemit_mapping

#endif // SPACEMIT_MAPPING_MAP_STORAGE_BLOCKS_H_
