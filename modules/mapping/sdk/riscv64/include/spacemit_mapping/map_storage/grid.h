// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_MAPPING_MAP_STORAGE_GRID_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_MAPPING_MAP_STORAGE_GRID_H_

#include "spacemit_mapping/map_storage/blocks.h"

namespace spacemit_mapping
{

int linear_index(Index3D local);
Index3D local_index(int linear);
Index3D block_index_from_voxel(Index3D global);
Index3D local_index_from_voxel(Index3D global);
Index3D global_voxel_index(Index3D block, Index3D local);
Index3D voxel_index_from_position(Vector3f position_m, float voxel_size_m);
Index3D block_index_from_position(Vector3f position_m, float voxel_size_m);
Vector3f voxel_center(Index3D block, Index3D local, float voxel_size_m);

namespace detail
{

inline float rounding_barrier(float value)
{
#if defined(__has_builtin)
#if __has_builtin(__builtin_assoc_barrier)
    return __builtin_assoc_barrier(value);
#endif
#endif
    volatile float rounded = value;
    return rounded;
}

} // namespace detail

// Hot-path form: caller validates local in [0,8) and consistent positive sizes.
// Keep products/additions rounded separately across compilers that contract FMA.
inline float voxel_center_coordinate(
    std::int32_t block, int local, float block_size_m, float voxel_size_m)
{
    const float half_voxel = block_size_m * (0.5F / static_cast<float>(kVoxelsPerSide));
    const float block_offset = detail::rounding_barrier(block_size_m * static_cast<float>(block));
    const float local_offset = detail::rounding_barrier(voxel_size_m * static_cast<float>(local));
    const float corner = detail::rounding_barrier(block_offset + local_offset);
    return corner + half_voxel;
}

} // namespace spacemit_mapping

#endif // SPACEMIT_MAPPING_MAP_STORAGE_GRID_H_
