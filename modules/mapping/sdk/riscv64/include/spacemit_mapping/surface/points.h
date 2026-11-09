// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_MAPPING_SURFACE_POINTS_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_MAPPING_SURFACE_POINTS_H_

#include "spacemit_mapping/map_storage/layer.h"

namespace spacemit_mapping
{

struct SurfacePoint
{
    Vector3f position;
    float signed_distance_m = 0;
    float weight = 0;
    Rgb8 color;
};

struct SurfacePointParams
{
    float maximum_absolute_distance_m = -1; // Negative means one voxel.
    float minimum_weight = 1.0F;
};

// Results own their values; input references are borrowed only for the call.
std::vector<SurfacePoint> extract_surface_points(
    const TsdfLayer& input, SurfacePointParams params = {});
std::vector<SurfacePoint> extract_surface_points(
    const TsdfLayer& input, const std::vector<Index3D>& indices, SurfacePointParams params = {});

} // namespace spacemit_mapping

#endif // SPACEMIT_MAPPING_SURFACE_POINTS_H_
