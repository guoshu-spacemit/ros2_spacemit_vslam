// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_MAPPING_MAPPING_DECAY_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_MAPPING_MAPPING_DECAY_H_

#include "spacemit_mapping/fusion/kernels.h"
#include "spacemit_mapping/geometry/bounding_shape.h"
#include "spacemit_mapping/mapping/views.h"
#include "spacemit_mapping/map_storage/layer.h"

namespace spacemit_mapping
{

struct BlockExclusions
{
    std::vector<Index3D> blocks;
    std::optional<BoundingSphere> origin_sphere;
};

struct TsdfDecayOptions
{
    float factor = 0.95F;
    float minimum_weight = 1.0e-3F;
    bool set_free_distance = false;
    float free_distance_vox = 4;
    bool erase_decayed_blocks = true;
    Backend backend = Backend::kAuto;
};

struct OccupancyDecayOptions
{
    OccupancyDecayKernelParams kernel;
    bool erase_decayed_blocks = true;
    Backend backend = Backend::kAuto;
};

struct DecayResult
{
    std::vector<Index3D> candidate_blocks;
    std::vector<Index3D> updated_blocks;
    std::vector<Index3D> removed_blocks;
};

// Layers are caller-owned. View storage is borrowed only for the call.
DecayResult decay_layer(
    TsdfLayer& layer, const TsdfDecayOptions& options = {}, const BlockExclusions& exclusions = {});
DecayResult decay_layer(TsdfLayer& layer, const TsdfDecayOptions& options,
    const BlockExclusions& exclusions, const CameraMapView& view);
DecayResult decay_layer(TsdfLayer& layer, const TsdfDecayOptions& options,
    const BlockExclusions& exclusions, const LidarMapView& view);
DecayResult decay_layer(OccupancyLayer& layer, const OccupancyDecayOptions& options = {},
    const BlockExclusions& exclusions = {});
DecayResult decay_layer(OccupancyLayer& layer, const OccupancyDecayOptions& options,
    const BlockExclusions& exclusions, const CameraMapView& view);
DecayResult decay_layer(OccupancyLayer& layer, const OccupancyDecayOptions& options,
    const BlockExclusions& exclusions, const LidarMapView& view);

} // namespace spacemit_mapping

#endif // SPACEMIT_MAPPING_MAPPING_DECAY_H_
