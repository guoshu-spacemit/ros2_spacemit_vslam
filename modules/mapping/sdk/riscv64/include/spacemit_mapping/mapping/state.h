// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_MAPPING_MAPPING_STATE_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_MAPPING_MAPPING_STATE_H_

#include "spacemit_mapping/mapping/session.h"
#include "spacemit_mapping/mapping/multi_session.h"

#include <variant>

namespace spacemit_mapping
{

struct CameraObservation
{
    std::vector<float> depth;
    PinholeCamera camera;
    RigidTransform pose;
};
struct LidarObservation
{
    std::vector<float> ranges;
    Lidar lidar;
    RigidTransform pose;
};

// Logical full-map state. Does not store container layouts or consumer tickets.
struct MapState
{
    float voxel_size_m = 0;
    MapKind kind = MapKind::kTsdf;
    std::size_t pending_capacity = 100000;
    MapSessionParams params;
    DistanceMode distance_mode = DistanceMode::kUnset;
    TimestampMs history_time_ms = 0;
    bool history_full_update = false;
    std::vector<Index3D> history_pending;
    std::optional<std::variant<HeightBand, PlaneBand>> slice_band;
    std::optional<CameraObservation> camera;
    std::optional<LidarObservation> lidar;
    std::vector<Index3D> last_view_blocks;
    std::vector<BlockUpdate<TsdfBlock>> tsdf;
    std::vector<BlockUpdate<OccupancyBlock>> occupancy;
    std::vector<BlockUpdate<ColorBlock>> colors;
    std::vector<BlockUpdate<FreespaceBlock>> history;
    std::vector<BlockUpdate<MeshBlock>> mesh;
    std::optional<DistanceFieldState> distance;
};

struct MultiMapState
{
    MappingMode mode = MappingMode::kStaticTsdf;
    DistanceMode distance = DistanceMode::kVolume;
    MultiMapParams params;
    MapState background, foreground;
};

} // namespace spacemit_mapping

#endif // SPACEMIT_MAPPING_MAPPING_STATE_H_
