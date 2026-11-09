// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_MAPPING_MAPPING_SESSION_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_MAPPING_MAPPING_SESSION_H_

#include "spacemit_mapping/distance_field/field.h"
#include "spacemit_mapping/fusion/appearance.h"
#include "spacemit_mapping/fusion/frames.h"
#include "spacemit_mapping/map_storage/change_tracker.h"
#include "spacemit_mapping/mapping/decay.h"
#include "spacemit_mapping/mapping/freespace.h"
#include "spacemit_mapping/surface/mesh.h"

#include <memory>

namespace spacemit_mapping
{

enum class MapKind
{
    kNone,
    kTsdf,
    kOccupancy,
    kTsdfWithHistory
};
enum class DistanceMode
{
    kUnset,
    kVolume,
    kSlice
};
enum class ObservationSource
{
    kCamera,
    kLidar
};
inline constexpr ConsumerId kFirstExternalMapConsumer = 16;

struct MapSessionParams
{
    TsdfFrameParams tsdf;
    OccupancyFrameParams occupancy;
    AppearanceParams appearance;
    FreespaceParams freespace;
    DistanceFieldParams distance;
    MeshParams mesh;
    TsdfDecayOptions tsdf_decay;
    OccupancyDecayOptions occupancy_decay;
    HeightBand slice;
    float slice_above_plane_m = 0;
    float slice_thickness_m = 0.1F;
};

struct MapFrameUpdate
{
    TsdfFrameStats tsdf;
    OccupancyFrameStats occupancy;
    std::vector<Index3D> view_blocks, updated_blocks, color_blocks;
};

struct MapState; // Include mapping/state.h when using the owning state API.

// Owns all map state; borrowed layer references require caller synchronization.
class MapSession
{
public:
    explicit MapSession(float voxel_size_m, MapKind kind = MapKind::kTsdf,
        std::size_t maximum_pending_blocks = 100000);
    ~MapSession();
    MapSession(MapSession&&) noexcept;
    MapSession& operator=(MapSession&&) noexcept;
    MapSession(const MapSession&) = delete;
    MapSession& operator=(const MapSession&) = delete;
    float voxel_size() const noexcept;
    std::size_t pending_capacity() const noexcept;
    MapKind kind() const noexcept;
    DistanceMode distance_mode() const noexcept;
    const MapSessionParams& params() const noexcept;
    void set_params(const MapSessionParams& params);
    void clear();
    MapState snapshot() const;
    // Fixed kind/resolution must match; success invalidates old borrows/tickets.
    void restore(const MapState& state);

    MapFrameUpdate integrate_camera(ImageView<float> depth, const PinholeCamera& camera,
        const RigidTransform& pose, ImageView<Rgb8> aligned_rgb = {},
        ImageView<std::uint8_t> mask = {});
    MapFrameUpdate integrate_camera_in_blocks(ImageView<float> depth, const PinholeCamera& camera,
        const RigidTransform& pose, const std::vector<Index3D>& indices,
        ImageView<std::uint8_t> mask = {});
    MapFrameUpdate integrate_lidar(ImageView<float> ranges, const Lidar& lidar,
        const RigidTransform& pose, ImageView<std::uint8_t> mask = {});
    MapFrameUpdate integrate_pointcloud(const Pointcloud& points, const Lidar& lidar,
        const RigidTransform& start_pose, const std::optional<ScanMotion>& motion = std::nullopt);
    AppearanceResult integrate_color(ImageView<Rgb8> rgb, const PinholeCamera& camera,
        const RigidTransform& pose, ImageView<std::uint8_t> mask = {});
    AppearanceResult integrate_color(ImageView<Rgb8> rgb, ImageView<float> depth,
        const PinholeCamera& camera, const RigidTransform& pose, ImageView<std::uint8_t> mask = {});

    std::vector<Index3D> update_freespace(TimestampMs time_ms, bool full = false);
    std::vector<Index3D> update_freespace(TimestampMs time_ms, ImageView<float> depth,
        const PinholeCamera& camera, const RigidTransform& pose, bool full = false);
    std::vector<Index3D> update_freespace(TimestampMs time_ms, ImageView<float> ranges,
        const Lidar& lidar, const RigidTransform& pose, bool full = false);
    DistanceUpdate update_distance(bool full = false);
    DistanceUpdate update_slice(
        bool full = false, const std::optional<Plane>& ground = std::nullopt);
    DistanceUpdate update_slice(HeightBand band, bool full = false);
    DistanceUpdate update_slice(const PlaneBand& band, bool full = false);
    MeshUpdate update_mesh(bool full = false);

    DecayResult decay(const BlockExclusions& exclusions = {});
    DecayResult decay_excluding_last_view(
        ObservationSource source, const BlockExclusions& exclusions = {});
    std::vector<Index3D> erase_blocks(const std::vector<Index3D>& indices);
    std::vector<Index3D> erase_outside_sphere(BoundingSphere sphere);
    std::vector<Index3D> clear_inside_shapes(const std::vector<BoundingShape>& shapes);
    std::vector<Index3D> mark_unobserved_free(BoundingSphere sphere);

    void mark_changed(const std::vector<Index3D>& indices);
    void mark_all();
    UpdateBatch pending_updates(ConsumerId consumer);
    void acknowledge(const UpdateBatch& batch);
    std::vector<Index3D> all_block_indices() const;
    const std::vector<Index3D>& last_view_blocks() const noexcept;
    TimestampMs history_time() const;

    TsdfLayer& tsdf();
    const TsdfLayer& tsdf() const;
    OccupancyLayer& occupancy();
    const OccupancyLayer& occupancy() const;
    ColorLayer& colors();
    const ColorLayer& colors() const;
    FreespaceLayer& freespace();
    const FreespaceLayer& freespace() const;
    MeshLayer& mesh();
    const MeshLayer& mesh() const;
    DistanceField& distance_field();
    const DistanceField& distance_field() const;

private:
    struct State;
    std::unique_ptr<State> state_;
};

} // namespace spacemit_mapping

#endif // SPACEMIT_MAPPING_MAPPING_SESSION_H_
