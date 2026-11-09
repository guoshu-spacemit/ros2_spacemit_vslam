// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_MAPPING_MAPPING_MULTI_SESSION_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_MAPPING_MAPPING_MULTI_SESSION_H_

#include "spacemit_mapping/mapping/dynamics.h"
#include "spacemit_mapping/mapping/masks.h"
#include "spacemit_mapping/mapping/session.h"

namespace spacemit_mapping
{

enum class MappingMode
{
    kStaticTsdf,
    kStaticOccupancy,
    kDynamic,
    kHumanTsdf,
    kHumanOccupancy
};
struct MultiMapParams
{
    int minimum_mask_area = 2000;
    bool clean_small_components = true;
    float box_proximity_m = 0.2F;
};
struct MultiMapFrame
{
    MapFrameUpdate background, foreground;
};
struct MultiMapDistance
{
    DistanceUpdate background, foreground;
};

struct MultiMapState; // Include mapping/state.h for the owning state API.

// Owns two maps and all results; caller coordinates access to borrowed getters.
class MultiMapSession
{
public:
    MultiMapSession(float voxel_size_m, MappingMode mode,
        DistanceMode distance = DistanceMode::kVolume, std::size_t maximum_pending_blocks = 100000);
    MultiMapSession(MultiMapSession&&) noexcept = default;
    MultiMapSession& operator=(MultiMapSession&&) noexcept = default;
    MultiMapSession(const MultiMapSession&) = delete;
    MultiMapSession& operator=(const MultiMapSession&) = delete;
    MappingMode mode() const noexcept { return mode_; }
    DistanceMode distance_preference() const noexcept { return distance_; }
    const MultiMapParams& params() const noexcept { return settings_; }
    void set_params(const MultiMapParams& params);
    void set_map_params(const MapSessionParams& background,
        const std::optional<MapSessionParams>& foreground = std::nullopt);
    void clear();
    MultiMapState snapshot() const;
    void restore(const MultiMapState& state);

    MultiMapFrame integrate_camera(ImageView<float> depth, const PinholeCamera& camera,
        const RigidTransform& pose, std::optional<TimestampMs> time_ms = std::nullopt);
    MultiMapFrame integrate_lidar(ImageView<float> ranges, const Lidar& lidar,
        const RigidTransform& pose, std::optional<TimestampMs> time_ms = std::nullopt);
    MultiMapFrame integrate_pointcloud(const Pointcloud& points, const Lidar& lidar,
        const RigidTransform& start_pose, const std::optional<ScanMotion>& motion = std::nullopt,
        std::optional<TimestampMs> time_ms = std::nullopt);
    MultiMapFrame integrate_masked_camera(ImageView<float> depth, const PinholeCamera& camera,
        const RigidTransform& pose, ImageView<std::uint8_t> foreground_mask);
    MultiMapFrame integrate_masked_camera(ImageView<float> depth, const PinholeCamera& depth_camera,
        const RigidTransform& pose, ImageView<std::uint8_t> foreground_mask,
        const PinholeCamera& mask_camera, const RigidTransform& mask_from_depth);
    MultiMapFrame integrate_camera_boxes(ImageView<float> depth, const PinholeCamera& camera,
        const RigidTransform& pose, const std::vector<ImageBox>& boxes);
    AppearanceResult integrate_color(
        ImageView<Rgb8> image, const PinholeCamera& camera, const RigidTransform& pose);
    AppearanceResult integrate_color(ImageView<Rgb8> image, const PinholeCamera& camera,
        const RigidTransform& pose, ImageView<std::uint8_t> foreground_mask);
    // Preserved behavior: color boxes are ignored; this integrates the full frame.
    AppearanceResult integrate_color(ImageView<Rgb8> image, const PinholeCamera& camera,
        const RigidTransform& pose, const std::vector<ImageBox>& boxes);
    MultiMapDistance update_distance(const std::optional<Plane>& ground = std::nullopt);
    MeshUpdate update_mesh();

    MapSession& background() noexcept { return background_; }
    const MapSession& background() const noexcept { return background_; }
    MapSession& foreground() noexcept { return foreground_; }
    const MapSession& foreground() const noexcept { return foreground_; }
    const DynamicsFrame& last_dynamics() const noexcept { return dynamics_; }
    const std::vector<std::uint8_t>& last_cleaned_mask() const noexcept { return mask_; }
    const DepthSplit& last_depth_split() const noexcept { return split_; }
    const MultiMapFrame& last_frame() const noexcept { return frame_; }
    const MultiMapDistance& last_distance() const noexcept { return distance_result_; }

private:
    bool human() const noexcept;
    std::vector<std::uint8_t> prepare_mask(ImageView<std::uint8_t> image) const;
    MappingMode mode_;
    DistanceMode distance_;
    MultiMapParams settings_;
    MapSession background_, foreground_;
    DynamicsFrame dynamics_;
    std::vector<std::uint8_t> mask_;
    DepthSplit split_;
    MultiMapFrame frame_;
    MultiMapDistance distance_result_;
};

} // namespace spacemit_mapping

#endif // SPACEMIT_MAPPING_MAPPING_MULTI_SESSION_H_
