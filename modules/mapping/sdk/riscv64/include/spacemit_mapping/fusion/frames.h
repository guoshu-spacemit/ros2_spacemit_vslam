// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_MAPPING_FUSION_FRAMES_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_MAPPING_FUSION_FRAMES_H_

#include "spacemit_mapping/fusion/kernels.h"
#include "spacemit_mapping/geometry/lidar.h"
#include "spacemit_mapping/map_storage/layer.h"

#include <memory>

namespace spacemit_mapping
{

struct TsdfFrameParams
{
    TsdfKernelParams kernel;
    float maximum_distance_m = 7;
    std::uint32_t ray_subsampling = 4;
    bool cull_blocks = true;
    Backend backend = Backend::kAuto;
};

struct TsdfFrameStats
{
    std::size_t allocated_blocks = 0;
    std::size_t candidate_block_keys = 0;
    std::size_t unique_block_keys = 0;
    std::size_t culled_blocks = 0;
    std::size_t candidate_voxels = 0;
    TsdfKernelCounts kernel;
    std::size_t colored_voxels = 0;
};

struct TsdfFrameResult
{
    TsdfFrameStats stats;
    std::vector<Index3D> view_blocks;
    std::vector<Index3D> updated_blocks;
    std::vector<Index3D> color_blocks;
};

// Layers belong to the caller; this object owns only reusable geometry/key/batch
// workspaces. Returned index lists own their data and survive later calls.
class TsdfFusion
{
public:
    explicit TsdfFusion(TsdfFrameParams params = {});
    ~TsdfFusion();
    TsdfFusion(TsdfFusion&&) noexcept;
    TsdfFusion& operator=(TsdfFusion&&) noexcept;
    TsdfFusion(const TsdfFusion&) = delete;
    TsdfFusion& operator=(const TsdfFusion&) = delete;
    const TsdfFrameParams& params() const noexcept { return params_; }
    void set_params(const TsdfFrameParams& params);
    TsdfFrameResult integrate_camera(TsdfLayer& layer, ImageView<float> depth,
        const PinholeCamera& camera, const RigidTransform& layer_from_camera,
        ImageView<Rgb8> aligned_rgb = {}, ImageView<std::uint8_t> active_mask = {});
    TsdfFrameResult integrate_lidar(TsdfLayer& layer, ImageView<float> ranges, const Lidar& lidar,
        const RigidTransform& layer_from_sensor, ImageView<std::uint8_t> active_mask = {});

private:
    struct Workspace;
    std::unique_ptr<Workspace> workspace_;
    TsdfFrameParams params_;
};

struct OccupancyFrameParams
{
    OccupancyKernelParams kernel;
    float maximum_distance_m = 7;
    float truncation_vox = 4;
    std::uint32_t ray_subsampling = 4;
    Backend backend = Backend::kAuto;
};

struct OccupancyFrameStats
{
    std::size_t allocated_blocks = 0;
    std::size_t visible_blocks = 0;
    std::size_t candidate_voxels = 0;
    std::size_t updated_voxels = 0;
};

struct OccupancyFrameResult
{
    OccupancyFrameStats stats;
    std::vector<Index3D> updated_blocks;
};

class OccupancyFusion
{
public:
    explicit OccupancyFusion(OccupancyFrameParams params = {});
    ~OccupancyFusion();
    OccupancyFusion(OccupancyFusion&&) noexcept;
    OccupancyFusion& operator=(OccupancyFusion&&) noexcept;
    OccupancyFusion(const OccupancyFusion&) = delete;
    OccupancyFusion& operator=(const OccupancyFusion&) = delete;
    const OccupancyFrameParams& params() const noexcept { return params_; }
    void set_params(const OccupancyFrameParams& params);
    OccupancyFrameResult integrate_camera(OccupancyLayer& layer, ImageView<float> depth,
        const PinholeCamera& camera, const RigidTransform& layer_from_camera,
        ImageView<std::uint8_t> active_mask = {});
    // Explicit empty selection performs no work, including allocation.
    OccupancyFrameResult integrate_camera_in_blocks(OccupancyLayer& layer, ImageView<float> depth,
        const PinholeCamera& camera, const RigidTransform& layer_from_camera,
        const std::vector<Index3D>& selected, ImageView<std::uint8_t> active_mask = {});
    OccupancyFrameResult integrate_lidar(OccupancyLayer& layer, ImageView<float> ranges,
        const Lidar& lidar, const RigidTransform& layer_from_sensor,
        ImageView<std::uint8_t> active_mask = {});

private:
    OccupancyFrameResult camera_impl(OccupancyLayer& layer, ImageView<float> depth,
        const PinholeCamera& camera, const RigidTransform& layer_from_camera,
        ImageView<std::uint8_t> active_mask, const std::vector<Index3D>* selected);
    struct Workspace;
    std::unique_ptr<Workspace> workspace_;
    OccupancyFrameParams params_;
};

} // namespace spacemit_mapping

#endif // SPACEMIT_MAPPING_FUSION_FRAMES_H_
