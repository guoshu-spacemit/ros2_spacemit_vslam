// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_MAPPING_FUSION_APPEARANCE_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_MAPPING_FUSION_APPEARANCE_H_

#include "spacemit_mapping/core/image_view.h"
#include "spacemit_mapping/geometry/rigid_transform.h"
#include "spacemit_mapping/geometry/camera.h"
#include "spacemit_mapping/map_storage/layer.h"

namespace spacemit_mapping
{

struct DepthTraceParams
{
    int ray_subsampling = 4;
    int maximum_steps = 100;
    float maximum_distance_m = 7;
    float surface_epsilon_vox = 0.1F;
};

struct AppearanceParams
{
    float truncation_vox = 4;
    float maximum_distance_m = 7;
    float maximum_weight = 5;
    float measurement_weight = 0.8F;
    std::uint32_t ray_subsampling = 4;
    DepthTraceParams trace;
};

struct TracedDepth
{
    int width = 0, height = 0;
    std::vector<float> pixels; // camera Z depth in metres; -1 denotes no hit
    ImageView<float> view() const;
};

struct AppearanceStats
{
    std::size_t candidate_blocks = 0;
    std::size_t allocated_blocks = 0;
    std::size_t updated_blocks = 0;
    std::size_t updated_voxels = 0;
};

struct AppearanceResult
{
    AppearanceStats stats;
    // Candidate order matches traversal; includes allocated-but-unpainted blocks.
    std::vector<Index3D> candidate_blocks;
    // Only blocks with a painted voxel. Both lists own their data.
    std::vector<Index3D> updated_blocks;
};

// CPU appearance and visibility queries. No graphics device or library is used.
// Geometry is read-only; the caller owns the separate color layer and all input.
class ColorFusion
{
public:
    explicit ColorFusion(AppearanceParams params = {});
    const AppearanceParams& params() const noexcept { return params_; }
    void set_params(const AppearanceParams& params);
    // Depth may have a common integer width/height reduction from the camera.
    AppearanceResult integrate_rgbd(const TsdfLayer& geometry, ColorLayer& colors,
        ImageView<Rgb8> rgb, ImageView<float> depth, const PinholeCamera& camera,
        const RigidTransform& layer_from_camera, ImageView<std::uint8_t> active_mask = {}) const;
    AppearanceResult integrate_rgb(const TsdfLayer& geometry, ColorLayer& colors,
        ImageView<Rgb8> rgb, const PinholeCamera& camera, const RigidTransform& layer_from_camera,
        ImageView<std::uint8_t> active_mask = {}) const;
    TracedDepth trace_depth(const TsdfLayer& geometry, const PinholeCamera& camera,
        const RigidTransform& layer_from_camera) const;

private:
    AppearanceResult integrate(const TsdfLayer& geometry, ColorLayer& colors, ImageView<Rgb8> rgb,
        ImageView<float> depth, const PinholeCamera& camera,
        const RigidTransform& layer_from_camera, ImageView<std::uint8_t> mask,
        bool select_by_projection) const;
    AppearanceParams params_;
};

} // namespace spacemit_mapping

#endif // SPACEMIT_MAPPING_FUSION_APPEARANCE_H_
