// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_MAPPING_DISTANCE_FIELD_FIELD_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_MAPPING_DISTANCE_FIELD_FIELD_H_

#include "spacemit_mapping/geometry/plane.h"
#include "spacemit_mapping/map_storage/layer.h"

#include <optional>

namespace spacemit_mapping
{

enum class UnknownSpace
{
    kUnknown,
    kFree,
    kOccupied
};

struct DistanceFieldParams
{
    float maximum_distance_m = 2;
    float minimum_tsdf_weight = 1.0e-4F;
    float maximum_site_distance_vox = 1;
    float truncation_vox = 4;
    float occupied_probability = 0.5F;
    bool negative_band_sites = false;
    UnknownSpace unknown = UnknownSpace::kUnknown;
    Backend backend = Backend::kAuto;
};

struct HeightBand
{
    float minimum_m = 0, maximum_m = 1, output_m = 1;
};

struct PlaneBand
{
    Plane ground;
    float height_above_m = 0, thickness_m = 0.1F, output_m = 1;
};

struct DistanceFieldStats
{
    std::size_t input_blocks = 0, allocated_blocks = 0, site_blocks = 0, sites = 0;
    std::size_t cleared_voxels = 0, propagated_blocks = 0;
    std::size_t clearing_seed_blocks = 0, clearing_candidate_blocks = 0;
    std::size_t invalidated_site_voxels = 0, propagation_seed_blocks = 0;
};

struct DistanceUpdate
{
    DistanceFieldStats stats;
    std::vector<Index3D> updated_blocks; // owning, first notification order
};

struct DistanceFieldState
{
    float voxel_size_m = 0;
    DistanceFieldParams params;
    std::optional<float> slice_output_m;
    std::vector<BlockUpdate<EsdfBlock>> volume, slice;
};

bool distance_field_rvv_compiled() noexcept;

// Owns its 3D and slice layers. Sources are borrowed only during the call.
// Calls and direct layer mutation require caller synchronization.
class DistanceField
{
public:
    explicit DistanceField(float voxel_size_m, DistanceFieldParams params = {});
    const DistanceFieldParams& params() const noexcept { return params_; }
    void set_params(const DistanceFieldParams& params);
    float voxel_size() const noexcept { return layer_.voxel_size(); }
    EsdfLayer& layer() noexcept { return layer_; }
    const EsdfLayer& layer() const noexcept { return layer_; }
    EsdfLayer& slice_layer() noexcept { return slice_; }
    const EsdfLayer& slice_layer() const noexcept { return slice_; }
    void clear() noexcept;
    DistanceFieldState snapshot() const;
    // Validates into temporary owned layers; failure leaves this field intact.
    void restore(const DistanceFieldState& state);

    DistanceUpdate update(const TsdfLayer& input, const std::vector<Index3D>& indices,
        const FreespaceLayer* freespace = nullptr);
    DistanceUpdate update(const OccupancyLayer& input, const std::vector<Index3D>& indices);
    DistanceUpdate update_all(const TsdfLayer& input, const FreespaceLayer* freespace = nullptr)
    {
        return update(input, input.indices(), freespace);
    }
    DistanceUpdate update_all(const OccupancyLayer& input)
    {
        return update(input, input.indices());
    }
    DistanceUpdate update_slice(const TsdfLayer& input, const std::vector<Index3D>& indices,
        HeightBand band, const FreespaceLayer* freespace = nullptr);
    DistanceUpdate update_slice(const TsdfLayer& input, const std::vector<Index3D>& indices,
        const PlaneBand& band, const FreespaceLayer* freespace = nullptr);
    DistanceUpdate update_slice(
        const OccupancyLayer& input, const std::vector<Index3D>& indices, HeightBand band);
    DistanceUpdate update_slice(
        const OccupancyLayer& input, const std::vector<Index3D>& indices, const PlaneBand& band);
    DistanceUpdate update_slice_all(
        const TsdfLayer& input, HeightBand band, const FreespaceLayer* freespace = nullptr)
    {
        return update_slice(input, input.indices(), band, freespace);
    }
    DistanceUpdate update_slice_all(
        const TsdfLayer& input, const PlaneBand& band, const FreespaceLayer* freespace = nullptr)
    {
        return update_slice(input, input.indices(), band, freespace);
    }
    DistanceUpdate update_slice_all(const OccupancyLayer& input, HeightBand band)
    {
        return update_slice(input, input.indices(), band);
    }
    DistanceUpdate update_slice_all(const OccupancyLayer& input, const PlaneBand& band)
    {
        return update_slice(input, input.indices(), band);
    }

    // Query the containing voxel, without interpolation. False leaves output intact.
    bool distance_at(Vector3f position_m, float* signed_distance_m) const;
    bool slice_distance_at(float x_m, float y_m, float* signed_distance_m) const;

private:
    EsdfLayer layer_, slice_;
    DistanceFieldParams params_;
    float maximum_squared_vox_ = 0;
    std::optional<float> slice_output_m_;
};

} // namespace spacemit_mapping

#endif // SPACEMIT_MAPPING_DISTANCE_FIELD_FIELD_H_
