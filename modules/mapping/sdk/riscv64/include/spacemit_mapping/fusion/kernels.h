// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_MAPPING_FUSION_KERNELS_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_MAPPING_FUSION_KERNELS_H_

#include "spacemit_mapping/core/types.h"

namespace spacemit_mapping
{

enum class Weighting
{
    kConstant,
    kConstantDropoff,
    kInverseSquare,
    kInverseSquareDropoff,
    kDistancePenalty,
    kLinearWithMax,
};

struct TsdfKernelParams
{
    float truncation_m = 0.2F;
    float maximum_weight = 5;
    Weighting weighting = Weighting::kInverseSquare;
    float reset_discrepancy_m = -1;
    float reset_minimum_weight = 2;
    float invalid_depth_weight_factor = -1;
};

struct TsdfKernelCounts
{
    std::size_t updated = 0;
    std::size_t reset = 0;
    std::size_t invalid_decayed = 0;
};

// Borrowed arrays, all count elements. Mutable arrays must be disjoint from all
// other arrays. Read-only arrays may alias each other. No ownership is retained.
struct TsdfBatch
{
    float* distance_m = nullptr;
    float* weight = nullptr;
    const float* surface_depth_m = nullptr;
    const float* voxel_depth_m = nullptr;
    const std::uint32_t* projected = nullptr;
    const std::uint32_t* active = nullptr;
    std::size_t count = 0;
};

struct SdfBatch
{
    float* distance_m = nullptr;
    float* weight = nullptr;
    const float* measured_sdf_m = nullptr;
    const std::uint32_t* valid = nullptr;
    std::size_t count = 0;
};

struct TsdfDecayBatch
{
    float* distance_m = nullptr;
    float* weight = nullptr;
    const std::uint32_t* active = nullptr;
    std::size_t count = 0;
};

struct TsdfDecayKernelParams
{
    float factor = 0.95F;
    float weight_threshold = 1.0e-3F;
    bool set_free_distance = false;
    float free_distance_m = 0.2F;
};

struct OccupancyKernelParams
{
    float free_probability = 0.3F;
    float occupied_probability = 0.7F;
    float unobserved_probability = 0.5F;
    float surface_half_width_m = 0.1F;
};

struct OccupancyBatch
{
    float* log_odds = nullptr;
    const float* surface_depth_m = nullptr;
    const float* voxel_depth_m = nullptr;
    const std::uint32_t* active = nullptr;
    std::size_t count = 0;
};

struct OccupancyDecayKernelParams
{
    float free_probability = 0.55F;
    float occupied_probability = 0.4F;
    float target_probability = 0.5F;
};

bool fusion_rvv_compiled() noexcept;
float log_odds_from_probability(float probability);
float probability_from_log_odds(float log_odds);

std::size_t fuse_sdf(
    SdfBatch batch, float truncation_m, float maximum_weight, Backend backend = Backend::kAuto);
TsdfKernelCounts fuse_tsdf(
    TsdfBatch batch, const TsdfKernelParams& params = {}, Backend backend = Backend::kAuto);
void decay_tsdf(TsdfDecayBatch batch, const TsdfDecayKernelParams& params = {},
    Backend backend = Backend::kAuto);
std::size_t fuse_occupancy(OccupancyBatch batch, const OccupancyKernelParams& params = {},
    Backend backend = Backend::kAuto);
// Counts target-crossing predicates, including values already at the target.
std::size_t decay_occupancy(float* log_odds, std::size_t count,
    const OccupancyDecayKernelParams& params = {}, Backend backend = Backend::kAuto);

} // namespace spacemit_mapping

#endif // SPACEMIT_MAPPING_FUSION_KERNELS_H_
