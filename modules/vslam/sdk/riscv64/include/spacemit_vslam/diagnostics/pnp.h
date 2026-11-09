// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_VSLAM_DIAGNOSTICS_PNP_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_VSLAM_DIAGNOSTICS_PNP_H_

#include <cstdint>
#include <limits>
#include <vector>

namespace spacemit_vslam::visual_pnp
{

// Read-only, opt-in snapshot of the observations offered to a PnP solve.
// These are solver inputs, not final inliers or a depth-validity guarantee.
struct Diagnostics
{
    struct Observation
    {
        uint64_t id;
        int32_t camera_index;
    };
    uint32_t observation_count = 0;
    uint32_t minimum_observations = 0;
    // Post-solve read-only projection statistics. Not robust-loss weights,
    // accepted inliers, observability, or a pose-accuracy guarantee.
    bool reprojection_evaluated = false;
    uint32_t depth_valid_count = 0;
    uint32_t projected_count = 0;
    uint32_t unique_landmark_count = 0;
    uint32_t error_over_3px_count = 0;
    float reprojection_rmse_px = std::numeric_limits<float>::quiet_NaN();
    // Inertial PnP only: recomputed at the accepted pose with the solver's
    // information matrices and Huber derivative. Normalized by the unrobust
    // derivative (0.5), so 1 means no robust attenuation, not visual/IMU share.
    bool visual_weights_evaluated = false;
    uint32_t visual_weight_count = 0;
    uint32_t visual_weight_below_half_count = 0;
    float visual_weight_mean = std::numeric_limits<float>::quiet_NaN();
    float visual_weight_min = std::numeric_limits<float>::quiet_NaN();
    // Configuration multiplier of the preintegration information matrices;
    // neither an information-matrix norm nor a fusion percentage.
    float imu_information_scale = std::numeric_limits<float>::quiet_NaN();
    std::vector<Observation> observations;
};

} // namespace spacemit_vslam::visual_pnp

#endif // SPACEMIT_VSLAM_DIAGNOSTICS_PNP_H_
