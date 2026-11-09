// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_VSLAM_DIAGNOSTICS_FRAME_PARAMETERS_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_VSLAM_DIAGNOSTICS_FRAME_PARAMETERS_H_

#include <cstdint>
#include <optional>
#include <string_view>

#include "spacemit_vslam/diagnostics/frame_diagnostics.h"

/**
 * @file frame_parameters.h
 *
 * The parameters in this file expose low-level solver knobs that are tuned for the default
 * hardware and dataset configurations shipped with spacemit_vslam. They are subject to change or
 * removal in any release without notice and are not covered by API stability guarantees.
 *
 * Do NOT adjust these values.
 *
 * Normal users should call Odometry::Track() without passing Internals — the defaults
 * are correct for virtually all use cases.
 */

namespace spacemit_vslam
{
namespace internal
{

/**
 * @brief Per-frame low-level solver parameters for Odometry::Track() (stateless).
 *
 * All fields carry carefully tuned defaults. Instantiate with Internals{} and
 * override only the fields you understand and have validated experimentally.
 * Each Track() call is independent; changes do not persist to the next call.
 */
struct Internals
{
    // Opt-in, synchronous output; never retained by the tracker or SBA worker.
    FrameDiagnostics* frame_diagnostics = nullptr;
    // ============================================================
    // Feature Selection Settings (from sof::Settings)
    // ============================================================

    /// Number of desired feature tracks. Default: 450
    int32_t num_desired_tracks = 450;

    /// Image border regions to ignore (in pixels). Default: 0
    int32_t border_top = 0;
    int32_t border_bottom = 0;
    int32_t border_left = 0;
    int32_t border_right = 0;

    /// Enable box filter preprocessing for this frame. Default: false
    bool box3_prefilter = false;

    /// Enable RANSAC filtering for this frame. Default: false
    bool ransac_filter = false;

    // ============================================================
    // Keyframe Settings (from KeyFrameSettings)
    // ============================================================

    /// Keyframe selection threshold: frame becomes keyframe if survivor tracks
    /// from last keyframe falls below this percentage (0-100). Default: 41.f
    float kf_survivor_from_last = 41.f;

    /// Maximum time delta between consecutive keyframes (in seconds). Default: 60
    int64_t kf_max_timedelta_between_kfs_s = 60;

    /// Optional keyframe decision. std::nullopt uses automatic keyframe selection (default),
    /// true forces a keyframe, and false forces a non-keyframe.
    std::optional<bool> kf_override_frame_selection;

    // ============================================================
    // PNP Solver Settings - visual PNP tracking (stereo / multi-camera modes)
    // ============================================================

    /// Levenberg-Marquardt damping factor. Default: 1e-3
    float vo_pnp_lambda = 1e-3f;

    /// Huber robustifier scale for reprojection residuals. Default: 2e-2
    float vo_pnp_huber = 2e-2f;

    /// Maximum LM solver iterations. Default: 13
    int32_t vo_pnp_max_iteration = 13;

    /// Whether to recompute the covariance matrix after a successful solve. Default: true
    bool vo_pnp_recalculate_cov = true;

    /// Whether to filter/sort observations to max_obs_per_camera oldest tracks first. Default: true
    bool vo_pnp_filter_new_observations = true;

    /// Maximum observations fed to the solver per camera. Default: 270
    int32_t vo_pnp_max_obs_per_camera = 270;

    /// Minimum z-depth for a landmark to be used (in camera frame). Default: 0.01
    float vo_pnp_point_z_thresh = 0.01f;

    /// Minimum number of observations required to attempt a solve. Default: 13
    int32_t vo_pnp_min_observations = 13;

    /// Absolute convergence threshold for the PnP solver.
    /// Solve succeeds when: (current_cost < vo_pnp_cost_thresh) || (current_cost < initial_cost).
    /// The OR means either condition alone is sufficient: a low final cost always passes, and
    /// any net improvement over the initial cost also passes regardless of its absolute value.
    /// Because of the OR, setting vo_pnp_cost_thresh very large (e.g. FLT_MAX) makes the
    /// absolute-threshold branch always true, rendering the relative-drop check irrelevant.
    /// Default: 0.6
    float vo_pnp_cost_thresh = 0.6f;

    // ============================================================
    // PNP Solver Settings - stereo fallback in inertial mode
    // Defaults match spacemit_vslam::visual_pnp::Settings::InertialSettings().
    // ============================================================

    /// Levenberg-Marquardt damping factor. Default: 1e-3
    float inertial_stereo_pnp_lambda = 1e-3f;

    /// Huber robustifier scale for reprojection residuals. Default: 0.1
    float inertial_stereo_pnp_huber = 0.1f;

    /// Maximum LM solver iterations. Default: 13
    int32_t inertial_stereo_pnp_max_iteration = 13;

    /// Whether to recompute the covariance matrix after a successful solve. Default: false
    bool inertial_stereo_pnp_recalculate_cov = false;

    /// Whether to filter/sort observations to max_obs_per_camera oldest tracks first. Default: true
    bool inertial_stereo_pnp_filter_new_observations = true;

    /// Maximum observations fed to the solver per camera. Default: 270
    int32_t inertial_stereo_pnp_max_obs_per_camera = 270;

    /// Minimum z-depth for a landmark to be used (in camera frame). Default: 0.01
    float inertial_stereo_pnp_point_z_thresh = 0.01f;

    /// Minimum number of observations required to attempt a solve. Default: 13
    int32_t inertial_stereo_pnp_min_observations = 13;

    /// Absolute convergence threshold for the inertial-mode stereo fallback PnP solver.
    /// Default: 0.6
    float inertial_stereo_pnp_cost_thresh = 0.6f;

    // ============================================================
    // Inertial PNP Solver Settings - IMU-fused tracking path
    // ============================================================

    /// Robustifier scale for inertial PNP reprojection residuals. Default: 0.4
    float imu_pnp_robustifier_scale = 0.4f;

    /// Maximum inertial PNP solver iterations. Default: 20
    int32_t imu_pnp_max_iteration = 20;

    /// Minimum number of observations required to attempt inertial PNP. Default: 25
    int32_t imu_pnp_min_observations = 25;

    /// Minimum camera-frame z-depth for IMU-fused PnP observations. Must be finite and positive.
    /// Default: 1.0 m.
    float imu_pnp_point_z_thresh = 1.0f;
};

// TODO(vikuznetsov): remove when https://gcc.gnu.org/bugzilla/show_bug.cgi?id=88165 is fixed
inline Internals GetDefaultInternals() { return Internals{}; }

/**
 * @brief Key/value pair for Odometry::ApplyPersistentInternalParameters.
 * @see Odometry::ApplyPersistentInternalParameters for the list of supported keys.
 */
struct InternalParameter
{
    std::string_view key;   ///< Parameter name (e.g. `sba.num_sba_iterations`).
    std::string_view value; ///< Parameter value as a string.
};

} // namespace internal
} // namespace spacemit_vslam

#endif // SPACEMIT_VSLAM_DIAGNOSTICS_FRAME_PARAMETERS_H_
