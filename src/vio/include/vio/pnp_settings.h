#ifndef PNP_SETTINGS_H
#define PNP_SETTINGS_H
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

#include <spacemit_vslam/diagnostics/frame_parameters.h>

namespace vio
{

// Opt-in camera-z gate shared by visual and IMU-fused PnP. Triangulation,
// feature tracking and map integration are unchanged.
inline std::optional<spacemit_vslam::internal::Internals> make_pnp_settings(double min_depth_m)
{
    if (!std::isfinite(min_depth_m) || min_depth_m < 0.0 ||
        min_depth_m > std::numeric_limits<float>::max())
    {
        throw std::invalid_argument(
            "pnp_min_depth_m must be finite, non-negative and fit in float");
    }
    if (min_depth_m == 0.0)
    {
        return std::nullopt;
    }
    const float threshold = static_cast<float>(min_depth_m);
    if (!(threshold > 0.0F))
    {
        throw std::invalid_argument(
            "pnp_min_depth_m is too small for the spacemit_vslam threshold");
    }
    auto settings = spacemit_vslam::internal::GetDefaultInternals();
    settings.vo_pnp_point_z_thresh = threshold;
    settings.inertial_stereo_pnp_point_z_thresh = threshold;
    settings.imu_pnp_point_z_thresh = threshold;
    return settings;
}

} // namespace vio

#endif // PNP_SETTINGS_H
