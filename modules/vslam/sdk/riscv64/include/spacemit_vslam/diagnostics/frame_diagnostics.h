// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_VSLAM_DIAGNOSTICS_FRAME_DIAGNOSTICS_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_VSLAM_DIAGNOSTICS_FRAME_DIAGNOSTICS_H_

#include "spacemit_vslam/diagnostics/pnp.h"

namespace spacemit_vslam::internal
{

// Expert/development diagnostics, not a pose-quality guarantee. The caller owns
// this storage until Track() returns. Track() clears it before validating input.
// Supported by Multicamera and Inertial; other modes leave the defaults intact.
struct FrameDiagnostics
{
    // Camera-0 observations delivered to the pose solver, not a count summed over eyes.
    uint32_t tracked_2d_count = 0;
    spacemit_vslam::visual_pnp::Diagnostics pnp;
    bool pnp_attempted = false;
    bool pnp_success = false;
    bool inertial_pnp = false;
    // True only when this solver frame falls back to IMU propagation after PnP failure.
    bool imu_prediction = false;
};

} // namespace spacemit_vslam::internal

#endif // SPACEMIT_VSLAM_DIAGNOSTICS_FRAME_DIAGNOSTICS_H_
