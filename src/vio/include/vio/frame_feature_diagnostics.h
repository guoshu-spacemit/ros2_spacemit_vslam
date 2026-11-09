#ifndef FRAME_FEATURE_DIAGNOSTICS_H
#define FRAME_FEATURE_DIAGNOSTICS_H
#include <spacemit_vslam/api.h>
#include <spacemit_vslam/diagnostics/frame_parameters.h>
#include <common/msg/feature_point_array.hpp>

#include "vio/feature_track_history.h"
#include "vio/imu_fallback_history.h"

#include <cmath>
#include <unordered_set>
#include <vector>

namespace vio
{
// Read-only decoration of a completed Track result. No solver settings are changed.
class FrameFeatureDiagnostics
{
public:
    void reset()
    {
        feature_track_history_.reset();
        imu_fallback_history_.reset();
    }

    void populate(common::msg::FeaturePointArray& message, std::int64_t timestamp_ns,
        const std::vector<spacemit_vslam::Observation>& observations,
        const spacemit_vslam::internal::FrameDiagnostics& diagnostics, bool pose_valid)
    {
        // Invalid poses can include an internal reset. Do not inherit recycled IDs.
        if (!pose_valid)
            feature_track_history_.reset();
        feature_track_history_.begin_frame(timestamp_ns);
        message.diagnostics_available = true;
        message.tracked_2d_count = diagnostics.tracked_2d_count;
        message.pnp_3d_observation_count = diagnostics.pnp.observation_count;
        message.pnp_attempted = diagnostics.pnp_attempted;
        message.pnp_success = diagnostics.pnp_success;
        message.inertial_pnp = diagnostics.inertial_pnp;
        message.pose_valid = pose_valid;
        message.imu_prediction = pose_valid && diagnostics.imu_prediction;
        imu_fallback_history_.update(timestamp_ns, message.imu_prediction);
        message.imu_prediction_streak = imu_fallback_history_.count();
        message.imu_prediction_span_ms = imu_fallback_history_.span_ms();
        message.pnp_minimum_observations = diagnostics.pnp.minimum_observations;
        message.reprojection_evaluated = diagnostics.pnp.reprojection_evaluated;
        message.depth_valid_count = diagnostics.pnp.depth_valid_count;
        message.projected_count = diagnostics.pnp.projected_count;
        message.unique_landmark_count = diagnostics.pnp.unique_landmark_count;
        message.error_over_3px_count = diagnostics.pnp.error_over_3px_count;
        message.reprojection_rmse_px = diagnostics.pnp.reprojection_rmse_px;
        message.visual_weights_evaluated = diagnostics.pnp.visual_weights_evaluated;
        message.visual_weight_count = diagnostics.pnp.visual_weight_count;
        message.visual_weight_below_half_count = diagnostics.pnp.visual_weight_below_half_count;
        message.visual_weight_mean = diagnostics.pnp.visual_weight_mean;
        message.visual_weight_min = diagnostics.pnp.visual_weight_min;
        message.imu_information_scale = diagnostics.pnp.imu_information_scale;
        std::unordered_set<std::uint64_t> left_pnp_ids;
        for (const auto& selected : diagnostics.pnp.observations)
        {
            if (selected.camera_index == 0)
                left_pnp_ids.insert(selected.id);
        }
        message.points.reserve(observations.size());
        for (const auto& observation : observations)
        {
            if (observation.camera_index != 0 || !std::isfinite(observation.u) ||
                !std::isfinite(observation.v))
            {
                continue;
            }
            common::msg::FeaturePoint point;
            point.id = observation.id;
            point.x = observation.u;
            point.y = observation.v;
            point.pnp_input = left_pnp_ids.count(observation.id) != 0;
            const auto history = feature_track_history_.observe(observation.id);
            point.track_age_seconds = history.age_seconds;
            point.track_observation_count = history.count;
            message.points.push_back(point);
        }
        // An invalid frame may expose the old frontend's last observations.
        // They may be displayed, but must not age recycled IDs in the next frame.
        if (!pose_valid)
            feature_track_history_.reset();
    }

private:
    FeatureTrackHistory feature_track_history_;
    ImuFallbackHistory imu_fallback_history_;
};
} // namespace vio

#endif // FRAME_FEATURE_DIAGNOSTICS_H
