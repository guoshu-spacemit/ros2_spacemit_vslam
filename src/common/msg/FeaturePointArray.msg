# Feature observations exported by the stereo VO front end.
std_msgs/Header header
uint32 source_frame_index
uint32 image_width
uint32 image_height
# False for ordinary feature export or the legacy PnP-only topic. Do not
# interpret unavailable counters/flags as a tracking failure.
bool diagnostics_available
# Diagnostics refer to the same source frame. PnP count is the number of
# matched 3D observations collected by the selected PnP solver, before
# minimum-count checks and outlier rejection. Zero can also mean PnP was not
# reached, so use pnp_attempted to distinguish that case.
uint32 tracked_2d_count
uint32 pnp_3d_observation_count
bool pnp_attempted
bool pnp_success
bool inertial_pnp
# Exact current Track outcome, not an inferred quality category.
bool pose_valid
bool imu_prediction
uint32 imu_prediction_streak
float32 imu_prediction_span_ms
# Post-success reprojection, evaluated before SBA/map update. Counts include
# all solver cameras. 3px is a diagnostic reference, NOT the rejection gate.
uint32 pnp_minimum_observations
bool reprojection_evaluated
uint32 depth_valid_count
uint32 projected_count
uint32 unique_landmark_count
uint32 error_over_3px_count
float32 reprojection_rmse_px
# Inertial PnP only, at the accepted pose. Huber derivative normalized to 1
# when unattenuated, using the actual observation information matrices.
# This is neither an inlier probability nor the visual/IMU fusion fraction.
bool visual_weights_evaluated
uint32 visual_weight_count
uint32 visual_weight_below_half_count
float32 visual_weight_mean
float32 visual_weight_min
# Configured multiplier of preintegration information; NOT total IMU weight.
float32 imu_information_scale
common/FeaturePoint[] points
