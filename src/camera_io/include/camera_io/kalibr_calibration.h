// Keep distinct guards for the SDK and ROS headers with this basename.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_VSLAM_CAMERA_IO_KALIBR_CALIBRATION_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_VSLAM_CAMERA_IO_KALIBR_CALIBRATION_H_
#include <camera_sdk/kalibr_calibration.h>

// Product-facing facade; implementation belongs to the native camera SDK.
namespace camera_io
{
using camera_sdk::KalibrCalibration;
using camera_sdk::KalibrCamera;
using camera_sdk::load_kalibr_calibration;
using camera_sdk::Transform4d;
} // namespace camera_io

#endif // SPACEMIT_VSLAM_CAMERA_IO_KALIBR_CALIBRATION_H_
