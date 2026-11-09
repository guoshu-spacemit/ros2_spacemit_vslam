// Keep distinct guards for the SDK and ROS headers with this basename.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_VSLAM_CAMERA_IO_MAHONY_AHRS_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_VSLAM_CAMERA_IO_MAHONY_AHRS_H_
#include <camera_sdk/mahony_ahrs.h>

// Product-facing facade; implementation belongs to the native camera SDK.
namespace camera_io
{
using camera_sdk::AhrsConfig;
using camera_sdk::MahonyAhrs;
using camera_sdk::Quaternion;
} // namespace camera_io

#endif // SPACEMIT_VSLAM_CAMERA_IO_MAHONY_AHRS_H_
