// Keep distinct guards for the SDK and ROS headers with this basename.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_VSLAM_CAMERA_IO_IMU_FREQUENCY_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_VSLAM_CAMERA_IO_IMU_FREQUENCY_H_
#include <camera_sdk/imu_frequency.h>

// Product-facing facade; implementation belongs to the native camera SDK.
namespace camera_io
{
using camera_sdk::imu_frequency_from_config;
using camera_sdk::imu_frequency_from_file;
} // namespace camera_io

#endif // SPACEMIT_VSLAM_CAMERA_IO_IMU_FREQUENCY_H_
