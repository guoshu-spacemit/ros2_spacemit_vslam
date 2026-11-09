// Keep distinct guards for the SDK and ROS headers with this basename.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_VSLAM_CAMERA_IO_CAMERA_CLOCK_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_VSLAM_CAMERA_IO_CAMERA_CLOCK_H_
#include <camera_sdk/camera_clock.h>

// Product-facing facade; implementation belongs to the native camera SDK.
namespace camera_io
{
using camera_sdk::CameraClock;
}

#endif // SPACEMIT_VSLAM_CAMERA_IO_CAMERA_CLOCK_H_
