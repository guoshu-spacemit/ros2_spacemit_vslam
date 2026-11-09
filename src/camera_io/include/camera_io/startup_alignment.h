// Keep distinct guards for the SDK and ROS headers with this basename.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_VSLAM_CAMERA_IO_STARTUP_ALIGNMENT_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_VSLAM_CAMERA_IO_STARTUP_ALIGNMENT_H_
#include <camera_sdk/startup_alignment.h>

// Product-facing facade; implementation belongs to the native camera SDK.
namespace camera_io
{
using camera_sdk::ExposureControl;
using camera_sdk::open_v4l2_exposure;
using camera_sdk::StartupAlignment;
} // namespace camera_io

#endif // SPACEMIT_VSLAM_CAMERA_IO_STARTUP_ALIGNMENT_H_
