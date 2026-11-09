// Keep distinct guards for the SDK and ROS headers with this basename.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_VSLAM_CAMERA_IO_KALIBR_TIME_ALIGNMENT_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_VSLAM_CAMERA_IO_KALIBR_TIME_ALIGNMENT_H_
#include <camera_sdk/kalibr_time_alignment.h>

// Product-facing facade; implementation belongs to the native camera SDK.
namespace camera_io
{
using camera_sdk::KalibrTimeAlignment;
}

#endif // SPACEMIT_VSLAM_CAMERA_IO_KALIBR_TIME_ALIGNMENT_H_
