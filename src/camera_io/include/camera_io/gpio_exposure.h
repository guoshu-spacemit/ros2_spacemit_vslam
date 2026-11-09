// Keep distinct guards for the SDK and ROS headers with this basename.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_VSLAM_CAMERA_IO_GPIO_EXPOSURE_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_VSLAM_CAMERA_IO_GPIO_EXPOSURE_H_
#include "camera_io/exposure_matcher.h"
#include <camera_sdk/gpio_exposure.h>

// Product-facing facade; implementation belongs to the native camera SDK.
namespace camera_io
{
using camera_sdk::GpioExposure;
}

#endif // SPACEMIT_VSLAM_CAMERA_IO_GPIO_EXPOSURE_H_
