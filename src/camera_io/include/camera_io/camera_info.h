#ifndef CAMERA_INFO_H
#define CAMERA_INFO_H
#include <camera_sdk/kalibr_calibration.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>

#include <cstdint>

namespace camera_io
{
// Describes a resized raw pinhole image. Stereo extrinsics remain separate TF
// data; this function does not claim to rectify images or alter distortion.
sensor_msgs::msg::CameraInfo make_camera_info(const camera_sdk::KalibrCamera& camera,
    const std_msgs::msg::Header& header, std::uint32_t width, std::uint32_t height);
} // namespace camera_io

#endif // CAMERA_INFO_H
