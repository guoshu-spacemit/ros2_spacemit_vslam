// Resolve IMU sampling frequency from immutable camera configuration.
// Preserve the namespace-qualified SDK guard across consumer include trees.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_CAMERA_CAMERA_SDK_IMU_FREQUENCY_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_CAMERA_CAMERA_SDK_IMU_FREQUENCY_H_

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <stdexcept>
#include <string>

namespace camera_sdk
{

// Describe the source sampling mode, not DDS delivery or consumer throughput.
// A fixed rate takes precedence for independent/fixed-rate IMUs and old profiles.
inline double imu_frequency_from_config(
    const boost::property_tree::ptree& root, int camera_fps_override = 0)
{
    if (!root.get<bool>("imu.enabled", false))
    {
        throw std::runtime_error("automatic IMU frequency requires imu.enabled=true");
    }
    double frequency;
    if (root.get_child_optional("imu.nominal_frequency_hz"))
    {
        frequency = root.get<double>("imu.nominal_frequency_hz");
    }
    else
    {
        const int samples = root.get<int>("imu.samples_per_frame");
        const int fps = camera_fps_override > 0 ? camera_fps_override : root.get<int>("camera.fps");
        if (samples <= 0 || samples > 5000 || fps <= 0)
        {
            throw std::runtime_error("IMU samples_per_frame and camera FPS must be positive");
        }
        frequency = static_cast<double>(samples) * fps;
    }
    if (!std::isfinite(frequency) || frequency <= 0.0 || frequency > 5000.0)
    {
        throw std::runtime_error("resolved IMU frequency must be in (0, 5000]");
    }
    return frequency;
}

inline double imu_frequency_from_file(const std::string& path, int camera_fps_override = 0)
{
    boost::property_tree::ptree root;
    boost::property_tree::read_json(path, root);
    return imu_frequency_from_config(root, camera_fps_override);
}

} // namespace camera_sdk

#endif // SPACEMIT_CAMERA_CAMERA_SDK_IMU_FREQUENCY_H_
