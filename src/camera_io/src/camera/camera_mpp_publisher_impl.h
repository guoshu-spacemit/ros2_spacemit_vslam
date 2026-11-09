#ifndef CAMERA_MPP_PUBLISHER_IMPL_H
#define CAMERA_MPP_PUBLISHER_IMPL_H
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <filesystem> // NOLINT(build/c++17): This target requires C++17.
#include <functional>
#include <iterator>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include "Imu.h"
#include "ImuPublisher.h"
#include "camera_io/camera_clock.h"
#include "camera_io/gpio_exposure.h"
#include "camera_io/imu_frequency.h"
#include "camera_io/kalibr_calibration.h"
#include "camera_io/mahony_ahrs.h"
#include "camera_io/mjpeg_packet_reader.h"
#include "camera_io/startup_alignment.h"
#include "camera_sdk/embedded_metadata.h"
#include "decoder_input_pool.h"
#include "imu_message.h"
#include "mpp_stream_compat.h"
#include "demux/demux_api.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "mpp_msgs/msg/mpp_video_frame_desc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "common/lifecycle/async_lifecycle_node.h"

extern "C"
{
#include "sys_api.h"
#include "uvc_api.h"
#include "v2d/v2d_api.h"
#include "vb_api.h"
#include "vdec/vdec_api.h"
}

#ifdef SUCCESS
#undef SUCCESS
#endif
#ifdef FAILURE
#undef FAILURE
#endif

namespace camera_io::detail
{

constexpr int kDemuxChn = 0;

struct CropConfig
{
    int x{0};
    int y{0};
    int width{0};
    int height{0};
};

struct StreamConfig
{
    bool enabled{true};
    std::string topic;
    std::string camera_info_topic;
    std::string frame_id;
    CropConfig crop;
    int output_width{0};
    int output_height{0};
};

struct BridgeConfig
{
    std::string input_type{"uvc"};
    std::string input_file;
    std::string device;
    int width{0};
    int height{0};
    int fps{0};
    std::string input_format;
    std::string decoded_format;
    bool decode_enabled{true};
    std::string timestamp_source{"ros_clock"};
    bool gpio_sync_enabled{false}, gpio_active_low{false};
    std::string gpio_chip;
    unsigned gpio_line{0};
    camera_io::ExposureMatcher::Config gpio_match;
    camera_io::StartupAlignment::Config startup_alignment;
    std::string calibration_file;
    bool publish_imu_calibration_tf{true};
    bool metadata_enabled{false};
    std::string metadata_parser;
    CropConfig metadata_region;
    StreamConfig left;
    StreamConfig right;
    StreamConfig full;
    bool full_enabled{false};
    bool full_only{false};
    StreamConfig compressed;
    bool compressed_enabled{false};
    bool imu_enabled{false};
    std::string imu_raw_dds_topic;
    std::string imu_raw_ros_topic;
    bool ahrs_enabled{false};
    std::string imu_fused_dds_topic;
    std::string imu_fused_ros_topic;
    std::string imu_frame_id;
    int domain_id{0};
    bool domain_id_from_environment{false};
    int imu_queue_depth{16};
    float imu_nominal_frequency_hz{0.0F};
    camera_io::AhrsConfig ahrs_config;
    float orientation_variance{0.0025F};
    int max_exported_pairs{6};
    int decoder_buffer_count{0};
    camera_sdk::DecoderInputPoolConfig decoder_input_pool;
};

std::optional<int> ros_domain_id_from_environment()
{
    const char* value = std::getenv("ROS_DOMAIN_ID");
    if (value == nullptr || value[0] == '\0')
    {
        return std::nullopt;
    }

    char* end = nullptr;
    errno = 0;
    const long parsed = std::strtol(value, &end, 10);
    if (errno != 0 || end == value || *end != '\0' || parsed < 0 || parsed > 232)
    {
        throw std::runtime_error("ROS_DOMAIN_ID must be an integer in the range [0, 232]");
    }
    return static_cast<int>(parsed);
}

CropConfig load_crop(const boost::property_tree::ptree& root, const std::string& path)
{
    CropConfig crop;
    crop.x = root.get<int>(path + ".x");
    crop.y = root.get<int>(path + ".y");
    crop.width = root.get<int>(path + ".width");
    crop.height = root.get<int>(path + ".height");
    return crop;
}

StreamConfig load_stream(const boost::property_tree::ptree& root, const std::string& path)
{
    StreamConfig stream;
    stream.topic = root.get<std::string>(path + ".topic");
    stream.camera_info_topic = root.get<std::string>(path + ".camera_info_topic", "");
    stream.frame_id = root.get<std::string>(path + ".frame_id");
    stream.crop = load_crop(root, path + ".crop");
    stream.output_width = root.get<int>(path + ".output.width", stream.crop.width);
    stream.output_height = root.get<int>(path + ".output.height", stream.crop.height);
    return stream;
}

BridgeConfig load_config(const std::string& path, int fps_override = 0)
{
    if (fps_override < 0)
    {
        throw std::runtime_error("fps must be zero (JSON) or positive");
    }
    boost::property_tree::ptree root;
    boost::property_tree::read_json(path, root);

    BridgeConfig config;
    config.input_type = root.get<std::string>("input.type", "uvc");
    config.input_file = root.get<std::string>("input.file", "");
    if (!config.input_file.empty())
    {
        std::filesystem::path input_path(config.input_file);
        if (input_path.is_relative())
        {
            input_path = std::filesystem::path(path).parent_path() / input_path;
        }
        config.input_file = std::filesystem::weakly_canonical(input_path).string();
    }
    config.device = root.get<std::string>("camera.device", "");
    config.width = root.get<int>("camera.width");
    config.height = root.get<int>("camera.height");
    config.fps = fps_override > 0 ? fps_override : root.get<int>("camera.fps");
    config.input_format = root.get<std::string>("camera.input_format");
    config.decoded_format = root.get<std::string>("camera.decoded_format");
    config.decode_enabled = root.get<bool>("camera.decode", true);
    config.timestamp_source = root.get<std::string>("timestamp.source", "ros_clock");
    config.gpio_sync_enabled = root.get<bool>("timestamp.gpio.enabled", false);
    config.startup_alignment.enabled =
        root.get<bool>("timestamp.gpio.startup_alignment.enabled", false);
    if (config.startup_alignment.enabled)
    {
        if (!config.gpio_sync_enabled)
            throw std::runtime_error("startup_alignment requires timestamp.gpio.enabled");
        const auto& options = root.get_child("timestamp.gpio.startup_alignment");
        config.startup_alignment.timeout_ms = options.get<int>("timeout_ms", 10000);
        config.startup_alignment.frames_per_step = options.get<unsigned>("frames_per_step", 4);
        config.startup_alignment.tolerance_us = options.get<int>("tolerance_us", 100);
        if (!options.get<bool>("restore_exposure", true))
            throw std::runtime_error("startup_alignment must restore the original exposure");
        if (auto sequence = options.get_child_optional("exposure_us"))
        {
            config.startup_alignment.exposure_us.clear();
            for (const auto& entry : *sequence)
                config.startup_alignment.exposure_us.push_back(entry.second.get_value<int>());
        }
        camera_io::StartupAlignment::validate(config.startup_alignment);
    }
    if (config.gpio_sync_enabled)
    {
        auto duration = [&](const char* key, std::int64_t fallback, std::int64_t scale)
        {
            const auto value = root.get<std::int64_t>(key, fallback);
            if (value <= 0 || value > 1000000)
                throw std::runtime_error(std::string("invalid GPIO duration: ") + key);
            return value * scale;
        };
        config.gpio_chip = root.get<std::string>("timestamp.gpio.chip");
        config.gpio_line = root.get<unsigned>("timestamp.gpio.line");
        config.gpio_active_low = root.get<bool>("timestamp.gpio.active_low", false);
        config.gpio_match.window = root.get<unsigned>("timestamp.gpio.window", 32);
        config.gpio_match.capacity = root.get<unsigned>("timestamp.gpio.capacity", 256);
        config.gpio_match.width_tolerance_ns =
            duration("timestamp.gpio.width_tolerance_us", 100, 1000);
        config.gpio_match.interval_tolerance_ns =
            duration("timestamp.gpio.interval_tolerance_us", 200, 1000);
        config.gpio_match.max_age_ns = duration("timestamp.gpio.max_age_ms", 500, 1000000);
        config.gpio_match.diversity_ns = duration("timestamp.gpio.diversity_us", 500, 1000);
        (void)camera_io::ExposureMatcher(config.gpio_match);
    }
    const std::string calibration_format = root.get<std::string>("calibration.format", "");
    if (!calibration_format.empty() && calibration_format != "kalibr")
    {
        throw std::runtime_error("unsupported calibration format");
    }
    config.calibration_file = root.get<std::string>("calibration.file", "");
    config.publish_imu_calibration_tf = root.get<bool>("calibration.publish_imu_tf", true);
    if (!config.calibration_file.empty())
    {
        std::filesystem::path calibration_path(config.calibration_file);
        if (calibration_path.is_relative())
        {
            calibration_path = std::filesystem::path(path).parent_path() / calibration_path;
        }
        config.calibration_file = std::filesystem::weakly_canonical(calibration_path).string();
    }
    config.metadata_enabled = root.get<bool>("metadata.enabled", false);
    if (config.metadata_enabled)
    {
        config.metadata_parser = root.get<std::string>("metadata.parser");
        config.metadata_region = load_crop(root, "metadata.region");
    }
    config.full_only = root.get<bool>("streams.full_only", false);
    if (root.get_child_optional("streams.left"))
    {
        config.left = load_stream(root, "streams.left");
    }
    if (root.get_child_optional("streams.right"))
    {
        config.right = load_stream(root, "streams.right");
    }
    if (root.get_child_optional("streams.full"))
    {
        config.full = load_stream(root, "streams.full");
        config.full_enabled = true;
    }
    if (root.get_child_optional("streams.compressed"))
    {
        const auto& stream = root.get_child("streams.compressed");
        config.compressed.topic = stream.get<std::string>("topic");
        config.compressed.frame_id = stream.get<std::string>("frame_id");
        config.compressed_enabled = stream.get<bool>("enabled", true);
    }
    config.imu_enabled = root.get<bool>("imu.enabled", false);
    if (config.imu_enabled)
    {
        config.imu_raw_dds_topic = root.get<std::string>("imu.raw.dds_topic");
        config.imu_raw_ros_topic = root.get<std::string>("imu.raw.ros_topic");
        config.imu_frame_id = root.get<std::string>("imu.frame_id");
        config.domain_id = root.get<int>("imu.domain_id", 0);
        if (const auto environment_domain = ros_domain_id_from_environment())
        {
            config.domain_id = *environment_domain;
            config.domain_id_from_environment = true;
        }
        config.imu_queue_depth = root.get<int>("imu.queue_depth", 16);
        config.imu_nominal_frequency_hz =
            static_cast<float>(imu_frequency_from_config(root, config.fps));
        config.ahrs_enabled = root.get<bool>("imu.ahrs.enabled", false);
        if (config.ahrs_enabled)
        {
            config.imu_fused_dds_topic = root.get<std::string>("imu.ahrs.dds_topic");
            config.imu_fused_ros_topic = root.get<std::string>("imu.ahrs.ros_topic");
            config.ahrs_config.kp = root.get<float>("imu.ahrs.kp", 1.0F);
            config.ahrs_config.ki = root.get<float>("imu.ahrs.ki", 0.0F);
            config.ahrs_config.accel_rejection_m_s2 =
                root.get<float>("imu.ahrs.accel_rejection_m_s2", 3.0F);
            config.ahrs_config.max_dt_seconds =
                root.get<float>("imu.ahrs.max_dt_ms", 50.0F) * 1.0e-3F;
            const float orientation_stddev =
                root.get<float>("imu.ahrs.orientation_stddev_rad", 0.05F);
            config.orientation_variance = orientation_stddev * orientation_stddev;
        }
    }
    config.max_exported_pairs = root.get<int>("buffers.max_exported_pairs", 6);
    config.decoder_buffer_count = root.get<int>("buffers.decoder_buffer_count", 0);
    config.decoder_input_pool = load_decoder_input_pool(root);
    return config;
}

bool region_is_valid(const CropConfig& region, int image_width, int image_height)
{
    return region.x >= 0 && region.y >= 0 && region.width > 0 && region.height > 0 &&
        region.x + region.width <= image_width && region.y + region.height <= image_height;
}

camera_io::Transform4d inverse_rigid_transform(const camera_io::Transform4d& transform)
{
    camera_io::Transform4d inverse{transform[0], transform[4], transform[8], 0.0, transform[1],
        transform[5], transform[9], 0.0, transform[2], transform[6], transform[10], 0.0, 0.0, 0.0,
        0.0, 1.0};
    for (int row = 0; row < 3; ++row)
    {
        inverse[row * 4 + 3] = -(inverse[row * 4] * transform[3] +
            inverse[row * 4 + 1] * transform[7] + inverse[row * 4 + 2] * transform[11]);
    }
    return inverse;
}

geometry_msgs::msg::Transform transform_message(const camera_io::Transform4d& matrix)
{
    geometry_msgs::msg::Transform message;
    message.translation.x = matrix[3];
    message.translation.y = matrix[7];
    message.translation.z = matrix[11];

    const double trace = matrix[0] + matrix[5] + matrix[10];
    double qw;
    double qx;
    double qy;
    double qz;
    if (trace > 0.0)
    {
        const double scale = std::sqrt(trace + 1.0) * 2.0;
        qw = 0.25 * scale;
        qx = (matrix[9] - matrix[6]) / scale;
        qy = (matrix[2] - matrix[8]) / scale;
        qz = (matrix[4] - matrix[1]) / scale;
    }
    else if (matrix[0] > matrix[5] && matrix[0] > matrix[10])
    {
        const double scale = std::sqrt(1.0 + matrix[0] - matrix[5] - matrix[10]) * 2.0;
        qw = (matrix[9] - matrix[6]) / scale;
        qx = 0.25 * scale;
        qy = (matrix[1] + matrix[4]) / scale;
        qz = (matrix[2] + matrix[8]) / scale;
    }
    else if (matrix[5] > matrix[10])
    {
        const double scale = std::sqrt(1.0 + matrix[5] - matrix[0] - matrix[10]) * 2.0;
        qw = (matrix[2] - matrix[8]) / scale;
        qx = (matrix[1] + matrix[4]) / scale;
        qy = 0.25 * scale;
        qz = (matrix[6] + matrix[9]) / scale;
    }
    else
    {
        const double scale = std::sqrt(1.0 + matrix[10] - matrix[0] - matrix[5]) * 2.0;
        qw = (matrix[4] - matrix[1]) / scale;
        qx = (matrix[2] + matrix[8]) / scale;
        qy = (matrix[6] + matrix[9]) / scale;
        qz = 0.25 * scale;
    }
    message.rotation.w = qw;
    message.rotation.x = qx;
    message.rotation.y = qy;
    message.rotation.z = qz;
    return message;
}

VideoFrameInfo make_nv12_frame(UL pool, UL buffer, std::uint32_t width, std::uint32_t height,
    std::uint32_t frame_index, std::uint64_t pts)
{
    VideoFrameInfo frame{};
    void* address = nullptr;
    S32 fd = -1;
    U64 physical = 0;
    if (VB_GetVirAddr(buffer, &address) != 0 || address == nullptr ||
        VB_GetDmaBufFd(buffer, &fd) != 0 || fd < 0 || VB_GetPhyAddr(buffer, &physical) != 0)
    {
        throw std::runtime_error("failed to resolve an eye VB buffer");
    }

    const U32 y_size = width * height;
    frame.eFrameType = FRAME_TYPE_COMMON;
    frame.eModId = MPP_ID_V2D;
    frame.ulPoolId = pool;
    frame.ulBufferId = buffer;
    frame.u32Idx = frame_index;
    frame.stCommFrameInfo.u32Width = width;
    frame.stCommFrameInfo.u32Height = height;
    frame.stCommFrameInfo.u32Align = 1;
    frame.stCommFrameInfo.ePixelFormat = MPP_PIXEL_FORMAT_NV12;
    frame.stVFrame.u64PTS = pts;
    frame.stVFrame.u32PlaneNum = 2;
    frame.stVFrame.u32TotalSize = y_size + y_size / 2U;

    frame.stVFrame.u32PlaneStride[0] = width;
    frame.stVFrame.u32PlaneSize[0] = y_size;
    frame.stVFrame.u32PlaneSizeValid[0] = y_size;
    frame.stVFrame.u32Fd[0] = static_cast<UL>(fd);
    frame.stVFrame.u64PlanePhyAddr[0] = physical;
    frame.stVFrame.ulPlaneVirAddr[0] = reinterpret_cast<UL>(address);

    frame.stVFrame.u32PlaneStride[1] = width;
    frame.stVFrame.u32PlaneSize[1] = y_size / 2U;
    frame.stVFrame.u32PlaneSizeValid[1] = y_size / 2U;
    frame.stVFrame.u32Fd[1] = static_cast<UL>(fd);
    frame.stVFrame.u64PlanePhyAddr[1] = physical + y_size;
    frame.stVFrame.ulPlaneVirAddr[1] = reinterpret_cast<UL>(address) + y_size;
    return frame;
}

VideoFrameInfo normalize_decoded_nv12_frame(const VideoFrameInfo& decoded,
    std::uint32_t fallback_width, std::uint32_t fallback_height, std::uint32_t frame_index)
{
    VideoFrameInfo frame = decoded;
    const auto decoded_width = decoded.stCommFrameInfo.u32Width != 0
        ? decoded.stCommFrameInfo.u32Width
        : decoded.stVdecFrameInfo.stCommFrameInfo.u32Width;
    const auto decoded_height = decoded.stCommFrameInfo.u32Height != 0
        ? decoded.stCommFrameInfo.u32Height
        : decoded.stVdecFrameInfo.stCommFrameInfo.u32Height;
    const U32 width = decoded_width != 0 ? decoded_width : fallback_width;
    const U32 height = decoded_height != 0 ? decoded_height : fallback_height;
    if (width == 0 || height == 0)
    {
        throw std::runtime_error("decoded NV12 frame has no valid dimensions");
    }

    frame.u32Idx = frame_index;
    frame.stCommFrameInfo.u32Width = width;
    frame.stCommFrameInfo.u32Height = height;
    frame.stCommFrameInfo.u32Align =
        decoded.stCommFrameInfo.u32Align != 0 ? decoded.stCommFrameInfo.u32Align : 1;
    frame.stCommFrameInfo.ePixelFormat = MPP_PIXEL_FORMAT_NV12;
    frame.stVFrame.u32PlaneNum = std::max<U32>(decoded.stVFrame.u32PlaneNum, 2U);

    U32 y_stride = decoded.stVFrame.u32PlaneStride[0];
    if (y_stride < width && decoded.stVFrame.u32PlaneSize[0] >= width * height &&
        decoded.stVFrame.u32PlaneSize[0] % height == 0)
    {
        y_stride = decoded.stVFrame.u32PlaneSize[0] / height;
    }
    if (y_stride < width)
    {
        y_stride = width;
    }
    const U32 chroma_height = (height + 1U) / 2U;
    U32 uv_stride = decoded.stVFrame.u32PlaneStride[1];
    if (uv_stride < width && chroma_height != 0 &&
        decoded.stVFrame.u32PlaneSize[1] >= width * chroma_height &&
        decoded.stVFrame.u32PlaneSize[1] % chroma_height == 0)
    {
        uv_stride = decoded.stVFrame.u32PlaneSize[1] / chroma_height;
    }
    if (uv_stride < width)
    {
        uv_stride = y_stride;
    }
    frame.stVFrame.u32PlaneStride[0] = y_stride;
    frame.stVFrame.u32PlaneStride[1] = uv_stride;

    const U32 minimum_y_size = y_stride * height;
    const U32 minimum_uv_size = uv_stride * chroma_height;
    frame.stVFrame.u32PlaneSize[0] = std::max(decoded.stVFrame.u32PlaneSize[0], minimum_y_size);
    frame.stVFrame.u32PlaneSize[1] = std::max(decoded.stVFrame.u32PlaneSize[1], minimum_uv_size);
    frame.stVFrame.u32PlaneSizeValid[0] =
        std::max(decoded.stVFrame.u32PlaneSizeValid[0], minimum_y_size);
    frame.stVFrame.u32PlaneSizeValid[1] =
        std::max(decoded.stVFrame.u32PlaneSizeValid[1], minimum_uv_size);
    frame.stVFrame.u32TotalSize = std::max(decoded.stVFrame.u32TotalSize,
        frame.stVFrame.u32PlaneSize[0] + frame.stVFrame.u32PlaneSize[1]);
    return frame;
}

} // namespace camera_io::detail

namespace camera_io
{
using detail::BridgeConfig;
using detail::configure_decoder_input_pool;
using detail::CropConfig;
using detail::invalidate_legacy_stream_dma_fd;
using detail::inverse_rigid_transform;
using detail::kDemuxChn;
using detail::load_config;
using detail::make_nv12_frame;
using detail::normalize_decoded_nv12_frame;
using detail::region_is_valid;
using detail::set_decoder_input_pool_mode;
using detail::StreamConfig;
using detail::transform_message;
using detail::validate_decoder_input_pool;

class CameraMppPublisher : public common::lifecycle::AsyncLifecycleNode
{
public:
    explicit CameraMppPublisher(const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
        const std::string& node_name = "camera_io")
        : AsyncLifecycleNode(node_name, options)
    {
        config_file_ = declare_parameter<std::string>("config_file", "");
        input_file_override_ = declare_parameter<std::string>("input_file", "");
        file_packet_reader_ = declare_parameter<std::string>("file_packet_reader", "mpp");
        device_override_ = declare_parameter<std::string>("device", "");
        width_override_ = declare_parameter<int>("width", 0);
        height_override_ = declare_parameter<int>("height", 0);
        fps_override_ = declare_parameter<int>("fps", 0);
        uvc_device_id_ = declare_parameter<int>("uvc_device_id", 0);
        uvc_channel_id_ = declare_parameter<int>("uvc_channel_id", 0);
        vdec_channel_id_ = declare_parameter<int>("vdec_channel_id", 0);
        file_start_ready_topic_ = declare_parameter<std::string>("file_start_ready_topic", "");
    }

    ~CameraMppPublisher() override { release_resources(); }

protected:
    using CallbackReturn =
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn on_configure(const rclcpp_lifecycle::State&) override
    {
        try
        {
            config_file_ = get_parameter("config_file").as_string();
            if (config_file_.empty())
            {
                throw std::runtime_error("config_file parameter is required");
            }
            fps_override_ = static_cast<int>(get_parameter("fps").as_int());
            config_ = load_config(config_file_, fps_override_);
            input_file_override_ = get_parameter("input_file").as_string();
            file_packet_reader_ = get_parameter("file_packet_reader").as_string();
            device_override_ = get_parameter("device").as_string();
            width_override_ = static_cast<int>(get_parameter("width").as_int());
            height_override_ = static_cast<int>(get_parameter("height").as_int());
            uvc_device_id_ = static_cast<int>(get_parameter("uvc_device_id").as_int());
            uvc_channel_id_ = static_cast<int>(get_parameter("uvc_channel_id").as_int());
            vdec_channel_id_ = static_cast<int>(get_parameter("vdec_channel_id").as_int());
            file_start_ready_topic_ = get_parameter("file_start_ready_topic").as_string();
            if (!input_file_override_.empty())
            {
                config_.input_type = "file";
                config_.input_file =
                    std::filesystem::weakly_canonical(input_file_override_).string();
            }
            if (!device_override_.empty())
            {
                config_.device = device_override_;
            }
            if (width_override_ > 0)
            {
                config_.width = width_override_;
            }
            if (height_override_ > 0)
            {
                config_.height = height_override_;
            }
            if (!config_.calibration_file.empty())
            {
                calibration_ = camera_io::load_kalibr_calibration(config_.calibration_file);
            }
            prepare_configuration();
            if (file_packet_reader_ != "mpp" && file_packet_reader_ != "libavformat")
                throw std::runtime_error("file_packet_reader must be mpp or libavformat");
            if (direct_file_packets())
            {
                if (!is_file_input() || config_.input_format != "mjpeg" || !config_.decode_enabled)
                    throw std::runtime_error(
                        "libavformat packet reader requires decoded MJPEG file input");
            }
            validate_config();

            file_start_ready_.store(file_start_ready_topic_.empty());
            if (is_file_input() && !file_start_ready_topic_.empty())
            {
                const auto ready_qos =
                    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
                file_start_ready_sub_ =
                    create_subscription<std_msgs::msg::Bool>(file_start_ready_topic_, ready_qos,
                        [this](const std_msgs::msg::Bool::SharedPtr message)
                        {
                            if (!message->data)
                            {
                                return;
                            }
                            file_start_ready_.store(true);
                            try
                            {
                                start_file_input_if_ready();
                            }
                            catch (const std::exception& error)
                            {
                                RCLCPP_ERROR(get_logger(), "playback ready-event start failed: %s",
                                    error.what());
                            }
                        });
            }

            if (config_.decode_enabled && !config_.full_only)
            {
                left_pub_ = create_publisher<mpp_msgs::msg::MppVideoFrameDesc>(
                    config_.left.topic, publication_qos());
                right_pub_ = create_publisher<mpp_msgs::msg::MppVideoFrameDesc>(
                    config_.right.topic, publication_qos());
            }
            if (config_.full_enabled)
            {
                full_pub_ = create_publisher<mpp_msgs::msg::MppVideoFrameDesc>(
                    config_.full.topic, publication_qos());
            }
            if (config_.compressed_enabled)
            {
                compressed_pub_ = create_publisher<mpp_msgs::msg::MppVideoFrameDesc>(
                    config_.compressed.topic, publication_qos());
            }
            if (calibration_ && config_.decode_enabled)
            {
                left_camera_info_ = make_camera_info(calibration_->cameras[0], config_.left);
                right_camera_info_ = make_camera_info(calibration_->cameras[1], config_.right);
                left_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
                    config_.left.camera_info_topic, publication_qos());
                right_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
                    config_.right.camera_info_topic, publication_qos());
                publish_calibration_transforms();
            }
            initialize_imu();
            open_pipeline();

            RCLCPP_INFO(get_logger(),
                "configured %s: input=%s (%s) %dx%d@%d -> left=%s %dx%d "
                "right=%s %dx%d "
                "timestamp=%s imu_raw=%s imu_fused=%s imu_rate=%.1fHz "
                "imu_domain=%d(%s) calibration=%s",
                config_file_.c_str(), config_.input_type.c_str(), input_description().c_str(),
                config_.width, config_.height, config_.fps, config_.left.topic.c_str(),
                config_.left.output_width, config_.left.output_height, config_.right.topic.c_str(),
                config_.right.output_width, config_.right.output_height,
                config_.timestamp_source.c_str(),
                config_.imu_enabled ? config_.imu_raw_dds_topic.c_str() : "disabled",
                config_.ahrs_enabled ? config_.imu_fused_dds_topic.c_str() : "disabled",
                config_.imu_enabled ? config_.imu_nominal_frequency_hz : 0.0F, config_.domain_id,
                config_.domain_id_from_environment ? "ROS_DOMAIN_ID" : "config",
                calibration_ ? config_.calibration_file.c_str() : "disabled");
            return CallbackReturn::SUCCESS;
        }
        catch (const std::exception& error)
        {
            RCLCPP_ERROR(get_logger(), "configure failed: %s", error.what());
            release_resources();
            return CallbackReturn::FAILURE;
        }
    }

    virtual void prepare_configuration() {}

    virtual rclcpp::QoS publication_qos() const { return rclcpp::SensorDataQoS(); }

    virtual void initialize_imu()
    {
        if (config_.imu_enabled)
        {
            std::size_t node_domain = 0;
            if (rcl_context_get_domain_id(
                    get_node_base_interface()->get_context()->get_rcl_context().get(),
                    &node_domain) != RCL_RET_OK)
                throw std::runtime_error("failed to resolve the ROS IMU publication domain");
            // Same-domain publication must use the node's RMW transport policy,
            // including localhost-only discovery. Retain explicit cross-domain DDS.
            if (node_domain == static_cast<std::size_t>(config_.domain_id))
            {
                const auto qos = rclcpp::SensorDataQoS().keep_last(
                    static_cast<std::size_t>(std::max(64, config_.imu_queue_depth)));
                imu_raw_ros_pub_ = rclcpp::create_publisher<sensor_msgs::msg::Imu>(
                    *this, config_.imu_raw_ros_topic, qos);
                if (config_.ahrs_enabled)
                {
                    ahrs_ = std::make_unique<camera_io::MahonyAhrs>(config_.ahrs_config);
                    imu_fused_ros_pub_ = rclcpp::create_publisher<sensor_msgs::msg::Imu>(
                        *this, config_.imu_fused_ros_topic, qos);
                }
                RCLCPP_INFO(
                    get_logger(), "IMU transport: native ROS publisher, domain=%zu", node_domain);
                return;
            }
            imu_raw_pub_ = std::make_unique<ImuPublisher>();
            if (!imu_raw_pub_->init(config_.imu_raw_dds_topic, config_.domain_id))
            {
                throw std::runtime_error("failed to create raw IMU DDS writer");
            }
            if (config_.ahrs_enabled)
            {
                ahrs_ = std::make_unique<camera_io::MahonyAhrs>(config_.ahrs_config);
                imu_fused_pub_ = std::make_unique<ImuPublisher>();
                if (!imu_fused_pub_->init(config_.imu_fused_dds_topic, config_.domain_id))
                {
                    throw std::runtime_error("failed to create fused IMU DDS writer");
                }
            }
        }
    }

    virtual bool write_imu(my_sensor_msgs::msg::Imu& message, bool fused)
    {
        const auto& ros_publisher = fused ? imu_fused_ros_pub_ : imu_raw_ros_pub_;
        if (ros_publisher)
        {
            ros_publisher->publish(to_ros_imu(message));
            return true;
        }
        return (fused ? imu_fused_pub_ : imu_raw_pub_)->writer_->write(&message);
    }

    virtual bool raw_imu_enabled() const { return bool(imu_raw_pub_) || bool(imu_raw_ros_pub_); }

    virtual bool fused_imu_enabled() const
    {
        return bool(imu_fused_pub_) || bool(imu_fused_ros_pub_);
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State&) override
    {
        if (config_.gpio_sync_enabled)
        {
            try
            {
                if (get_parameter("use_sim_time").as_bool())
                    throw std::runtime_error("GPIO synchronization requires system ROS time");
                gpio_sync_ = std::make_unique<camera_io::GpioExposure>(config_.gpio_chip,
                    config_.gpio_line, config_.gpio_active_low, config_.gpio_match);
                auto offset = gpio_clock_offset();
                if (!offset)
                    throw std::runtime_error("unable to sample ROS/monotonic clocks within 100us");
                gpio_ros_offset_ns_ = *offset;
                if (config_.startup_alignment.enabled)
                {
                    startup_alignment_ = std::make_unique<camera_io::StartupAlignment>(
                        config_.startup_alignment, camera_io::open_v4l2_exposure(config_.device),
                        [this](const std::string& message)
                        {
                            RCLCPP_INFO(
                                get_logger(), "GPIO startup alignment: %s", message.c_str());
                        });
                }
            }
            catch (const std::exception& error)
            {
                startup_alignment_.reset();
                gpio_sync_.reset();
                RCLCPP_ERROR(get_logger(), "GPIO activation failed: %s", error.what());
                return CallbackReturn::FAILURE;
            }
        }
        reset_capture_stats();
        if (is_file_input())
        {
            file_playback_active_.store(true);
            try
            {
                start_file_input_if_ready();
            }
            catch (const std::exception& error)
            {
                file_playback_active_.store(false);
                capture_running_.store(false);
                stop_capture();
                RCLCPP_ERROR(get_logger(), "playback activation failed: %s", error.what());
                return CallbackReturn::FAILURE;
            }
        }
        else
        {
            capture_running_.store(true);
            if (config_.compressed_enabled)
            {
                compressed_thread_ =
                    std::thread(&CameraMppPublisher::compressed_capture_loop, this);
            }
            if (config_.decode_enabled)
            {
                decode_thread_ = std::thread(&CameraMppPublisher::decode_loop, this);
            }
        }
        RCLCPP_INFO(get_logger(), "%s activated (%s%s)",
            is_file_input() ? "MP4 playback" : "camera capture",
            config_.decode_enabled && config_.compressed_enabled
                ? "asynchronous UVC/VDEC dual-output pipeline"
                : config_.decode_enabled ? "event-driven VDEC output"
                                            : "asynchronous compressed UVC passthrough",
            is_file_input() && !file_start_ready_.load() ? ", waiting for ready event" : "");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override
    {
        file_playback_active_.store(false);
        stop_file_input();
        stop_capture();
        RCLCPP_INFO(
            get_logger(), "%s deactivated", is_file_input() ? "MP4 playback" : "camera capture");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override
    {
        release_resources();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override
    {
        release_resources();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_error(const rclcpp_lifecycle::State&) override
    {
        release_resources();
        return CallbackReturn::SUCCESS;
    }

    void release_resources() noexcept
    {
        stop_capture();
        close_pipeline();
        static_tf_broadcaster_.reset();
        imu_fused_pub_.reset();
        imu_raw_pub_.reset();
        imu_fused_ros_pub_.reset();
        imu_raw_ros_pub_.reset();
        ahrs_.reset();
        left_info_pub_.reset();
        right_info_pub_.reset();
        left_pub_.reset();
        right_pub_.reset();
        full_pub_.reset();
        compressed_pub_.reset();
        file_start_ready_sub_.reset();
        calibration_.reset();
    }

    struct ExportedPair
    {
        UL left_handle{0};
        UL right_handle{0};
        UL full_handle{0};
    };

    struct PendingCompressed
    {
        VideoFrameInfo frame{};
        U64 token{0};
    };

    sensor_msgs::msg::CameraInfo make_camera_info(
        const camera_io::KalibrCamera& calibration, const StreamConfig& stream) const
    {
        if (calibration.camera_model != "pinhole")
        {
            throw std::runtime_error(
                calibration.name + ": only Kalibr pinhole cameras are currently supported");
        }
        sensor_msgs::msg::CameraInfo message;
        message.header.frame_id = stream.frame_id;
        message.width = static_cast<std::uint32_t>(stream.output_width);
        message.height = static_cast<std::uint32_t>(stream.output_height);
        if (calibration.distortion_model == "radtan")
        {
            message.distortion_model = "plumb_bob";
        }
        else if (calibration.distortion_model == "equidistant")
        {
            message.distortion_model = "equidistant";
        }
        else
        {
            throw std::runtime_error(calibration.name + ": unsupported distortion model " +
                calibration.distortion_model);
        }
        message.d = calibration.distortion_coeffs;
        const double scale_x = static_cast<double>(stream.output_width) /
            static_cast<double>(calibration.resolution[0]);
        const double scale_y = static_cast<double>(stream.output_height) /
            static_cast<double>(calibration.resolution[1]);
        const double fx = calibration.intrinsics[0] * scale_x;
        const double fy = calibration.intrinsics[1] * scale_y;
        const double cx = calibration.intrinsics[2] * scale_x;
        const double cy = calibration.intrinsics[3] * scale_y;
        message.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
        message.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        message.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
        return message;
    }

    void publish_calibration_transforms()
    {
        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        const auto& left = calibration_->cameras[0];
        const auto& right = calibration_->cameras[1];
        if (config_.imu_enabled && config_.publish_imu_calibration_tf && left.t_cam_imu)
        {
            geometry_msgs::msg::TransformStamped imu_to_left;
            imu_to_left.header.stamp = now();
            imu_to_left.header.frame_id = config_.imu_frame_id;
            imu_to_left.child_frame_id = config_.left.frame_id;
            imu_to_left.transform = transform_message(inverse_rigid_transform(*left.t_cam_imu));
            transforms.push_back(std::move(imu_to_left));
        }
        if (right.t_cn_cnm1)
        {
            geometry_msgs::msg::TransformStamped left_to_right;
            left_to_right.header.stamp = now();
            left_to_right.header.frame_id = config_.left.frame_id;
            left_to_right.child_frame_id = config_.right.frame_id;
            left_to_right.transform = transform_message(inverse_rigid_transform(*right.t_cn_cnm1));
            transforms.push_back(std::move(left_to_right));
        }
        if (!transforms.empty())
        {
            static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
            static_tf_broadcaster_->sendTransform(transforms);
        }
    }

    void validate_config()
    {
        validate_decoder_input_pool(config_.decoder_input_pool, config_.decode_enabled);
        if (config_.input_type != "uvc" && config_.input_type != "file")
        {
            throw std::runtime_error("input.type must be uvc or file");
        }
        if (!config_.decode_enabled && !config_.compressed_enabled)
        {
            throw std::runtime_error("camera.decode=false requires streams.compressed");
        }
        if (uvc_device_id_ < 0 || uvc_device_id_ > 15 || uvc_channel_id_ < 0 ||
            uvc_channel_id_ > 15 || vdec_channel_id_ < 0 || vdec_channel_id_ > 63)
        {
            throw std::runtime_error("invalid MPP channel ID override");
        }
        if (config_.decode_enabled && config_.decoded_format != "nv12")
        {
            throw std::runtime_error("this MPP pipeline currently supports NV12 decoder output");
        }
        if (is_file_input())
        {
            if (config_.input_format != "h264" && config_.input_format != "h265" &&
                config_.input_format != "mjpeg")
            {
                throw std::runtime_error("file input requires camera.input_format h264, h265, "
                                            "or mjpeg");
            }
            if (config_.input_file.empty() || !std::filesystem::is_regular_file(config_.input_file))
            {
                throw std::runtime_error("input.file must name an existing MP4 file");
            }
            if (config_.input_file.size() >= DEMUX_URL_MAX_LEN)
            {
                throw std::runtime_error("input.file path exceeds MPP limit");
            }
        }
        else
        {
            if (config_.device.empty())
            {
                throw std::runtime_error("camera.device is required for UVC input");
            }
            if (config_.input_format != "mjpeg")
            {
                throw std::runtime_error("UVC input currently supports only MJPEG");
            }
        }
        if (config_.width <= 0 || config_.height <= 0 || config_.fps <= 0 ||
            config_.max_exported_pairs <= 0 || config_.decoder_buffer_count < 0 ||
            config_.decoder_buffer_count > 64)
        {
            throw std::runtime_error("invalid camera or buffer values in JSON");
        }
        const auto valid_nv12_output = [](const StreamConfig& stream)
        {
            return stream.output_width > 0 && stream.output_height > 0 &&
                stream.output_width % 2 == 0 && stream.output_height % 2 == 0;
        };
        if (config_.full_only && (!config_.decode_enabled || !config_.full_enabled))
        {
            throw std::runtime_error("full_only requires decoded full stream");
        }
        if (config_.decode_enabled && !config_.full_only)
        {
            if (!region_is_valid(config_.left.crop, config_.width, config_.height) ||
                !region_is_valid(config_.right.crop, config_.width, config_.height))
            {
                throw std::runtime_error("stream crop is outside the input frame");
            }
            if (config_.left.crop.width != config_.right.crop.width ||
                config_.left.crop.height != config_.right.crop.height)
            {
                throw std::runtime_error("left and right stream dimensions must match");
            }
            if (!valid_nv12_output(config_.left) || !valid_nv12_output(config_.right))
            {
                throw std::runtime_error(
                    "stereo output dimensions must be positive and even for NV12");
            }
            if (config_.left.output_width != config_.right.output_width ||
                config_.left.output_height != config_.right.output_height)
            {
                throw std::runtime_error("left and right output dimensions must match");
            }
        }
        if (config_.full_enabled &&
            (!region_is_valid(config_.full.crop, config_.width, config_.height) ||
                config_.full.crop.x != 0 || config_.full.crop.y != 0 ||
                config_.full.crop.width != config_.width ||
                config_.full.crop.height != config_.height))
        {
            throw std::runtime_error("full stream crop must cover the complete decoded frame");
        }
        if (calibration_ && !config_.decode_enabled)
        {
            throw std::runtime_error("calibration publication requires camera.decode=true");
        }
        if ((config_.metadata_enabled || config_.imu_enabled) && !config_.decode_enabled)
        {
            throw std::runtime_error(
                "embedded metadata and IMU parsing require camera.decode=true");
        }
        if (calibration_)
        {
            if (calibration_->cameras.size() < 2)
            {
                throw std::runtime_error("the stereo bridge requires Kalibr cam0 and cam1");
            }
            if (config_.left.camera_info_topic.empty() || config_.right.camera_info_topic.empty())
            {
                throw std::runtime_error("camera_info_topic is required when calibration is set");
            }
            const auto matches_stream =
                [](const camera_io::KalibrCamera& camera, const CropConfig& crop)
            { return camera.resolution[0] == crop.width && camera.resolution[1] == crop.height; };
            if (!matches_stream(calibration_->cameras[0], config_.left.crop) ||
                !matches_stream(calibration_->cameras[1], config_.right.crop))
            {
                throw std::runtime_error("Kalibr cam0/cam1 resolution does not match stream crops");
            }
        }
        if (config_.metadata_enabled)
        {
            if (config_.metadata_parser != "camera_sdk_embedded")
            {
                throw std::runtime_error("unsupported metadata parser");
            }
            if (!region_is_valid(config_.metadata_region, config_.width, config_.height) ||
                config_.metadata_region.width < 160 || config_.metadata_region.height < 768)
            {
                throw std::runtime_error("invalid embedded metadata region");
            }
        }
        if (config_.timestamp_source != "ros_clock" &&
            config_.timestamp_source != "embedded_exposure_start" &&
            config_.timestamp_source != "embedded_exposure_midpoint")
        {
            throw std::runtime_error("unsupported timestamp source");
        }
        if ((config_.timestamp_source == "embedded_exposure_start" ||
                config_.timestamp_source == "embedded_exposure_midpoint") &&
            !config_.metadata_enabled)
        {
            throw std::runtime_error("embedded exposure timestamp requires metadata.enabled=true");
        }
        if (config_.gpio_sync_enabled &&
            (is_file_input() || !config_.decode_enabled || !config_.metadata_enabled ||
                config_.timestamp_source == "ros_clock"))
        {
            throw std::runtime_error("GPIO sync requires live decoded input and "
                                        "embedded exposure timestamps");
        }
        if (config_.compressed_enabled)
        {
            if (is_file_input())
            {
                throw std::runtime_error(
                    "compressed stream publication currently requires UVC input");
            }
            if (config_.input_format != "mjpeg")
            {
                throw std::runtime_error("compressed stream publication currently supports MJPEG");
            }
            if (config_.compressed.topic.empty() || config_.compressed.frame_id.empty())
            {
                throw std::runtime_error("compressed stream topic and frame_id are required");
            }
        }
        if (config_.imu_enabled && !config_.metadata_enabled)
        {
            throw std::runtime_error("embedded IMU requires metadata.enabled=true");
        }
        if (config_.imu_enabled && config_.imu_queue_depth <= 0)
        {
            throw std::runtime_error("imu.queue_depth must be positive");
        }
        if (config_.imu_enabled &&
            (!(config_.imu_nominal_frequency_hz > 0.0F) ||
                config_.imu_nominal_frequency_hz > 5000.0F))
        {
            throw std::runtime_error("resolved IMU frequency must be in (0, 5000]");
        }
        if (config_.ahrs_enabled && config_.orientation_variance < 0.0F)
        {
            throw std::runtime_error("imu.ahrs.orientation_stddev_rad must be non-negative");
        }
    }

    void require_ok(S32 result, const char* operation)
    {
        if (result != 0)
        {
            throw std::runtime_error(std::string(operation) + " failed: " + std::to_string(result));
        }
    }

    bool is_file_input() const { return config_.input_type == "file"; }

    bool direct_file_packets() const { return file_packet_reader_ == "libavformat"; }

    const std::string& input_description() const
    {
        return is_file_input() ? config_.input_file : config_.device;
    }

    MppStreamCodecType decoder_codec() const
    {
        if (config_.input_format == "mjpeg")
        {
            return MPP_STREAM_CODEC_MJPEG;
        }
        if (config_.input_format == "h264")
        {
            return MPP_STREAM_CODEC_H264;
        }
        if (config_.input_format == "h265")
        {
            return MPP_STREAM_CODEC_H265;
        }
        throw std::runtime_error("unsupported MPP decoder codec");
    }

    void open_decoder()
    {
        VdecChnAttr decoder_attr{};
        decoder_attr.eCodecType = decoder_codec();
        decoder_attr.eOutputPixelFormat = MPP_PIXEL_FORMAT_NV12;
        // The K3 Linlon decoder requires an explicit output alignment for
        // 4000x1200 streams. Leaving this zero can queue invalid buffers in
        // the hardware down-scale path.
        decoder_attr.u32Align = 16;
        decoder_attr.u32Width = config_.width;
        decoder_attr.u32Height = config_.height;
        // The recording contract prohibits B frames. Keeping decoder output
        // ordered lets each decoded frame retain its embedded camera metadata.
        decoder_attr.bIsFrameReordering = MPP_FALSE;
        decoder_attr.bDispErrorFrame = MPP_FALSE;
        decoder_attr.stScale.bScaleEnable = MPP_FALSE;
        decoder_attr.u32BufCnt = static_cast<U32>(config_.decoder_buffer_count);
        set_decoder_input_pool_mode(decoder_attr, config_.decoder_input_pool);
        require_ok(VDEC_CreateChn(vdec_channel_id_, &decoder_attr), "VDEC_CreateChn");
        vdec_created_ = true;
        require_ok(VDEC_EnableChn(vdec_channel_id_), "VDEC_EnableChn");
        vdec_enabled_ = true;
    }

    void configure_input_pool(const MppNode& source, const MppNode& sink)
    {
        require_ok(configure_decoder_input_pool(source, sink, config_.decoder_input_pool),
            "SYS_ConfigStreamDmaBufPool");
        if (config_.decoder_input_pool.enabled)
        {
            RCLCPP_INFO(get_logger(), "VDEC input: fixed DMA-BUF pool, %u slots x %u bytes",
                config_.decoder_input_pool.slot_count, config_.decoder_input_pool.slot_bytes);
        }
    }

    virtual void open_pipeline()
    {
        require_ok(SYS_Init(), "SYS_Init");
        sys_initialized_ = true;
        require_ok(VB_Init(), "VB_Init");
        vb_initialized_ = true;
        if (is_file_input())
        {
            if (direct_file_packets())
            {
                require_ok(VDEC_Init(), "VDEC_Init");
                vdec_initialized_ = true;
                open_decoder();
                if (config_.decoder_input_pool.enabled)
                {
                    // SYS owns this logical source and the preallocated input ring.
                    // libavformat supplies packets; no MPP DEMUX channel is opened.
                    demux_source_ = {MPP_ID_DEMUX, 0, kDemuxChn};
                    vdec_sink_ = {MPP_ID_VDEC, 0, vdec_channel_id_};
                    require_ok(
                        SYS_Bind(&demux_source_, &vdec_sink_), "SYS_Bind(file packets->VDEC)");
                    demux_vdec_bound_ = true;
                    configure_input_pool(demux_source_, vdec_sink_);
                    RCLCPP_INFO(get_logger(),
                        "MP4 input: libavformat original MJPEG packets -> "
                        "SYS_SendStream -> fixed DMA-BUF pool -> VDEC");
                }
                else
                {
                    RCLCPP_INFO(get_logger(),
                        "MP4 input: libavformat original MJPEG packets -> "
                        "VDEC_SendStream; decoder_input_pool disabled in configuration");
                }
            }
            else
            {
                require_ok(DEMUX_Init(), "DEMUX_Init");
                demux_initialized_ = true;
                require_ok(VDEC_Init(), "VDEC_Init");
                vdec_initialized_ = true;

                DemuxChnAttr demux_attr{};
                demux_attr.eInputType = DEMUX_INPUT_FILE;
                std::strncpy(
                    demux_attr.szUrl, config_.input_file.c_str(), sizeof(demux_attr.szUrl) - 1);
                demux_attr.u32OpenTimeoutMs = 5000;
                demux_attr.u32RwTimeoutMs = 5000;
                // MPP inserts VPS/SPS/PPS before each IDR for reliable VDEC
                // startup without CPU packet handling.
                demux_attr.bInjectPS = MPP_TRUE;
                require_ok(DEMUX_CreateChn(kDemuxChn, &demux_attr), "DEMUX_CreateChn");
                demux_created_ = true;

                open_decoder();
                require_ok(DEMUX_GetSrcNode(kDemuxChn, &demux_source_), "DEMUX_GetSrcNode");
                vdec_sink_ = {MPP_ID_VDEC, 0, vdec_channel_id_};
                require_ok(SYS_Bind(&demux_source_, &vdec_sink_), "SYS_Bind(DEMUX->VDEC)");
                demux_vdec_bound_ = true;
                configure_input_pool(demux_source_, vdec_sink_);
            }
        }
        else
        {
            require_ok(UVC_Init(), "UVC_Init");
            uvc_initialized_ = true;
            if (config_.decode_enabled)
            {
                require_ok(VDEC_Init(), "VDEC_Init");
                vdec_initialized_ = true;
            }

            UvcDevAttr device_attr{};
            std::strncpy(
                device_attr.acDevNode, config_.device.c_str(), sizeof(device_attr.acDevNode) - 1);
            require_ok(UVC_CreateDev(uvc_device_id_, &device_attr), "UVC_CreateDev");
            uvc_created_ = true;
            require_ok(UVC_EnableDev(uvc_device_id_), "UVC_EnableDev");
            uvc_device_enabled_ = true;

            UvcChnAttr channel_attr{};
            channel_attr.u32Width = config_.width;
            channel_attr.u32Height = config_.height;
            channel_attr.ePixelFormat = MPP_PIXEL_FORMAT_MJPEG;
            channel_attr.u32Fps = config_.fps;
            // A positive depth exposes the original compressed VB block so it
            // can be exported before the same payload is sent to VDEC.
            channel_attr.u32Depth = config_.compressed_enabled ? 6U : 0U;
            require_ok(
                UVC_SetChnAttr(uvc_device_id_, uvc_channel_id_, &channel_attr), "UVC_SetChnAttr");
            if (config_.decode_enabled)
            {
                open_decoder();
            }
            if (config_.decode_enabled)
            {
                MppNode source{MPP_ID_UVC, uvc_device_id_, uvc_channel_id_};
                MppNode destination{MPP_ID_VDEC, 0, vdec_channel_id_};
                require_ok(SYS_Bind(&source, &destination), "SYS_Bind(UVC->VDEC)");
                uvc_vdec_bound_ = true;
                configure_input_pool(source, destination);
            }
            // Finish binding and preallocation before the producer can submit input.
            require_ok(UVC_EnableChn(uvc_device_id_, uvc_channel_id_), "UVC_EnableChn");
            uvc_channel_enabled_ = true;
        }

        open_eye_pool();
    }

    void open_eye_pool()
    {
        if (config_.decode_enabled && !config_.full_only)
        {
            VbPoolCfg pool_config{};
            pool_config.u32BufSize =
                static_cast<U32>(config_.left.output_width * config_.left.output_height * 3 / 2);
            pool_config.u32BufCnt = static_cast<U32>(2 * (config_.max_exported_pairs + 2));
            pool_config.eModId = MPP_ID_V2D;
            pool_config.eRemapMode = VBUF_REMAP_MODE_NOCACHE;
            eye_pool_ = VB_CreatePool(&pool_config);
            if (eye_pool_ == 0)
            {
                throw std::runtime_error("VB_CreatePool failed");
            }
        }
    }

    void close_pipeline() noexcept
    {
        while (!exports_.empty())
        {
            release_oldest_export();
        }
        while (!compressed_exports_.empty())
        {
            (void)VB_Unexport(compressed_exports_.front());
            compressed_exports_.pop_front();
        }
        stop_file_input();
        // Return all decoder input leases before unbinding the fixed ring.
        if (uvc_channel_enabled_)
        {
            (void)UVC_DisableChn(uvc_device_id_, uvc_channel_id_);
            uvc_channel_enabled_ = false;
        }
        if (vdec_enabled_)
        {
            (void)VDEC_DisableChn(vdec_channel_id_);
            vdec_enabled_ = false;
        }
        if (demux_vdec_bound_)
        {
            (void)SYS_UnBind(&demux_source_, &vdec_sink_);
            demux_vdec_bound_ = false;
        }
        if (uvc_vdec_bound_)
        {
            MppNode source{MPP_ID_UVC, uvc_device_id_, uvc_channel_id_};
            MppNode destination{MPP_ID_VDEC, 0, vdec_channel_id_};
            (void)SYS_UnBind(&source, &destination);
            uvc_vdec_bound_ = false;
        }
        if (eye_pool_ != 0)
        {
            (void)VB_DestroyPool(eye_pool_);
            eye_pool_ = 0;
        }
        if (vdec_created_)
        {
            (void)VDEC_DestroyChn(vdec_channel_id_);
            vdec_created_ = false;
        }
        if (uvc_device_enabled_)
        {
            (void)UVC_DisableDev(uvc_device_id_);
            uvc_device_enabled_ = false;
        }
        if (uvc_created_)
        {
            (void)UVC_DestroyDev(uvc_device_id_);
            uvc_created_ = false;
        }
        if (vdec_initialized_)
        {
            (void)VDEC_Exit();
            vdec_initialized_ = false;
        }
        if (uvc_initialized_)
        {
            (void)UVC_Exit();
            uvc_initialized_ = false;
        }
        if (demux_created_)
        {
            (void)DEMUX_DestroyChn(kDemuxChn);
            demux_created_ = false;
        }
        if (demux_initialized_)
        {
            (void)DEMUX_Exit();
            demux_initialized_ = false;
        }
        if (vb_initialized_)
        {
            (void)VB_Exit();
            vb_initialized_ = false;
        }
        if (sys_initialized_)
        {
            (void)SYS_Exit();
            sys_initialized_ = false;
        }
    }

    void stop_file_input() noexcept
    {
        std::lock_guard<std::mutex> lock(file_start_mutex_);
        if (direct_file_packets())
        {
            capture_running_.store(false);
            file_packet_cv_.notify_all();
            if (file_packet_thread_.joinable())
                file_packet_thread_.join();
            file_packet_started_ = false;
        }
        if (demux_started_)
        {
            (void)DEMUX_StopChn(kDemuxChn);
            demux_started_ = false;
        }
    }

    void start_file_input_if_ready()
    {
        if (!is_file_input() || !file_playback_active_.load() || !file_start_ready_.load())
        {
            return;
        }
        std::lock_guard<std::mutex> lock(file_start_mutex_);
        if (demux_started_ || file_packet_started_)
        {
            return;
        }
        reset_capture_stats();
        capture_running_.store(true);
        if (config_.decode_enabled)
        {
            decode_thread_ = std::thread(&CameraMppPublisher::decode_loop, this);
        }
        if (direct_file_packets())
        {
            file_packets_submitted_.store(0);
            file_packets_read_complete_.store(false);
            file_packet_error_.store(false);
            try
            {
                file_packet_thread_ = std::thread(&CameraMppPublisher::file_packet_loop, this);
                file_packet_started_ = true;
            }
            catch (...)
            {
                capture_running_.store(false);
                if (decode_thread_.joinable())
                    decode_thread_.join();
                throw;
            }
            RCLCPP_INFO(get_logger(), "MP4 libavformat playback started after ready event: %s",
                file_start_ready_topic_.empty() ? "immediate" : file_start_ready_topic_.c_str());
            return;
        }
        const S32 start_result = DEMUX_StartChn(kDemuxChn);
        if (start_result != 0)
        {
            capture_running_.store(false);
            if (decode_thread_.joinable())
            {
                decode_thread_.join();
            }
            require_ok(start_result, "DEMUX_StartChn");
        }
        demux_started_ = true;
        RCLCPP_INFO(get_logger(), "MP4 playback started after ready event: %s",
            file_start_ready_topic_.empty() ? "immediate" : file_start_ready_topic_.c_str());
    }

    bool send_file_packet(const StreamBufferInfo& stream)
    {
        const bool use_pool = config_.decoder_input_pool.enabled;
        const char* operation = use_pool ? "SYS_SendStream" : "VDEC_SendStream";
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (capture_running_.load() && rclcpp::ok())
        {
            const auto result = use_pool ? SYS_SendStream(&demux_source_, &stream)
                                            : VDEC_SendStream(vdec_channel_id_, &stream, 100);
            if (result == 0)
                return true;
            // Only queue/slot exhaustion is transient on a configured SYS ring.
            // SYS_ERR_BUSY means owner/cache synchronization failure, not queue pressure.
            const bool retry = use_pool ? result == SYS_ERR_FULL
                                        : result == ERR_VDEC_BUSY || result == ERR_VDEC_TIMEOUT;
            if (!retry)
                throw std::runtime_error(
                    std::string(operation) + " failed: " + std::to_string(result));
            if (std::chrono::steady_clock::now() >= deadline)
                throw std::runtime_error("timeout submitting original MP4 packet to VDEC");
            std::unique_lock<std::mutex> lock(file_packet_mutex_);
            file_packet_cv_.wait_for(
                lock, std::chrono::milliseconds(5), [this] { return !capture_running_.load(); });
        }
        return false;
    }

    void file_packet_loop() noexcept
    {
        try
        {
            camera_io::MjpegPacketReader reader(config_.input_file, config_.width, config_.height,
                [this] { return !capture_running_.load() || !rclcpp::ok(); });
            camera_io::MjpegPacketReader::Packet packet;
            std::optional<std::uint64_t> first_pts;
            auto first_wall = std::chrono::steady_clock::now();
            std::uint64_t bytes = 0, large_packets = 0;
            while (capture_running_.load() && rclcpp::ok() && reader.next(&packet))
            {
                if (!first_pts)
                {
                    first_pts = packet.pts_us;
                    first_wall = std::chrono::steady_clock::now();
                }
                const auto delta = packet.pts_us - *first_pts;
                if (delta >
                    static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max() / 1000))
                    throw std::runtime_error("MP4 pacing interval is out of range");
                {
                    std::unique_lock<std::mutex> lock(file_packet_mutex_);
                    file_packet_cv_.wait_until(lock, first_wall + std::chrono::microseconds(delta),
                        [this] { return !capture_running_.load(); });
                }
                if (!capture_running_.load() || !rclcpp::ok())
                    return;
                StreamBufferInfo stream{};
                stream.pu8Addr = const_cast<U8*>(packet.data);
                stream.u32Size = packet.bytes;
                stream.bKeyFrame = MPP_TRUE;
                stream.eCodecType = MPP_STREAM_CODEC_MJPEG;
                stream.u64PTS = packet.pts_us;
                stream.u32Width = config_.width;
                stream.u32Height = config_.height;
                invalidate_legacy_stream_dma_fd(stream);
                if (!send_file_packet(stream))
                    return;
                ++file_packets_submitted_;
                bytes += packet.bytes;
                large_packets += packet.bytes > 512U * 1024U ? 1U : 0U;
            }
            if (!capture_running_.load() || !rclcpp::ok())
                return;
            file_packets_read_complete_.store(true);
            StreamBufferInfo eos{};
            eos.bEndOfStream = MPP_TRUE;
            eos.eCodecType = MPP_STREAM_CODEC_MJPEG;
            invalidate_legacy_stream_dma_fd(eos);
            if (!send_file_packet(eos))
                return;
            RCLCPP_INFO(get_logger(),
                "MP4 libavformat source EOF: packets=%" PRIu64 " bytes=%" PRIu64
                " packets_over_512KiB=%" PRIu64 "; EOS submitted",
                file_packets_submitted_.load(), bytes, large_packets);
        }
        catch (const std::exception& error)
        {
            if (capture_running_.load() && rclcpp::ok())
            {
                file_packet_error_.store(true);
                RCLCPP_ERROR(
                    get_logger(), "MP4 libavformat playback FAILED (not EOF): %s", error.what());
            }
            capture_running_.store(false);
            file_packet_cv_.notify_all();
        }
        catch (...)
        {
            file_packet_error_.store(true);
            capture_running_.store(false);
            file_packet_cv_.notify_all();
            RCLCPP_ERROR(get_logger(), "MP4 libavformat playback FAILED: unknown exception");
        }
    }

    void compressed_capture_loop()
    {
        while (capture_running_.load())
        {
            VideoFrameInfo compressed{};
            const S32 uvc_result = UVC_GetFrame(uvc_device_id_, uvc_channel_id_, &compressed, 100);
            if (uvc_result != 0)
            {
                ++compressed_capture_failures_;
                if (!config_.decode_enabled)
                {
                    ++capture_failures_;
                    report_capture_stats(std::chrono::steady_clock::now());
                }
                continue;
            }

            const std::uint32_t input_index = input_frame_index_++;
            ++compressed_capture_frames_;
            U64 token = 0;
            if (VB_Export(compressed.ulBufferId, &token) != 0)
            {
                ++compressed_capture_failures_;
                if (!config_.decode_enabled)
                {
                    ++capture_failures_;
                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 2000, "compressed VB_Export failed");
                }
                (void)UVC_ReleaseFrame(uvc_device_id_, uvc_channel_id_, &compressed);
                continue;
            }

            if (config_.decode_enabled)
            {
                std::unique_lock<std::mutex> lock(pending_mutex_);
                pending_cv_.wait(lock,
                    [this]()
                    {
                        return !capture_running_.load() ||
                            pending_compressed_.size() < max_pending_compressed();
                    });
                if (!capture_running_.load())
                {
                    lock.unlock();
                    (void)VB_Unexport(compressed.ulBufferId);
                    (void)UVC_ReleaseFrame(uvc_device_id_, uvc_channel_id_, &compressed);
                    break;
                }
                pending_compressed_.push_back(PendingCompressed{compressed, token});
                lock.unlock();
                pending_cv_.notify_all();
            }
            else
            {
                try
                {
                    compressed_exports_.push_back(compressed.ulBufferId);
                    publish_descriptor(*compressed_pub_, compressed, token, now(),
                        config_.compressed.frame_id, input_index);
                    while (
                        static_cast<int>(compressed_exports_.size()) > config_.max_exported_pairs)
                    {
                        (void)VB_Unexport(compressed_exports_.front());
                        compressed_exports_.pop_front();
                    }
                    ++capture_frames_;
                }
                catch (const std::exception& error)
                {
                    if (!compressed_exports_.empty() &&
                        compressed_exports_.back() == compressed.ulBufferId)
                    {
                        compressed_exports_.pop_back();
                    }
                    (void)VB_Unexport(compressed.ulBufferId);
                    ++capture_failures_;
                    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                        "compressed publish failed: %s", error.what());
                }
                report_capture_stats(std::chrono::steady_clock::now());
            }
            (void)UVC_ReleaseFrame(uvc_device_id_, uvc_channel_id_, &compressed);
        }
    }

    std::size_t max_pending_compressed() const
    {
        return static_cast<std::size_t>(std::max(2, std::min(4, config_.max_exported_pairs)));
    }

    std::optional<PendingCompressed> take_pending_compressed(U64 pts)
    {
        std::vector<UL> stale_handles;
        std::optional<PendingCompressed> match;
        {
            std::unique_lock<std::mutex> lock(pending_mutex_);
            pending_cv_.wait_for(lock, std::chrono::milliseconds(50),
                [this, pts]()
                {
                    return !capture_running_.load() ||
                        std::any_of(pending_compressed_.begin(), pending_compressed_.end(),
                            [pts](const PendingCompressed& pending)
                            { return pending.frame.stVFrame.u64PTS == pts; });
                });
            const auto position = std::find_if(pending_compressed_.begin(),
                pending_compressed_.end(), [pts](const PendingCompressed& pending)
                { return pending.frame.stVFrame.u64PTS == pts; });
            if (position != pending_compressed_.end())
            {
                for (auto it = pending_compressed_.begin(); it != position; ++it)
                {
                    stale_handles.push_back(it->frame.ulBufferId);
                }
                match = *position;
                pending_compressed_.erase(pending_compressed_.begin(), std::next(position));
            }
        }
        for (const UL handle : stale_handles)
        {
            (void)VB_Unexport(handle);
            ++compressed_correlation_drops_;
        }
        pending_cv_.notify_all();
        return match;
    }

    void decode_once()
    {
        const auto begin = std::chrono::steady_clock::now();
        VideoFrameInfo decoded{};
        const S32 result = VDEC_GetFrame(vdec_channel_id_, &decoded, 100);
        const auto decoded_at = std::chrono::steady_clock::now();
        if (result != ERR_VDEC_OK)
        {
            if (result == ERR_VDEC_EOS && is_file_input())
            {
                if (direct_file_packets())
                {
                    const bool complete = file_packets_read_complete_.load() &&
                        !file_packet_error_.load() &&
                        capture_frames_ == file_packets_submitted_.load();
                    if (complete)
                        RCLCPP_INFO(get_logger(),
                            "MP4 libavformat decode complete: packets=%" PRIu64 " decoded=%" PRIu64,
                            file_packets_submitted_.load(), capture_frames_);
                    else
                    {
                        file_packet_error_.store(true);
                        RCLCPP_ERROR(get_logger(),
                            "MP4 libavformat decode INCOMPLETE: packets=%" PRIu64
                            " decoded=%" PRIu64,
                            file_packets_submitted_.load(), capture_frames_);
                        capture_running_.store(false);
                        file_packet_cv_.notify_all();
                        report_capture_stats(decoded_at);
                        return;
                    }
                }
                RCLCPP_INFO(get_logger(), "MP4 playback reached end of stream");
                capture_running_.store(false);
                report_capture_stats(decoded_at);
                return;
            }
            ++capture_failures_;
            if (capture_running_.load())
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000, "VDEC_GetFrame failed: %d", result);
            }
            report_capture_stats(decoded_at);
            return;
        }

        std::optional<PendingCompressed> pending;
        if (!is_file_input() && config_.compressed_enabled)
        {
            pending = take_pending_compressed(decoded.stVFrame.u64PTS);
        }
        const std::uint32_t published_index = decoded_frame_index_++;
        const auto image_stamp = process_decoded(decoded, published_index);
        if (pending && image_stamp && compressed_pub_)
        {
            try
            {
                compressed_exports_.push_back(pending->frame.ulBufferId);
                publish_descriptor(*compressed_pub_, pending->frame, pending->token, *image_stamp,
                    config_.compressed.frame_id, published_index);
                while (static_cast<int>(compressed_exports_.size()) > config_.max_exported_pairs)
                {
                    (void)VB_Unexport(compressed_exports_.front());
                    compressed_exports_.pop_front();
                }
            }
            catch (const std::exception& error)
            {
                if (!compressed_exports_.empty() &&
                    compressed_exports_.back() == pending->frame.ulBufferId)
                {
                    compressed_exports_.pop_back();
                }
                (void)VB_Unexport(pending->frame.ulBufferId);
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                    "compressed publish failed: %s", error.what());
            }
        }
        else if (pending)
        {
            (void)VB_Unexport(pending->frame.ulBufferId);
        }
        const auto processed_at = std::chrono::steady_clock::now();
        (void)VDEC_ReleaseFrame(vdec_channel_id_, decoded.ulBufferId);
        const auto released_at = std::chrono::steady_clock::now();

        ++capture_frames_;
        vdec_wait_ns_ +=
            std::chrono::duration_cast<std::chrono::nanoseconds>(decoded_at - begin).count();
        process_ns_ +=
            std::chrono::duration_cast<std::chrono::nanoseconds>(processed_at - decoded_at).count();
        release_ns_ +=
            std::chrono::duration_cast<std::chrono::nanoseconds>(released_at - processed_at)
                .count();
        report_capture_stats(released_at);
    }

    void decode_loop()
    {
        while (capture_running_.load())
        {
            decode_once();
        }
    }

    void stop_capture() noexcept
    {
        capture_running_.store(false);
        file_packet_cv_.notify_all();
        pending_cv_.notify_all();
        if (file_packet_thread_.joinable())
        {
            file_packet_thread_.join();
        }
        if (compressed_thread_.joinable())
        {
            compressed_thread_.join();
        }
        if (decode_thread_.joinable())
        {
            decode_thread_.join();
        }
        startup_alignment_.reset();
        gpio_sync_.reset();
        std::deque<PendingCompressed> pending;
        {
            std::lock_guard<std::mutex> lock(pending_mutex_);
            pending.swap(pending_compressed_);
        }
        for (const auto& entry : pending)
        {
            (void)VB_Unexport(entry.frame.ulBufferId);
        }
    }

    void reset_capture_stats()
    {
        input_frame_index_ = 0;
        decoded_frame_index_ = 0;
        last_imu_timestamp_ns_ = 0;
        gpio_last_imu_raw_.reset();
        imu_duplicate_count_ = 0;
        imu_backward_count_ = 0;
        imu_metadata_sample_count_ = 0;
        imu_invalid_sample_count_ = 0;
        imu_raw_write_successes_ = 0;
        imu_raw_write_failures_ = 0;
        imu_fused_write_failures_ = 0;
        capture_frames_ = 0;
        capture_failures_ = 0;
        compressed_capture_frames_ = 0;
        compressed_capture_failures_ = 0;
        compressed_correlation_drops_ = 0;
        vdec_wait_ns_ = 0;
        process_ns_ = 0;
        release_ns_ = 0;
        last_image_timestamp_ns_ = 0;
        image_timestamp_regression_count_ = 0;
        metadata_timestamp_rejection_count_ = 0;
        clock_.reset();
        capture_stats_start_ = std::chrono::steady_clock::now();
        capture_stats_last_report_ = capture_stats_start_;
    }

    void report_capture_stats(const std::chrono::steady_clock::time_point now)
    {
        if (now - capture_stats_last_report_ < std::chrono::seconds(2))
        {
            return;
        }

        const double elapsed_s = std::chrono::duration<double>(now - capture_stats_start_).count();
        const double divisor = capture_frames_ == 0 ? 1.0 : static_cast<double>(capture_frames_);
        RCLCPP_INFO(get_logger(),
            "capture frames=%" PRIu64 " failures=%" PRIu64 " fps=%.1f input_frames=%" PRIu64
            " input_fps=%.1f "
            "input_failures=%" PRIu64 " correlation_drops=%" PRIu64 " vdec_wait_avg=%.2fms "
            "process_avg=%.2fms release_avg=%.3fms metadata_reject=%" PRIu64
            " imu_metadata=%" PRIu64 " imu_invalid=%" PRIu64 " imu_duplicate=%" PRIu64
            " imu_backward=%" PRIu64 " imu_written=%" PRIu64 " imu_write_failures=%" PRIu64,
            capture_frames_, capture_failures_,
            elapsed_s > 0.0 ? static_cast<double>(capture_frames_) / elapsed_s : 0.0,
            compressed_capture_frames_.load(),
            elapsed_s > 0.0 ? static_cast<double>(compressed_capture_frames_.load()) / elapsed_s
                            : 0.0,
            compressed_capture_failures_.load(), compressed_correlation_drops_.load(),
            static_cast<double>(vdec_wait_ns_) / divisor / 1.0e6,
            static_cast<double>(process_ns_) / divisor / 1.0e6,
            static_cast<double>(release_ns_) / divisor / 1.0e6, metadata_timestamp_rejection_count_,
            imu_metadata_sample_count_, imu_invalid_sample_count_, imu_duplicate_count_,
            imu_backward_count_, imu_raw_write_successes_, imu_raw_write_failures_);
        capture_stats_last_report_ = now;
    }

    std::optional<std::int64_t> gpio_clock_offset()
    {
        std::int64_t best_span = 100001, best_offset = 0;
        for (int i = 0; i < 5; ++i)
        {
            const auto before = camera_io::GpioExposure::monotonic_ns();
            const auto ros = now().nanoseconds();
            const auto after = camera_io::GpioExposure::monotonic_ns();
            if (after - before < best_span)
            {
                best_span = after - before;
                best_offset = ros - before - (after - before) / 2;
            }
        }
        if (best_span > 100000)
            return std::nullopt;
        return best_offset;
    }

    std::optional<rclcpp::Time> process_decoded(const VideoFrameInfo& decoded,
        std::uint32_t descriptor_index,
        std::optional<std::int64_t> recorded_image_stamp = std::nullopt, bool publish_images = true,
        bool parse_metadata = true, bool publish_stereo = true)
    {
        rclcpp::Time image_stamp =
            recorded_image_stamp ? rclcpp::Time(*recorded_image_stamp) : now();
        if (config_.metadata_enabled && parse_metadata)
        {
            camera_sdk::EmbeddedMetadata metadata;
            const auto metadata_offset = static_cast<std::size_t>(config_.metadata_region.y) *
                    decoded.stVFrame.u32PlaneStride[0] +
                static_cast<std::size_t>(config_.metadata_region.x);
            camera_sdk::LumaView luma{
                reinterpret_cast<const std::uint8_t*>(decoded.stVFrame.ulPlaneVirAddr[0]) +
                    metadata_offset,
                static_cast<std::uint32_t>(config_.metadata_region.width),
                static_cast<std::uint32_t>(config_.metadata_region.height),
                decoded.stVFrame.u32PlaneStride[0], 1};
            std::string metadata_error;
            const bool metadata_ok =
                camera_sdk::decode_embedded_metadata(luma, metadata, &metadata_error);
            if (metadata_ok)
            {
                camera_io::CameraClock::FrameTimestamps timestamps;
                // A session sidecar preserves the configured exposure event,
                // not necessarily exposure START. Anchor IMU and images using
                // that same convention, including midpoint recordings.
                const auto reference = config_.timestamp_source == "embedded_exposure_midpoint"
                    ? CameraClock::ExposureReference::Midpoint
                    : CameraClock::ExposureReference::Start;
                const bool clock_ok = recorded_image_stamp
                    ? clock_.preview_recorded(metadata.exposure_start_raw_us,
                            metadata.exposure_end_raw_us, *recorded_image_stamp, reference,
                            &timestamps, &metadata_error)
                    : clock_.preview(metadata.exposure_start_raw_us, metadata.exposure_end_raw_us,
                            image_stamp.nanoseconds(), &timestamps, &metadata_error);
                if (!clock_ok)
                {
                    if (recorded_image_stamp)
                    {
                        throw std::runtime_error(metadata_error);
                    }
                    if (gpio_sync_)
                    {
                        gpio_sync_->reset();
                        clock_.reset();
                        gpio_last_imu_raw_.reset();
                    }
                    ++metadata_timestamp_rejection_count_;
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "metadata timestamp rejected before publish: %s "
                        "total=%" PRIu64,
                        metadata_error.c_str(), metadata_timestamp_rejection_count_);
                    return std::nullopt;
                }
                if (gpio_sync_)
                {
                    const auto offset = gpio_clock_offset();
                    if (!offset)
                        return std::nullopt;
                    if (std::llabs(*offset - gpio_ros_offset_ns_) > 5000000)
                    {
                        gpio_sync_->reset();
                        gpio_ros_offset_ns_ = *offset;
                    }
                    const auto duration_us = static_cast<std::uint32_t>(
                        metadata.exposure_end_raw_us - metadata.exposure_start_raw_us);
                    const auto end_ns = timestamps.exposure_end_extended_us * 1000;
                    auto matched = gpio_sync_->match({end_ns - duration_us * 1000LL, end_ns});
                    const auto state = gpio_sync_->status();
                    if (state != gpio_sync_status_)
                    {
                        RCLCPP_INFO(get_logger(), "GPIO exposure sync: %s", state.c_str());
                        gpio_sync_status_ = state;
                    }
                    const bool startup_ready = !startup_alignment_ ||
                        startup_alignment_->observe(duration_us, matched.has_value());
                    if (!matched || !startup_ready)
                    {
                        clock_.commit(timestamps); // Unwrap metadata, but publish
                                                   // neither images nor IMU.
                        return std::nullopt;
                    }
                    timestamps.exposure_start_ns = matched->start_ns + gpio_ros_offset_ns_;
                    timestamps.exposure_end_ns = matched->end_ns + gpio_ros_offset_ns_;
                    timestamps.epoch_offset_ns = timestamps.exposure_end_ns - end_ns;
                }
                if (config_.timestamp_source == "embedded_exposure_start" ||
                    config_.timestamp_source == "embedded_exposure_midpoint")
                {
                    image_stamp =
                        rclcpp::Time(config_.timestamp_source == "embedded_exposure_midpoint"
                                ? timestamps.exposure_midpoint_ns()
                                : timestamps.exposure_start_ns);
                    if (image_stamp.nanoseconds() <=
                        static_cast<std::int64_t>(last_image_timestamp_ns_))
                    {
                        ++image_timestamp_regression_count_;
                        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                            "non-monotonic image timestamp rejected before "
                            "metadata commit; total=%" PRIu64,
                            image_timestamp_regression_count_);
                        return std::nullopt;
                    }
                }
                // The decoded metadata is the only source of embedded IMU
                // samples. Commit and write the native-timestamp IMU block
                // before V2D or image publication so downstream video work
                // cannot delay its delivery.
                clock_.commit(timestamps);
                publish_imu(metadata);
            }
            else
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "embedded metadata decode failed: %s", metadata_error.c_str());
                if (config_.timestamp_source == "embedded_exposure_start" ||
                    config_.timestamp_source == "embedded_exposure_midpoint")
                {
                    return std::nullopt;
                }
            }
        }

        if (!publish_images)
            return image_stamp;

        // Embedded exposure time is the synchronization authority.  A VDEC
        // recycle/source-change can occasionally return an older metadata
        // block; forwarding it would make downstream visual tracking reject
        // the stereo pair.  Drop it here instead of substituting a receive
        // timestamp.
        const std::uint64_t image_timestamp_ns =
            static_cast<std::uint64_t>(image_stamp.nanoseconds());
        if (image_timestamp_ns <= last_image_timestamp_ns_)
        {
            ++image_timestamp_regression_count_;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "non-monotonic image timestamp dropped; total=%" PRIu64,
                image_timestamp_regression_count_);
            return std::nullopt;
        }
        last_image_timestamp_ns_ = image_timestamp_ns;

        if (config_.full_only || !publish_stereo)
        {
            if (!config_.full_enabled || !full_pub_)
                return image_stamp;
            const auto full =
                normalize_decoded_nv12_frame(decoded, static_cast<std::uint32_t>(config_.width),
                    static_cast<std::uint32_t>(config_.height), descriptor_index);
            U64 token = 0;
            if (VB_Export(decoded.ulBufferId, &token) != 0)
            {
                throw std::runtime_error("full VB_Export failed");
            }
            try
            {
                exports_.push_back({0, 0, decoded.ulBufferId});
            }
            catch (...)
            {
                (void)VB_Unexport(decoded.ulBufferId);
                throw;
            }
            while (static_cast<int>(exports_.size()) > config_.max_exported_pairs)
            {
                release_oldest_export();
            }
            publish_descriptor(
                *full_pub_, full, token, image_stamp, config_.full.frame_id, descriptor_index);
            return image_stamp;
        }

        UL first_buffer = VB_GetBuffer(eye_pool_, 100);
        UL second_buffer = VB_GetBuffer(eye_pool_, 100);
        if (first_buffer == 0 || second_buffer == 0)
        {
            if (first_buffer != 0)
                (void)VB_ReleaseBuffer(first_buffer);
            if (second_buffer != 0)
                (void)VB_ReleaseBuffer(second_buffer);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "eye VB pool exhausted");
            return std::nullopt;
        }

        try
        {
            VideoFrameInfo first =
                make_nv12_frame(eye_pool_, first_buffer, config_.left.output_width,
                    config_.left.output_height, descriptor_index, decoded.stVFrame.u64PTS);
            VideoFrameInfo second =
                make_nv12_frame(eye_pool_, second_buffer, config_.right.output_width,
                    config_.right.output_height, descriptor_index, decoded.stVFrame.u64PTS);
            if (!crop_eyes(decoded, first, second))
            {
                throw std::runtime_error("V2D stereo crop failed");
            }
            (void)VB_UpdateBufferFrameInfo(first_buffer, &first);
            (void)VB_UpdateBufferFrameInfo(second_buffer, &second);

            VideoFrameInfo& left = first;
            VideoFrameInfo& right = second;
            // VDEC does not guarantee that every public frame-info field is
            // populated on the source-change/recycled output path.  The
            // exported full-frame descriptor must nevertheless carry a
            // complete NV12 layout for consumers such as LAS2.  Rebuild the
            // descriptor from the live VB block instead of forwarding the
            // partially populated VDEC metadata.
            VideoFrameInfo full = decoded;
            if (config_.full_enabled)
            {
                full =
                    normalize_decoded_nv12_frame(decoded, static_cast<std::uint32_t>(config_.width),
                        static_cast<std::uint32_t>(config_.height), descriptor_index);
                RCLCPP_INFO_ONCE(get_logger(),
                    "full decoded NV12 layout: %ux%u stride=%u/%u "
                    "size=%u/%u total=%u",
                    full.stCommFrameInfo.u32Width, full.stCommFrameInfo.u32Height,
                    full.stVFrame.u32PlaneStride[0], full.stVFrame.u32PlaneStride[1],
                    full.stVFrame.u32PlaneSize[0], full.stVFrame.u32PlaneSize[1],
                    full.stVFrame.u32TotalSize);
            }
            U64 left_token = 0;
            U64 right_token = 0;
            U64 full_token = 0;
            if (VB_Export(left.ulBufferId, &left_token) != 0)
            {
                throw std::runtime_error("left VB_Export failed");
            }
            if (VB_Export(right.ulBufferId, &right_token) != 0)
            {
                (void)VB_Unexport(left.ulBufferId);
                throw std::runtime_error("right VB_Export failed");
            }
            if (config_.full_enabled && VB_Export(decoded.ulBufferId, &full_token) != 0)
            {
                (void)VB_Unexport(left.ulBufferId);
                (void)VB_Unexport(right.ulBufferId);
                throw std::runtime_error("full VB_Export failed");
            }
            bool export_recorded = false;
            try
            {
                exports_.push_back({left.ulBufferId, right.ulBufferId,
                    config_.full_enabled ? decoded.ulBufferId : 0});
                export_recorded = true;

                publish_descriptor(*left_pub_, left, left_token, image_stamp, config_.left.frame_id,
                    descriptor_index);
                publish_descriptor(*right_pub_, right, right_token, image_stamp,
                    config_.right.frame_id, descriptor_index);
                if (config_.full_enabled && full_pub_)
                {
                    publish_descriptor(*full_pub_, full, full_token, image_stamp,
                        config_.full.frame_id, descriptor_index);
                }
                if (left_info_pub_ && right_info_pub_)
                {
                    left_camera_info_.header.stamp = image_stamp;
                    right_camera_info_.header.stamp = image_stamp;
                    left_info_pub_->publish(left_camera_info_);
                    right_info_pub_->publish(right_camera_info_);
                }
            }
            catch (...)
            {
                // The descriptor queue owns the export references only after
                // the pair is inserted.  If publishing/allocation fails,
                // revoke all exports immediately to avoid pinning VDEC/VB.
                if (export_recorded)
                {
                    const auto pair = exports_.back();
                    exports_.pop_back();
                    if (pair.left_handle)
                        (void)VB_Unexport(pair.left_handle);
                    if (pair.right_handle)
                        (void)VB_Unexport(pair.right_handle);
                    if (pair.full_handle != 0)
                    {
                        (void)VB_Unexport(pair.full_handle);
                    }
                }
                else
                {
                    (void)VB_Unexport(left.ulBufferId);
                    (void)VB_Unexport(right.ulBufferId);
                    if (config_.full_enabled)
                    {
                        (void)VB_Unexport(decoded.ulBufferId);
                    }
                }
                throw;
            }
        }
        catch (const std::exception& error)
        {
            if (recorded_image_stamp)
            {
                (void)VB_ReleaseBuffer(first_buffer);
                (void)VB_ReleaseBuffer(second_buffer);
                throw;
            }
            RCLCPP_ERROR(get_logger(), "%s", error.what());
        }
        catch (...)
        {
            (void)VB_ReleaseBuffer(first_buffer);
            (void)VB_ReleaseBuffer(second_buffer);
            throw;
        }

        (void)VB_ReleaseBuffer(first_buffer);
        (void)VB_ReleaseBuffer(second_buffer);
        while (static_cast<int>(exports_.size()) > config_.max_exported_pairs)
        {
            release_oldest_export();
        }
        return image_stamp;
    }

    bool crop_eyes(const VideoFrameInfo& source, VideoFrameInfo& first, VideoFrameInfo& second)
    {
        V2DHandle job = 0;
        S32 result = V2D_BeginJob(&job);
        if (result != 0)
        {
            RCLCPP_ERROR(get_logger(), "V2D_BeginJob failed: %d", result);
            return false;
        }
        V2DArea first_source{static_cast<U16>(config_.left.crop.x),
            static_cast<U16>(config_.left.crop.y), static_cast<U16>(config_.left.crop.width),
            static_cast<U16>(config_.left.crop.height)};
        V2DArea second_source{static_cast<U16>(config_.right.crop.x),
            static_cast<U16>(config_.right.crop.y), static_cast<U16>(config_.right.crop.width),
            static_cast<U16>(config_.right.crop.height)};
        V2DArea first_destination{0, 0, static_cast<U16>(config_.left.output_width),
            static_cast<U16>(config_.left.output_height)};
        V2DArea second_destination{0, 0, static_cast<U16>(config_.right.output_width),
            static_cast<U16>(config_.right.output_height)};
        result = V2D_AddBitblitTask(
            job, &source, &first_source, &first, &first_destination, V2D_CSC_MODE_BUTT);
        if (result != 0)
        {
            RCLCPP_ERROR(get_logger(), "first V2D_AddBitblitTask failed: %d", result);
            (void)V2D_CancelJob(job);
            return false;
        }
        result = V2D_AddBitblitTask(
            job, &source, &second_source, &second, &second_destination, V2D_CSC_MODE_BUTT);
        if (result != 0)
        {
            RCLCPP_ERROR(get_logger(), "second V2D_AddBitblitTask failed: %d", result);
            (void)V2D_CancelJob(job);
            return false;
        }
        result = V2D_EndJob(job);
        if (result != 0)
        {
            RCLCPP_ERROR(get_logger(), "V2D_EndJob failed: %d", result);
            return false;
        }
        return true;
    }

    void publish_descriptor(rclcpp::Publisher<mpp_msgs::msg::MppVideoFrameDesc>& publisher,
        const VideoFrameInfo& frame, U64 token, const rclcpp::Time& stamp,
        const std::string& frame_id, std::uint32_t descriptor_index)
    {
        mpp_msgs::msg::MppVideoFrameDesc message;
        message.header.stamp = stamp;
        message.header.frame_id = frame_id;
        message.version = 1;
        message.token = token;
        message.pool_id = frame.ulPoolId;
        message.buffer_id = frame.ulBufferId;
        message.frame_index = descriptor_index;
        message.width = frame.stCommFrameInfo.u32Width;
        message.height = frame.stCommFrameInfo.u32Height;
        message.align = frame.stCommFrameInfo.u32Align;
        message.pixel_format = frame.stCommFrameInfo.ePixelFormat;
        message.compress_mode = frame.stCommFrameInfo.eCompressMode;
        message.color_space = frame.stCommFrameInfo.eColorSpace;
        message.pts = frame.stVFrame.u64PTS;
        message.total_size = frame.stVFrame.u32TotalSize;
        message.plane_num = frame.stVFrame.u32PlaneNum;
        for (std::size_t i = 0; i < message.plane_stride.size() && i < frame.stVFrame.u32PlaneNum;
            ++i)
        {
            message.plane_stride[i] = frame.stVFrame.u32PlaneStride[i];
            message.plane_size[i] = frame.stVFrame.u32PlaneSize[i];
            message.plane_size_valid[i] = frame.stVFrame.u32PlaneSizeValid[i];
            message.plane_phy_addr[i] = frame.stVFrame.u64PlanePhyAddr[i];
        }
        publisher.publish(std::move(message));
    }

    void publish_imu(const camera_sdk::EmbeddedMetadata& metadata)
    {
        if (!config_.imu_enabled || !raw_imu_enabled())
            return;
        imu_metadata_sample_count_ += metadata.imu_count;
        for (std::size_t i = 0; i < metadata.imu_count; ++i)
        {
            const auto& sample = metadata.imu[i];
            if (!sample.valid || !clock_.initialized())
            {
                ++imu_invalid_sample_count_;
                continue;
            }
            if (gpio_sync_ && gpio_last_imu_raw_)
            {
                const auto delta =
                    static_cast<std::int32_t>(sample.timestamp_raw_us - *gpio_last_imu_raw_);
                if (delta <= 0)
                {
                    if (delta == 0)
                        ++imu_duplicate_count_;
                    else
                        ++imu_backward_count_;
                    continue;
                }
            }
            const std::int64_t timestamp_ns = clock_.ros_time_ns(sample.timestamp_raw_us);
            if (timestamp_ns <= 0 ||
                static_cast<std::uint64_t>(timestamp_ns) <= last_imu_timestamp_ns_)
            {
                if (static_cast<std::uint64_t>(timestamp_ns) == last_imu_timestamp_ns_)
                {
                    ++imu_duplicate_count_;
                }
                else
                {
                    ++imu_backward_count_;
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "backward IMU timestamp dropped; total=%" PRIu64, imu_backward_count_);
                }
                continue;
            }
            last_imu_timestamp_ns_ = static_cast<std::uint64_t>(timestamp_ns);
            if (gpio_sync_)
                gpio_last_imu_raw_ = sample.timestamp_raw_us;

            my_sensor_msgs::msg::Imu message;
            message.header().stamp().sec(static_cast<std::int32_t>(timestamp_ns / 1000000000LL));
            message.header().stamp().nanosec(
                static_cast<std::uint32_t>(timestamp_ns % 1000000000LL));
            message.header().frame_id(config_.imu_frame_id);
            message.orientation().w(1.0);
            message.orientation_covariance().fill(0.0);
            message.orientation_covariance()[0] = -1.0;
            message.angular_velocity().x(sample.gyro_rad_s[0]);
            message.angular_velocity().y(sample.gyro_rad_s[1]);
            message.angular_velocity().z(sample.gyro_rad_s[2]);
            message.angular_velocity_covariance().fill(0.0);
            message.linear_acceleration().x(sample.accel_m_s2[0]);
            message.linear_acceleration().y(sample.accel_m_s2[1]);
            message.linear_acceleration().z(sample.accel_m_s2[2]);
            message.linear_acceleration_covariance().fill(0.0);
            const bool raw_written = write_imu(message, false);
            if (!raw_written)
            {
                ++imu_raw_write_failures_;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "raw IMU DDS write failed; total=%" PRIu64, imu_raw_write_failures_);
            }
            else
            {
                ++imu_raw_write_successes_;
            }

            if (ahrs_ && fused_imu_enabled() &&
                ahrs_->update(sample.gyro_rad_s[0], sample.gyro_rad_s[1], sample.gyro_rad_s[2],
                    sample.accel_m_s2[0], sample.accel_m_s2[1], sample.accel_m_s2[2], timestamp_ns))
            {
                const auto& orientation = ahrs_->orientation();
                message.orientation().w(orientation.w);
                message.orientation().x(orientation.x);
                message.orientation().y(orientation.y);
                message.orientation().z(orientation.z);
                message.orientation_covariance().fill(0.0);
                message.orientation_covariance()[0] = config_.orientation_variance;
                message.orientation_covariance()[4] = config_.orientation_variance;
                message.orientation_covariance()[8] = config_.orientation_variance;
                if (!write_imu(message, true))
                {
                    ++imu_fused_write_failures_;
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "fused IMU DDS write failed; total=%" PRIu64, imu_fused_write_failures_);
                }
            }
        }
    }

    void release_oldest_export()
    {
        const auto pair = exports_.front();
        exports_.pop_front();
        if (pair.left_handle)
            (void)VB_Unexport(pair.left_handle);
        if (pair.right_handle)
            (void)VB_Unexport(pair.right_handle);
        if (pair.full_handle != 0)
        {
            (void)VB_Unexport(pair.full_handle);
        }
    }

    std::string config_file_;
    std::string input_file_override_;
    std::string file_packet_reader_;
    std::string file_start_ready_topic_;
    std::string device_override_;
    int width_override_{0};
    int height_override_{0};
    int fps_override_{0};
    int uvc_device_id_{0};
    int uvc_channel_id_{0};
    int vdec_channel_id_{0};
    BridgeConfig config_;
    std::uint32_t input_frame_index_{0};
    std::uint32_t decoded_frame_index_{0};
    std::uint64_t last_imu_timestamp_ns_{0};
    std::uint64_t last_image_timestamp_ns_{0};
    std::uint64_t imu_duplicate_count_{0};
    std::uint64_t imu_backward_count_{0};
    std::uint64_t image_timestamp_regression_count_{0};
    std::uint64_t imu_metadata_sample_count_{0};
    std::uint64_t imu_invalid_sample_count_{0};
    std::uint64_t imu_raw_write_successes_{0};
    std::uint64_t imu_raw_write_failures_{0};
    std::uint64_t imu_fused_write_failures_{0};
    camera_io::CameraClock clock_;
    std::unique_ptr<camera_io::GpioExposure> gpio_sync_;
    std::unique_ptr<camera_io::StartupAlignment> startup_alignment_;
    std::int64_t gpio_ros_offset_ns_{0};
    std::string gpio_sync_status_;
    std::optional<std::uint32_t> gpio_last_imu_raw_;
    std::uint64_t metadata_timestamp_rejection_count_{0};

    bool sys_initialized_{false};
    bool vb_initialized_{false};
    bool uvc_initialized_{false};
    bool demux_initialized_{false};
    bool vdec_initialized_{false};
    bool demux_created_{false};
    bool demux_started_{false};
    std::atomic<bool> file_start_ready_{true};
    std::atomic<bool> file_playback_active_{false};
    std::mutex file_start_mutex_;
    bool uvc_created_{false};
    bool uvc_device_enabled_{false};
    bool uvc_channel_enabled_{false};
    bool vdec_created_{false};
    bool vdec_enabled_{false};
    bool demux_vdec_bound_{false};
    bool uvc_vdec_bound_{false};
    MppNode demux_source_{};
    MppNode vdec_sink_{};
    UL eye_pool_{0};
    std::deque<ExportedPair> exports_;
    std::deque<UL> compressed_exports_;
    std::mutex pending_mutex_;
    std::condition_variable pending_cv_;
    std::deque<PendingCompressed> pending_compressed_;

    std::unique_ptr<ImuPublisher> imu_raw_pub_;
    std::unique_ptr<ImuPublisher> imu_fused_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_raw_ros_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_fused_ros_pub_;
    std::unique_ptr<camera_io::MahonyAhrs> ahrs_;
    rclcpp::Publisher<mpp_msgs::msg::MppVideoFrameDesc>::SharedPtr left_pub_;
    rclcpp::Publisher<mpp_msgs::msg::MppVideoFrameDesc>::SharedPtr right_pub_;
    rclcpp::Publisher<mpp_msgs::msg::MppVideoFrameDesc>::SharedPtr full_pub_;
    rclcpp::Publisher<mpp_msgs::msg::MppVideoFrameDesc>::SharedPtr compressed_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr file_start_ready_sub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;
    sensor_msgs::msg::CameraInfo left_camera_info_;
    sensor_msgs::msg::CameraInfo right_camera_info_;
    std::optional<camera_io::KalibrCalibration> calibration_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    std::atomic<bool> capture_running_{false};
    std::thread compressed_thread_;
    std::thread decode_thread_;
    std::thread file_packet_thread_;
    std::mutex file_packet_mutex_;
    std::condition_variable file_packet_cv_;
    bool file_packet_started_{false};
    std::atomic<std::uint64_t> file_packets_submitted_{0};
    std::atomic<bool> file_packets_read_complete_{false};
    std::atomic<bool> file_packet_error_{false};
    std::chrono::steady_clock::time_point capture_stats_start_;
    std::chrono::steady_clock::time_point capture_stats_last_report_;
    std::uint64_t capture_frames_{0};
    std::uint64_t capture_failures_{0};
    std::atomic<std::uint64_t> compressed_capture_frames_{0};
    std::atomic<std::uint64_t> compressed_capture_failures_{0};
    std::atomic<std::uint64_t> compressed_correlation_drops_{0};
    std::uint64_t vdec_wait_ns_{0};
    std::uint64_t process_ns_{0};
    std::uint64_t release_ns_{0};
};

} // namespace camera_io

#endif // CAMERA_MPP_PUBLISHER_IMPL_H
