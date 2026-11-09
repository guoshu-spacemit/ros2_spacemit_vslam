#ifndef IMU_TRANSPORT_H
#define IMU_TRANSPORT_H
#include <Imu.h>
#include <ImuPublisher.h>
#include <sensor_msgs/msg/imu.hpp>

namespace camera_io
{
using DdsImuPublisher = ::ImuPublisher;
using DdsImuMessage = my_sensor_msgs::msg::Imu;

// Converts metadata only. The caller selects ROS RMW or explicit-domain DDS.
sensor_msgs::msg::Imu to_ros_imu(const DdsImuMessage& source);
} // namespace camera_io

#endif // IMU_TRANSPORT_H
