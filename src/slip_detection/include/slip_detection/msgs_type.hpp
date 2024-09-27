#ifndef SLIPDETECTION__MSGS_TYPE_HPP
#define SLIPDETECTION__MSGS_TYPE_HPP

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace anomaly_detection
{
    struct MessageType_ODOM_LO_IMU
    {
        nav_msgs::msg::Odometry::ConstSharedPtr odom_msg;
        geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg; 
        sensor_msgs::msg::Imu::ConstSharedPtr imu_msg;
    };
    
} // namespace anomaly_detection


#endif // SLIPDETECTION__MSGS_TYPE_HPP