#ifndef SLIPDETECTION__MSGS_TYPE_HPP
#define SLIPDETECTION__MSGS_TYPE_HPP
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace anomaly_detection
{   
    struct MessageType_ODOM
    {
        nav_msgs::msg::Odometry::ConstSharedPtr odom_msg;
    };

    struct MessageType_ODOM_IMU : public MessageType_ODOM
    {
        sensor_msgs::msg::Imu::ConstSharedPtr imu_msg;
    };

    struct MessageType_ODOM_LO : public MessageType_ODOM
    {
        geometry_msgs::msg::PoseStamped::ConstSharedPtr lo_msg;
    };


    struct IMU_Data
    {
        Eigen::Vector3d acce;
        Eigen::Vector3d gyro;
    };
    
    
} // namespace anomaly_detection


#endif // SLIPDETECTION__MSGS_TYPE_HPP