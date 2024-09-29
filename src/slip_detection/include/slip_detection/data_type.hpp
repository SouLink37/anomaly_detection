#ifndef SLIPDETECTION__MSGS_TYPE_HPP
#define SLIPDETECTION__MSGS_TYPE_HPP
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace anomaly_detection
{   
    struct MessageType_ODOM_IMU 
    {
        nav_msgs::msg::Odometry::ConstSharedPtr odom_msg;
        sensor_msgs::msg::Imu::ConstSharedPtr imu_msg;
    };

    struct IMU_Data
    {
        Eigen::Vector3d acce;
        Eigen::Vector3d gyro;
    };
    
    
} // namespace anomaly_detection


#endif // SLIPDETECTION__MSGS_TYPE_HPP