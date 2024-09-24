#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "slip_detection/pose.hpp"
#include "slip_detection/options.hpp"

namespace anomaly_detection
{
    void msg_transform(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg, 
                       anomaly_detection::Pose<Orietation_xyzw> &pose);
    void msg_transform(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg, 
                       anomaly_detection::Pose<Orietation_xyzw> &pose);

    void calculate_imu_integration(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg,
                                   anomaly_detection::Pose<Orietation_xyz> &intergtated_pose_imu,
                                   const rclcpp::Time last_pose_time_, 
                                   const rclcpp::Time current_pose_time_);

    void calculate_pose_diff_2d(anomaly_detection::PoseDifference &pose_difference, 
                                const anomaly_detection::Pose<Orietation_xyzw> &pose_1, 
                                const anomaly_detection::Pose<Orietation_xyzw> &pose_2);

    void calculate_pose_diff_2d(anomaly_detection::PoseDifference &pose_difference_imu, 
                                const anomaly_detection::Pose<Orietation_xyz> &intergtated_pose_imu_);

    bool compair_diff_2d(const anomaly_detection::Options &options,
                         const anomaly_detection::PoseDifference &pose_difference_1, 
                         const anomaly_detection::PoseDifference &pose_difference_2);

    double yaw_angle_diff(const anomaly_detection::Pose<Orietation_xyzw> &pose_1, 
                          const anomaly_detection::Pose<Orietation_xyzw> &pose_2);
    double dot_2d(const anomaly_detection::Pose<Orietation_xyzw> &pose_1, 
                  const anomaly_detection::Pose<Orietation_xyzw> &pose_2);

} // namespace anomaly_detection
