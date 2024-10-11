#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "anomaly_detection/pose.hpp"
#include "anomaly_detection/options.hpp"


namespace anomaly_detection
{   
    // using velocity = anomaly_detection::Position;

    void msg_transform(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg, 
                       anomaly_detection::Pose<Orietation_xyzw> &pose);
    void msg_transform(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg, 
                       anomaly_detection::Pose<Orietation_xyzw> &pose);

    void calculate_pose_diff_2d(anomaly_detection::PoseDifference &pose_difference, 
                                const anomaly_detection::Pose<Orietation_xyzw> &pose_1, 
                                const anomaly_detection::Pose<Orietation_xyzw> &pose_2);

    void calculate_pose_diff_2d(anomaly_detection::PoseDifference &pose_difference_imu, 
                                const anomaly_detection::Pose<Orietation_xyz> &intergtated_pose_imu_);

    auto pose_diff_2d_per_sec(const anomaly_detection::PoseDifference &pose_difference, 
                              const rclcpp::Time last_pose_time, 
                              const rclcpp::Time current_pose_time) 
                              -> anomaly_detection::PoseDifference;

    bool compair_diff_2d(const anomaly_detection::Options &options,
                         const anomaly_detection::PoseDifference &pose_difference_1, 
                         const anomaly_detection::PoseDifference &pose_difference_2);

    double yaw_angle_diff(const anomaly_detection::Pose<Orietation_xyzw> &pose_1, 
                          const anomaly_detection::Pose<Orietation_xyzw> &pose_2);
    double compute_yaw(const anomaly_detection::Orietation_xyzw &orientation);

    auto get_vel_from_odom(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) -> anomaly_detection::Velocity;
} // namespace anomaly_detection