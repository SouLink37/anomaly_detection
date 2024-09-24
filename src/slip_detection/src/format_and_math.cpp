#include<cmath>
#include "slip_detection/format_and_math.hpp"


void anomaly_detection::msg_transform(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg, 
                                      anomaly_detection::Pose<Orietation_xyzw> &pose)
{
    pose.position.x = odom_msg->pose.pose.position.x;
    pose.position.y = odom_msg->pose.pose.position.y;
    pose.position.z = odom_msg->pose.pose.position.z;

    pose.orientation.x = odom_msg->pose.pose.orientation.x;
    pose.orientation.y = odom_msg->pose.pose.orientation.y;
    pose.orientation.z = odom_msg->pose.pose.orientation.z;
    pose.orientation.w = odom_msg->pose.pose.orientation.w;
}

void anomaly_detection::msg_transform(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg, 
                                      anomaly_detection::Pose<Orietation_xyzw> &pose)
{
    pose.position.x = pose_msg->pose.position.x;
    pose.position.y = pose_msg->pose.position.y;
    pose.position.z = pose_msg->pose.position.z;

    pose.orientation.x = pose_msg->pose.orientation.x;
    pose.orientation.y = pose_msg->pose.orientation.y;
    pose.orientation.z = pose_msg->pose.orientation.z;
    pose.orientation.w = pose_msg->pose.orientation.w;
}

void anomaly_detection::calculate_imu_integration(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg,
                                                  anomaly_detection::Pose<Orietation_xyz> &intergtated_pose_imu,
                                                  const rclcpp::Time last_pose_time_, 
                                                  const rclcpp::Time current_pose_time_)
{
    double dt = (current_pose_time_ - last_pose_time_).seconds(); 

    intergtated_pose_imu.position.x += 0.5 * imu_msg->linear_acceleration.x * dt * dt; 
    intergtated_pose_imu.position.y += 0.5 * imu_msg->linear_acceleration.y * dt * dt;
    intergtated_pose_imu.position.z += 0.5 * imu_msg->linear_acceleration.z * dt * dt;

    intergtated_pose_imu.orientation.x += imu_msg->angular_velocity.x * dt;
    intergtated_pose_imu.orientation.y += imu_msg->angular_velocity.y * dt;
    intergtated_pose_imu.orientation.z += imu_msg->angular_velocity.z * dt;
}

void anomaly_detection::calculate_pose_diff_2d(anomaly_detection::PoseDifference &pose_difference, 
                                               const anomaly_detection::Pose<Orietation_xyzw> &pose_1, 
                                               const anomaly_detection::Pose<Orietation_xyzw> &pose_2)
{
    pose_difference.linear = sqrt(pow(pose_1.position.x - pose_2.position.x, 2) + pow(pose_1.position.y - pose_2.position.y, 2));

    pose_difference.angle = yaw_angle_diff(pose_1, pose_2);
}

void anomaly_detection::calculate_pose_diff_2d(anomaly_detection::PoseDifference &pose_difference, 
                                               const anomaly_detection::Pose<Orietation_xyz> &intergtated_pose)
{
    pose_difference.linear = sqrt(pow(intergtated_pose.position.x, 2) + pow(intergtated_pose.position.y, 2));
    pose_difference.angle = intergtated_pose.orientation.z;
}

bool anomaly_detection::compair_diff_2d(const anomaly_detection::Options &options,
                                        const anomaly_detection::PoseDifference &pose_difference_1, 
                                        const anomaly_detection::PoseDifference &pose_difference_2)
{   
    double linear_diff = fabs(pose_difference_1.linear - pose_difference_2.linear);
    double angle_diff = fabs(pose_difference_1.angle - pose_difference_2.angle);

    if (linear_diff > options.accepted_diff.linear || angle_diff > options.accepted_diff.angle) return true;
    
    return false;
}

double anomaly_detection::yaw_angle_diff(const anomaly_detection::Pose<Orietation_xyzw> &pose_1, const anomaly_detection::Pose<Orietation_xyzw> &pose_2)
{
    return 2 * acos(fabs(dot_2d(pose_1, pose_2)));
}

double anomaly_detection::dot_2d(const anomaly_detection::Pose<Orietation_xyzw> &pose_1, const anomaly_detection::Pose<Orietation_xyzw> &pose_2) 
{
    return pose_1.orientation.w * pose_2.orientation.w + pose_1.orientation.x + pose_2.orientation.x;
}