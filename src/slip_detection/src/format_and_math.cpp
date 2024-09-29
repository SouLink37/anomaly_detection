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

// void anomaly_detection::calculate_imu_integration(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg,
//                                                   anomaly_detection::Pose<Orietation_xyz> &intergtated_pose_imu,
//                                                   const rclcpp::Time last_pose_time, 
//                                                   const rclcpp::Time current_pose_time)
// {
//     double dt = (current_pose_time - last_pose_time).seconds(); 

//     intergtated_pose_imu.position.x += 0.5 * imu_msg->linear_acceleration.x * dt * dt * 9.8; 
//     intergtated_pose_imu.position.y += 0.5 * imu_msg->linear_acceleration.y * dt * dt * 9.8;
//     intergtated_pose_imu.position.z += 0.5 * (imu_msg->linear_acceleration.z - 1)  * dt * dt * 9.8;

//     intergtated_pose_imu.orientation.x += imu_msg->angular_velocity.x * dt;
//     intergtated_pose_imu.orientation.y += imu_msg->angular_velocity.y * dt;
//     intergtated_pose_imu.orientation.z += imu_msg->angular_velocity.z * dt;
// }

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
    pose_difference.angle = fabs(intergtated_pose.orientation.z);
}

bool anomaly_detection::compair_diff_2d(const anomaly_detection::Options &options,
                                        const anomaly_detection::PoseDifference &pose_difference_1, 
                                        const anomaly_detection::PoseDifference &pose_difference_2)
{   
    double linear_diff = pose_difference_1.linear - pose_difference_2.linear;
    double angle_diff = pose_difference_1.angle - pose_difference_2.angle;

    bool linear_condition = linear_diff > options.accepted_diff.linear;
    bool angle_condition = angle_diff > options.accepted_diff.angle;

    if (linear_condition || angle_condition) return true;
    
    return false;
}

auto anomaly_detection::pose_diff_2d_per_sec(const anomaly_detection::PoseDifference &pose_difference, 
                              const rclcpp::Time last_pose_time, 
                              const rclcpp::Time current_pose_time) 
                              -> anomaly_detection::PoseDifference
{   
    double dt = (current_pose_time - last_pose_time).seconds();

    return {pose_difference.linear / dt, pose_difference.angle / dt};
}

double anomaly_detection::compute_yaw(const anomaly_detection::Orietation_xyzw &orientation) 
{
    double w = orientation.w;
    double x = orientation.x;
    double y = orientation.y;
    double z = orientation.z;
    return atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

double anomaly_detection::yaw_angle_diff(const anomaly_detection::Pose<Orietation_xyzw> &pose_1, const anomaly_detection::Pose<Orietation_xyzw> &pose_2) 
{
    double yaw1 = compute_yaw(pose_1.orientation);
    double yaw2 = compute_yaw(pose_2.orientation);
    double delta_yaw = yaw2 - yaw1;
    
    delta_yaw = fmod(delta_yaw + M_PI, 2 * M_PI) - M_PI;
    return fabs(delta_yaw);
}

// double anomaly_detection::yaw_angle_diff(const anomaly_detection::Pose<Orietation_xyzw> &pose_1, const anomaly_detection::Pose<Orietation_xyzw> &pose_2) 
// {
//     return fabs(pose_1.orientation.z - pose_2.orientation.z);
// }

auto anomaly_detection::get_vel_from_odom(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg) -> anomaly_detection::Velocity
{
    return {odom_msg->twist.twist.linear.x,
            odom_msg->twist.twist.linear.y,
            odom_msg->twist.twist.linear.z};
}


