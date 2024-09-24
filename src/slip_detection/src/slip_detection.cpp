#include "slip_detection/slip_detection.hpp"

using namespace anomaly_detection;

SlipDetection::SlipDetection()
    :Node("slip_detection_node")
{
    odom_sub_.subscribe(this, "/odom");
    lo_sub_.subscribe(this, "/slam/topic/current_pose_from_lidar");
    imu_sub_.subscribe(this, "/imu_data");

    // sync_ = std::make_shared<odom_lo_sync>(odom_lo_sync_policy(10), odom_sub_, lo_sub_);
    // sync_->registerCallback(std::bind(&SlipDetection::callback, this, std::placeholders::_1, std::placeholders::_2));
    sync_ = std::make_shared<odom_lo_imu_sync>(odom_lo_imu_sync_policy(10), odom_sub_, lo_sub_, imu_sub_);
    sync_->registerCallback(std::bind(&SlipDetection::callback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(options_.period),
            std::bind(&SlipDetection::timer_callback, this));
}

void SlipDetection::callback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg, 
                             const geometry_msgs::msg::PoseStamped::ConstSharedPtr &pose_msg, 
                             const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg)
{
    if(!initialized_)
    {   
        // last_pose_time_ = pose_msg->header.stamp;
        last_pose_time_ = imu_msg->header.stamp;
        msg_transform(odom_msg, last_pose_odom_);
        msg_transform(pose_msg, last_pose_lo_);

        initialized_ = true;
        return;
    }

    // current_pose_time_ = pose_msg->header.stamp;
    current_pose_time_ = imu_msg->header.stamp;
    msg_transform(odom_msg, current_pose_odom_);
    msg_transform(pose_msg, current_pose_lo_);
    calculate_imu_integration(imu_msg, intergtated_pose_imu_, last_pose_time_, current_pose_time_);
}

void SlipDetection::timer_callback()
{
    if (!initialized_) return;

    
    anomaly_detection::PoseDifference pose_difference_odom;
    anomaly_detection::PoseDifference pose_difference_lo;
    anomaly_detection::PoseDifference pose_difference_imu;
    
    calculate_pose_diff_2d(pose_difference_odom, current_pose_odom_, last_pose_odom_);
    calculate_pose_diff_2d(pose_difference_lo, current_pose_lo_, last_pose_lo_);
    calculate_pose_diff_2d(pose_difference_imu, intergtated_pose_imu_);

    if (compair_diff_2d(options_, pose_difference_odom, pose_difference_lo))
    {
        RCLCPP_INFO(this->get_logger(), "Detect slip (odom & lidar) at %f", last_pose_time_.seconds());
    }

    if (compair_diff_2d(options_, pose_difference_odom, pose_difference_imu))
    {
        RCLCPP_INFO(this->get_logger(), "Detect slip (odom & imu) at %f", last_pose_time_.seconds());
    }

    last_pose_time_ = current_pose_time_;
    last_pose_odom_ = current_pose_odom_;
    last_pose_lo_ = current_pose_lo_;
    intergtated_pose_imu_.reset();
}


