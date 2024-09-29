#include "slip_detection/slip_detection.hpp"

using namespace anomaly_detection;


    SlipDetection::SlipDetection()
        :Node("slip_detection_node")
    {   
        this->declare_parameter<bool>("print_timestamps", false);
        this->declare_parameter<bool>("show_diff", false);
        this->declare_parameter<uint16_t>("period", 200);
        this->declare_parameter<double>("threshold_linear", 0.05);
        this->declare_parameter<double>("threshold_angle", 100); //暂不考虑角度差

        this->get_parameter("print_timestamps", options_.print_timestamps);
        this->get_parameter("show_diff", options_.show_diff);
        this->get_parameter("period", options_.period);
        this->get_parameter("threshold_linear", options_.accepted_diff.linear);
        this->get_parameter("threshold_angle", options_.accepted_diff.angle);

        
        sync_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        sub_options_.callback_group = sync_callback_group_;

        odom_sub_.subscribe(this, "/odom", rmw_qos_profile_default, sub_options_);
        imu_sub_.subscribe(this, "/imu_data", rmw_qos_profile_default, sub_options_);

 
        sync_ = std::make_shared<odom_imu_sync>(odom_imu_sync_policy(10), odom_sub_, imu_sub_);
        sync_->registerCallback(std::bind(&SlipDetection::callback, this, std::placeholders::_1, std::placeholders::_2));
        
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(options_.period),
                std::bind(&SlipDetection::timer_callback, this), timer_callback_group_);
        
    }

    
    void SlipDetection::callback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,  
                                const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg)
    {   
        if (options_.print_timestamps)
        {
            RCLCPP_INFO(this->get_logger(), 
            "Received odom_msg time: %f, imu_msg time: %f",
            static_cast<rclcpp::Time>(odom_msg->header.stamp).seconds(),
            static_cast<rclcpp::Time>(imu_msg->header.stamp).seconds());        
        }

        std::unique_lock<std::mutex> lock(msgs_que_mtx_);
        msgs_queue_.push({odom_msg, imu_msg});
    }


    void SlipDetection::timer_callback()
    {   
        {
            std::unique_lock<std::mutex> lock(msgs_que_mtx_);

            if (msgs_queue_.empty()) return;

            imu_ = std::make_shared<anomaly_detection::IMU>();

            while (!msgs_queue_.empty())
            {
                if(!initialized_)
                {   
                    last_pose_time_ = msgs_queue_.front().imu_msg->header.stamp;
                    last_imu_time_ = last_pose_time_;
                    last_odom_time_ = msgs_queue_.front().odom_msg->header.stamp;
                    odom_velocity_ = get_vel_from_odom(msgs_queue_.front().odom_msg);
                    msg_transform(msgs_queue_.front().odom_msg, last_pose_odom_);

                    initialized_ = true;
                    return;
                }

                current_pose_time_ = msgs_queue_.front().imu_msg->header.stamp;
                current_imu_time_ = current_pose_time_;
                current_odom_time_ = msgs_queue_.front().odom_msg->header.stamp;
                
                if (current_odom_time_.seconds() - last_odom_time_.seconds() > 0.5)
                {
                    odom_velocity_ = get_vel_from_odom(msgs_queue_.front().odom_msg);
                }
                imu_->imu_intergration(msgs_queue_.front().imu_msg, intergtated_pose_imu_, last_imu_time_, current_imu_time_, odom_velocity_);
                // calculate_imu_integration(msgs_queue_.front().imu_msg, intergtated_pose_imu_, last_pose_time_, current_pose_time_);
                
                if (msgs_queue_.size() == 1)
                {
                    msg_transform(msgs_queue_.front().odom_msg, current_pose_odom_);
                }

                last_imu_time_ = current_pose_time_;
                msgs_queue_.pop();
            }      
        }

        anomaly_detection::PoseDifference pose_difference_odom;
        anomaly_detection::PoseDifference pose_difference_imu;
        calculate_pose_diff_2d(pose_difference_odom, current_pose_odom_, last_pose_odom_);
        calculate_pose_diff_2d(pose_difference_imu, intergtated_pose_imu_);
        anomaly_detection::PoseDifference odom_diff_norm = pose_diff_2d_per_sec(pose_difference_odom, last_pose_time_, current_pose_time_);
        anomaly_detection::PoseDifference imu_diff_norm = pose_diff_2d_per_sec(pose_difference_imu, last_pose_time_, current_pose_time_);

        if (options_.show_diff)
        {
            RCLCPP_INFO(this->get_logger(), "odom_diff between %f and %f, odom:[linear:%f], [angle:%f]", 
                        last_pose_time_.seconds(), current_pose_time_.seconds(), 
                        odom_diff_norm.linear, odom_diff_norm.angle);
            
            RCLCPP_INFO(this->get_logger(), "imu_diff between %f and %f, imu:[linear:%f], [angle:%f]", 
                        last_pose_time_.seconds(), current_pose_time_.seconds(), 
                        imu_diff_norm.linear, imu_diff_norm.angle);
        }

        if (compair_diff_2d(options_, odom_diff_norm, imu_diff_norm))
        {
            RCLCPP_INFO(this->get_logger(), "Detect slip (odom & imu) at %f", last_pose_time_.seconds());
        }


        last_pose_time_ = current_pose_time_;
        last_pose_odom_ = current_pose_odom_;
        intergtated_pose_imu_.reset();
    }


