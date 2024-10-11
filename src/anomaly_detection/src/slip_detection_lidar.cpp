#include "anomaly_detection/slip_detection_lidar.hpp"

using namespace anomaly_detection;


    SlipDetectionLidar::SlipDetectionLidar()
        :Node("slip_detection_node")
    {   
        this->declare_parameter<bool>("print_timestamps", false);
        this->declare_parameter<bool>("show_diff", false);
        this->declare_parameter<uint16_t>("period", 200);
        this->declare_parameter<double>("threshold_linear", 0.2);
        this->declare_parameter<double>("threshold_angle", 100); //暂不考旋转打滑

        this->get_parameter("print_timestamps", options_.print_timestamps);
        this->get_parameter("show_diff", options_.show_diff);
        this->get_parameter("period", options_.period);
        this->get_parameter("threshold_linear", options_.accepted_diff.linear);
        this->get_parameter("threshold_angle", options_.accepted_diff.angle);

        
        sync_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        sub_options_.callback_group = sync_callback_group_;

        odom_sub_.subscribe(this, "/odom", rmw_qos_profile_default, sub_options_);
        lo_sub_.subscribe(this, "/slam/topic/current_pose_from_lidar", rmw_qos_profile_default, sub_options_);

 
        sync_ = std::make_shared<odom_lo_sync>(odom_lo_sync_policy(10), odom_sub_, lo_sub_);
        sync_->registerCallback(std::bind(&SlipDetectionLidar::callback, this, std::placeholders::_1, std::placeholders::_2));
        
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(options_.period),
                std::bind(&SlipDetectionLidar::timer_callback, this), timer_callback_group_);
        
    }

    
    void SlipDetectionLidar::callback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,  
                                      const geometry_msgs::msg::PoseStamped::ConstSharedPtr &lo_msg)
    {   
        if (options_.print_timestamps)
        {
            RCLCPP_INFO(this->get_logger(), 
            "Received odom_msg time: %f, lo_msg time: %f",
            static_cast<rclcpp::Time>(odom_msg->header.stamp).seconds(),
            static_cast<rclcpp::Time>(lo_msg->header.stamp).seconds());        
        }

        std::unique_lock<std::mutex> lock_msgs(msgs_que_mtx_);
        msgs_queue_.push({odom_msg, lo_msg});
    }


    void SlipDetectionLidar::timer_callback()
    {   
        {
            std::unique_lock<std::mutex> lock_msgs(msgs_que_mtx_);
            if (msgs_queue_.empty()) return;

            std::unique_lock<std::mutex> lock_exec(exec_que_mtx_);
            while (!msgs_queue_.empty())
            {
                exec_queue_.push(msgs_queue_.front());
                msgs_queue_.pop();
            }
        }

        {   
            std::unique_lock<std::mutex> lock_exec(exec_que_mtx_);

            while (!exec_queue_.empty())
            {   
                if(!initialized_)
                {   
                    last_pose_time_ = exec_queue_.front().lo_msg->header.stamp;

                    msg_transform(exec_queue_.front().odom_msg, last_pose_odom_);
                    msg_transform(exec_queue_.front().lo_msg, last_pose_lo_);

                    initialized_ = true;
                    return;
                }
                
                if (exec_queue_.size() == 1)
                {   
                    current_pose_time_ = exec_queue_.front().lo_msg->header.stamp;
                    msg_transform(exec_queue_.front().odom_msg, current_pose_odom_);
                    msg_transform(exec_queue_.front().lo_msg, current_pose_lo_);
                }

                exec_queue_.pop();
            }      
        }

        anomaly_detection::PoseDifference pose_difference_odom;
        anomaly_detection::PoseDifference pose_difference_lo;
        calculate_pose_diff_2d(pose_difference_odom, current_pose_odom_, last_pose_odom_);
        calculate_pose_diff_2d(pose_difference_lo, current_pose_lo_, last_pose_lo_);
        anomaly_detection::PoseDifference odom_diff_norm = pose_diff_2d_per_sec(pose_difference_odom, last_pose_time_, current_pose_time_);
        anomaly_detection::PoseDifference lo_diff_norm = pose_diff_2d_per_sec(pose_difference_lo, last_pose_time_, current_pose_time_);

        if (options_.show_diff)
        {
            RCLCPP_INFO(this->get_logger(), "odom_diff between %f and %f, odom:[linear:%f], [angle:%f]", 
                        last_pose_time_.seconds(), current_pose_time_.seconds(), 
                        odom_diff_norm.linear, odom_diff_norm.angle);
            
            RCLCPP_INFO(this->get_logger(), "lo_diff between %f and %f, lo:[linear:%f], [angle:%f]", 
                        last_pose_time_.seconds(), current_pose_time_.seconds(), 
                        lo_diff_norm.linear, lo_diff_norm.angle);
        }

        if (compair_diff_2d(options_, odom_diff_norm, lo_diff_norm))
        {
            RCLCPP_INFO(this->get_logger(), "Detect slip (odom & lo) at %f", last_pose_time_.seconds());
        }


        last_pose_time_ = current_pose_time_;
        last_pose_odom_ = current_pose_odom_;
        last_pose_lo_ = current_pose_lo_;
    }


