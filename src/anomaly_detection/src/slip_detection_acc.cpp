#include "anomaly_detection/slip_detection_acc.hpp"

using namespace anomaly_detection;


    SlipDetectionAcc::SlipDetectionAcc()
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

        sensor_sub_.subscribe(this, "/sensor_data_with_head", rmw_qos_profile_default, sub_options_);
        imu_sub_.subscribe(this, "/imu_data", rmw_qos_profile_default, sub_options_);

 
        sync_ = std::make_shared<sensor_imu_sync>(sensor_imu_sync_policy(10), sensor_sub_, imu_sub_);
        sync_->registerCallback(std::bind(&SlipDetectionAcc::callback, this, std::placeholders::_1, std::placeholders::_2));
        
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(options_.period),
                std::bind(&SlipDetectionAcc::timer_callback, this), timer_callback_group_);
        
    }

    
    void SlipDetectionAcc::callback(const anomaly_detection::msg::SensorDataHeader::ConstSharedPtr &sensor_msg,  
                                    const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg)
    {   
        if (options_.print_timestamps)
        {
            RCLCPP_INFO(this->get_logger(), 
            "Received sensor_msg time: %f, imu_msg time: %f",
            static_cast<rclcpp::Time>(sensor_msg->header.stamp).seconds(),
            // "Received sensor_msg time: %lu.%lu, imu_msg time: %f",
            // sensor_msg->time_ses, sensor_msg->time_nsec,
            static_cast<rclcpp::Time>(imu_msg->header.stamp).seconds());        
        }

        std::unique_lock<std::mutex> lock_msgs(msgs_que_mtx_);
        msgs_queue_.push({sensor_msg, imu_msg});
    }


    void SlipDetectionAcc::timer_callback()
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
                    last_pose_time_ = exec_queue_.front().imu_msg->header.stamp;
                    last_accx_imu_ = exec_queue_.front().imu_msg->linear_acceleration.x;
                    last_vell_sensor_ = exec_queue_.front().sensor_msg->l_speed;
                    last_velr_sensor_ = exec_queue_.front().sensor_msg->r_speed;
                    initialized_ = true;
                    return;
                }
                
                if (exec_queue_.size() == 1)
                {   
                    current_pose_time_ = exec_queue_.front().imu_msg->header.stamp;
                    current_accx_imu_ = exec_queue_.front().imu_msg->linear_acceleration.x;
                    current_vell_sensor_ = exec_queue_.front().sensor_msg->l_speed;
                    current_velr_sensor_ = exec_queue_.front().sensor_msg->r_speed;
                }

                exec_queue_.pop();
            }      
        }

        
        double current_velx_sensor = (current_vell_sensor_ + current_velr_sensor_) * 0.5;
        double last_velx_sensor = (last_vell_sensor_ + last_velr_sensor_) * 0.5;
        double current_accx_sensor = (current_velx_sensor - last_velx_sensor) / (current_pose_time_ - last_pose_time_).seconds();


        if (options_.show_diff)
        {
            RCLCPP_INFO(this->get_logger(), "acceleration_x at %f, imu:%f, sensor%f", 
                        current_pose_time_.seconds(), 
                        current_accx_imu_, current_accx_sensor);
        }

        if (fabs(current_accx_sensor - current_accx_imu_) > options_.accepted_diff.linear)
        {
            RCLCPP_INFO(this->get_logger(), "Detect slip (odom & lo) at %f", last_pose_time_.seconds());
        }

        last_pose_time_ = current_pose_time_;
    }


