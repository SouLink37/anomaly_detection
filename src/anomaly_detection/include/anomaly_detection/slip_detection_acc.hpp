#include <mutex>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "anomaly_detection/options.hpp"
#include "anomaly_detection/pose.hpp"
#include "anomaly_detection/format_and_math.hpp"
#include "anomaly_detection/data_type.hpp"
#include "anomaly_detection/imu.hpp"
#include "anomaly_detection/msg/sensor_data.hpp"
#include "anomaly_detection/msg/sensor_data_header.hpp"

namespace anomaly_detection
{   
    typedef message_filters::sync_policies::ApproximateTime
            <anomaly_detection::msg::SensorDataHeader, sensor_msgs::msg::Imu> sensor_imu_sync_policy;
    typedef message_filters::Synchronizer<sensor_imu_sync_policy> sensor_imu_sync;

    class SlipDetectionAcc : public rclcpp::Node
    {
    public:
        SlipDetectionAcc();
        
    private:

        void callback(const anomaly_detection::msg::SensorDataHeader::ConstSharedPtr &sensor_msg,  
                      const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg);
        void timer_callback();

        message_filters::Subscriber<anomaly_detection::msg::SensorDataHeader> sensor_sub_;
        message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;

        rclcpp::TimerBase::SharedPtr timer_;

        std::shared_ptr<sensor_imu_sync> sync_;

        anomaly_detection::Options options_;
        bool initialized_ = false;


        rclcpp::SubscriptionOptions sub_options_;
        rclcpp::CallbackGroup::SharedPtr sync_callback_group_;
        rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
        
        rclcpp::Time last_pose_time_;
        rclcpp::Time current_pose_time_;

        double last_accx_imu_;
        double current_accx_imu_;
        double last_vell_sensor_;
        double current_vell_sensor_;
        double last_velr_sensor_;
        double current_velr_sensor_;

        std::queue<MessageType_SENSOR_IMU> msgs_queue_;
        std::queue<MessageType_SENSOR_IMU> exec_queue_;
        std::mutex msgs_que_mtx_;
        std::mutex exec_que_mtx_;
    };
} // namespace anomaly_detection

