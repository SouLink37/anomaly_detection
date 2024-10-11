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


namespace anomaly_detection
{   
    typedef message_filters::sync_policies::ApproximateTime
            <nav_msgs::msg::Odometry, geometry_msgs::msg::PoseStamped> odom_lo_sync_policy;
    typedef message_filters::Synchronizer<odom_lo_sync_policy> odom_lo_sync;

    class SlipDetectionLidar : public rclcpp::Node
    {
    public:
        SlipDetectionLidar();
        
    private:

        void callback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,  
                      const geometry_msgs::msg::PoseStamped::ConstSharedPtr &lo_msg);
        void timer_callback();

        message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
        message_filters::Subscriber<geometry_msgs::msg::PoseStamped> lo_sub_;

        rclcpp::TimerBase::SharedPtr timer_;

        std::shared_ptr<odom_lo_sync> sync_;

        anomaly_detection::Options options_;
        bool initialized_ = false;

        rclcpp::SubscriptionOptions sub_options_;
        rclcpp::CallbackGroup::SharedPtr sync_callback_group_;
        rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
        
        rclcpp::Time last_pose_time_;
        rclcpp::Time current_pose_time_;

        anomaly_detection::Pose<Orietation_xyzw> last_pose_odom_;
        anomaly_detection::Pose<Orietation_xyzw> current_pose_odom_;
        anomaly_detection::Pose<Orietation_xyzw> last_pose_lo_;
        anomaly_detection::Pose<Orietation_xyzw> current_pose_lo_;

        std::queue<MessageType_ODOM_LO> msgs_queue_;
        std::queue<MessageType_ODOM_LO> exec_queue_;
        std::mutex msgs_que_mtx_;
        std::mutex exec_que_mtx_;
    };
} // namespace anomaly_detection

