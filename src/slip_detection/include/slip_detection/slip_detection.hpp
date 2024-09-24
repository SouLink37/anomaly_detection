#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "slip_detection/options.hpp"
#include "slip_detection/pose.hpp"
#include "slip_detection/format_and_math.hpp"



namespace anomaly_detection
{   
    // typedef message_filters::sync_policies::ApproximateTime
    //         <nav_msgs::msg::Odometry, geometry_msgs::msg::PoseStamped> odom_lo_sync_policy;
    // typedef message_filters::Synchronizer<odom_lo_sync_policy> odom_lo_sync;

    typedef message_filters::sync_policies::ApproximateTime
            <nav_msgs::msg::Odometry, geometry_msgs::msg::PoseStamped, sensor_msgs::msg::Imu> odom_lo_imu_sync_policy;
    typedef message_filters::Synchronizer<odom_lo_imu_sync_policy> odom_lo_imu_sync;

    class SlipDetection : public rclcpp::Node
    {
    public:
        SlipDetection();
        
    private:

        // void callback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg, 
                     //  const geometry_msgs::msg::PoseStamped::ConstSharedPtr &pose_msg);
        void callback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg, 
                      const geometry_msgs::msg::PoseStamped::ConstSharedPtr &pose_msg, 
                      const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg);
        void timer_callback();

        message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
        message_filters::Subscriber<geometry_msgs::msg::PoseStamped> lo_sub_; //lidar odometry
        message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;

        rclcpp::TimerBase::SharedPtr timer_;
        // std::shared_ptr<odom_lo_sync> sync_;
        std::shared_ptr<odom_lo_imu_sync> sync_;


        anomaly_detection::Options options_;
        bool initialized_ = false;

        rclcpp::Time last_pose_time_;
        rclcpp::Time current_pose_time_;

        anomaly_detection::Pose<Orietation_xyzw> last_pose_odom_;
        anomaly_detection::Pose<Orietation_xyzw> last_pose_lo_;
        anomaly_detection::Pose<Orietation_xyzw> current_pose_odom_;
        anomaly_detection::Pose<Orietation_xyzw> current_pose_lo_; //lidar odometry
        anomaly_detection::Pose<Orietation_xyz> intergtated_pose_imu_;

    };
} // namespace slip_detection

