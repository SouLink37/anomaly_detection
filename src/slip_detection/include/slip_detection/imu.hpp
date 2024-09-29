#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "slip_detection/data_type.hpp"

namespace anomaly_detection
{
    class IMU
    {   
        public:

        IMU(const Eigen::Vector3d &gravity, const Eigen::Vector3d &ba, const Eigen::Vector3d &bg)
            : gravity_(gravity), ba_(ba), bg_(bg) {}
        IMU() : IMU(Eigen::Vector3d(0, 0, -9.8), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()) {}

        void imu_intergration(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg,
                              anomaly_detection::Pose<Orietation_xyz> &intergtated_pose_imu,
                              const rclcpp::Time last_pose_time, 
                              const rclcpp::Time current_pose_time,
                              anomaly_detection::Velocity &velocity)
        {
            double dt = (current_pose_time - last_pose_time).seconds(); 
            anomaly_detection::IMU_Data eigen_imu = rosmsg2eigen(imu_msg);
            const Eigen::Vector3d& acce = eigen_imu.acce;
            const Eigen::Vector3d& gyro = eigen_imu.gyro;
            v_ = xyz2eigen(velocity);

            Eigen::Vector3d corrected_gyro = gyro - bg_;

            p_ = p_ + v_ * dt + 0.5 * gravity_ * dt * dt + 0.5 * (R_ * (acce - ba_)) * dt * dt;
            v_ = v_ + R_ * (acce - ba_) * dt + gravity_ * dt;
            Eigen::Vector3d delta_theta = corrected_gyro * dt;
            if (delta_theta.norm() > 1e-6) 
            {
                Eigen::AngleAxisd angle_axis(delta_theta.norm(), delta_theta.normalized());
                Eigen::Matrix3d delta_R = angle_axis.toRotationMatrix();
                R_ = (R_ * delta_R).normalized();
            }

            velocity = eigen2xyz(v_);
            intergtated_pose_imu.position.x = p_[0];
            intergtated_pose_imu.position.y = p_[1];   
            intergtated_pose_imu.position.z = p_[2];

            intergtated_pose_imu.orientation.x = delta_theta[0];
            intergtated_pose_imu.orientation.y = delta_theta[1];   
            intergtated_pose_imu.orientation.z = delta_theta[2];                        
        }


        
        private:

        anomaly_detection::IMU_Data rosmsg2eigen(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
        {
            return {
                {imu_msg->linear_acceleration.x * 9.8,
                 imu_msg->linear_acceleration.y * 9.8,
                 imu_msg->linear_acceleration.z * 9.8},

                {imu_msg->angular_velocity.x,
                 imu_msg->angular_velocity.y,
                 imu_msg->angular_velocity.z}
            };
        }

        Eigen::Vector3d xyz2eigen(const anomaly_detection::Velocity &xyz)
        {
            return {xyz.x, xyz.y, xyz.z};
        }

        anomaly_detection::Velocity eigen2xyz(const Eigen::Vector3d &vec3d)
        {
            return {vec3d[0], vec3d[1], vec3d[2]};
        }


        Eigen::Vector3d p_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d v_ = Eigen::Vector3d::Zero();
        Eigen::Matrix3d R_ = Eigen::Matrix3d::Identity();

        Eigen::Vector3d gravity_;
        Eigen::Vector3d ba_;
        Eigen::Vector3d bg_;
    };

} // namespace anomaly_detection