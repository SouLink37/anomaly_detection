#include "slip_detection/slip_detection_imu.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    auto anomaly_detection_node = std::make_shared<anomaly_detection::SlipDetectionIMU>();

    executor.add_node(anomaly_detection_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}