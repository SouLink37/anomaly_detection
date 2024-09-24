#include "slip_detection/slip_detection.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<anomaly_detection::SlipDetection>());
    rclcpp::shutdown();
    return 0;
}