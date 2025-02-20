#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "wheel_odometry/wheel_odometry.hpp"

using liesun::sentry::lidar_mapping::WheelOdometry;

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelOdometry>());
    rclcpp::shutdown();
    return 0;
}