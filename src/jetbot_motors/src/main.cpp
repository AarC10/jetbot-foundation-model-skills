#include <rclcpp/rclcpp.hpp>
#include "jetbot_motors/MotorNode.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<jetbot_motors::MotorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
