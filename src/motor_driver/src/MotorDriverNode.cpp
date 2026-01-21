#include <motor_driver/MotorDriverNode.hpp>

MotorDriverNode::MotorDriverNode() : Node("motor_driver_node"), leftMotorChannel(0), rightMotorChannel(1) {
    if (!pca9865.setPwmFrequency(0)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set PWM frequency on PCA9865");
    }

    cmdVelSub = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&MotorDriverNode::cmdVelCallback, this, std::placeholders::_1)
    );

    motorStatusPub = this->create_publisher<geometry_msgs::msg::Twist>("motor_status", 10);
}

