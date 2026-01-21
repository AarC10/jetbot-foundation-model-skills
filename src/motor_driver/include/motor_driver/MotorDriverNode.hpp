#pragma once


#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <motor_driver/Pca9685.hpp>

class MotorDriverNode : public rclcpp::Node {
public:
    MotorDriverNode();

    ~MotorDriverNode();

private:
    Pca9685 pca9685;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;
    // TODO: Custom message
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motorStatusPub;

    const int leftMotorChannel;
    const int rightMotorChannel;

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    bool setDirection(bool forward, bool backward, int motor);
};
