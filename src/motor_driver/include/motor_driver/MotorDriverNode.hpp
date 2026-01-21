#pragma once


#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <motor_driver/Pca9865.hpp>

class MotorDriverNode : public rclcpp::Node {
public:
    MotorDriverNode();

    ~MotorDriverNode();

private:
    Pca9865 pca9865;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;
    // TODO: Custom message
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motorStatusPub;

    int leftMotorChannel;
    int rightMotorChannel;

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
};
