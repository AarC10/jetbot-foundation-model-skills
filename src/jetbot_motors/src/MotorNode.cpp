#include "jetbot_motors/MotorNode.hpp"

#include <functional>

MotorNode::MotorNode() : Node("jetbot_motors_node") {
    // 1600 khz similar to Motorhat driver from Adafruit
    if (!pca9685.setPwmFrequency(1600)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set PWM frequency on PCA9685");
    }

    cmdVelSub = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&MotorNode::cmdVelCallback, this, std::placeholders::_1));

    motorStatusPub = this->create_publisher<geometry_msgs::msg::Twist>("motor_status", 10);
}

MotorNode::~MotorNode() = default;

void MotorNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    const double velMps = msg->linear.x;
    const double omegaRps = msg->angular.z;

    const WheelLinear wheels = computeWheelSpeeds(velMps, omegaRps);
    const double leftNorm = normalizeWheelSpeed(wheels.leftMps, MAX_WHEEL_MPS);
    const double rightNorm = normalizeWheelSpeed(wheels.rightMps, MAX_WHEEL_MPS);

    const MotorDirection leftDir = directionFromCmd(leftNorm);
    const MotorDirection rightDir = directionFromCmd(rightNorm);

    const uint16_t leftPwm = (leftDir == STOP || leftDir == COAST) ? 0 : pwmFromNormalized(leftNorm);
    const uint16_t rightPwm = (rightDir == STOP || rightDir == COAST) ? 0 : pwmFromNormalized(rightNorm);

    if (!setDirection(leftDir, leftMotor)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set direction for left motor");
    } else if (!pca9685.setPwm(leftMotor.pwm, 0, leftPwm)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set PWM for left motor");
    }

    if (!setDirection(rightDir, rightMotor)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set direction for right motor");
    } else if (!pca9685.setPwm(rightMotor.pwm, 0, rightPwm)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set PWM for right motor");
    }

    geometry_msgs::msg::Twist status;
    status.linear.x = wheels.leftMps;
    status.angular.z = wheels.rightMps;
    motorStatusPub->publish(status);
}

bool MotorNode::setDirection(MotorDirection direction, const MotorChannels &motor) {
    const auto it = motorDirectionMap.find(direction);
    if (it == motorDirectionMap.end()) {
        RCLCPP_ERROR(this->get_logger(), "Invalid motor direction");
        return false;
    }

    const MotorDirectionBits &bits = it->second;

    if (!pca9685.setPin(motor.analogIn1, bits.in1High)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set IN1 for motor");
        return false;
    }
    if (!pca9685.setPin(motor.analogIn2, bits.in2High)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set IN2 for motor");
        return false;
    }

    return true;
}
