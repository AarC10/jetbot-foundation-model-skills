#include <motor_driver/MotorDriverNode.hpp>

MotorDriverNode::MotorDriverNode() : Node("motor_driver_node"), leftMotorChannel(0), rightMotorChannel(1) {
    if (!pca9685.setPwmFrequency(0)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set PWM frequency on PCA9685");
    }

    cmdVelSub = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&MotorDriverNode::cmdVelCallback, this, std::placeholders::_1)
    );

    motorStatusPub = this->create_publisher<geometry_msgs::msg::Twist>("motor_status", 10);
}

MotorDriverNode::~MotorDriverNode() {}

void MotorDriverNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // This is a diff drive robot, so we convert linear and angular velocity to left and right motor speeds
    static constexpr float MAX_SPEED = 1.0; // TODO: Get the actual number


    const double linear = msg->linear.x;
    const double angular = msg->angular.z;

    const double leftSpeed = linear - angular;
    const double rightSpeed = linear + angular;

    const double leftClamped = std::max(-static_cast<double>(MAX_SPEED), std::min(static_cast<double>(MAX_SPEED), leftSpeed));
    const double rightClamped = std::max(-static_cast<double>(MAX_SPEED), std::min(static_cast<double>(MAX_SPEED), rightSpeed));

    const uint16_t leftPwm = static_cast<uint16_t>((leftClamped / MAX_SPEED + 1.0) / 2.0 * MAX_PWM);
    const uint16_t rightPwm = static_cast<uint16_t>((rightClamped / MAX_SPEED + 1.0) / 2.0 * MAX_PWM);

    if (!pca9685.setPwm(leftMotorChannel, 0, leftPwm)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set PWM for left motor");
    } else {

    }

    if (!pca9685.setPwm(rightMotorChannel, 0, rightPwm)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set PWM for right motor");
    } else {

    }


    // Publish motor status
    auto statusMsg = geometry_msgs::msg::Twist();
    statusMsg.linear.x = leftSpeed;
    statusMsg.angular.z = rightSpeed;
    motorStatusPub->publish(statusMsg);
}

bool MotorDriverNode::setDirection(const MotorDirection direction, const MotorChannels motor) {
    auto it = motorDirectionMap.find(direction);
    if (it == motorDirectionMap.end()) {
        RCLCPP_ERROR(this->get_logger(), "Invalid motor direction");
        return false;
    }

    const MotorDirectionValues& values = it->second;

    const int channelIn1 = motor.analogIn1;
    const int channelIn2 = motor.analogIn2;

    if (!pca9685.setPwm(channelIn1, 0, values.in1)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set IN1 PWM for motor %d", motor.analogIn1);
        return false;
    }

    if (!pca9685.setPwm(channelIn2, 0, values.in2)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set IN2 PWM for motor %d", motor.analogIn2);
        return false;
    }

    return true;
}

