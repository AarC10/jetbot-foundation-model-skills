#include <motor_driver/MotorDriverNode.hpp>

MotorDriverNode::MotorDriverNode() : Node("motor_driver_node") {
    if (!pca9685.setPwmFrequency(0)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set PWM frequency on PCA9685");
    }

    cmdVelSub = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&MotorDriverNode::cmdVelCallback, this, std::placeholders::_1));

    motorStatusPub = this->create_publisher<geometry_msgs::msg::Twist>("motor_status", 10);
}

MotorDriverNode::~MotorDriverNode() {}

void MotorDriverNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    static constexpr float MAX_SPEED = 1.0; // TODO: Get the actual number

    auto pwmFromMag = [&](double v) -> uint16_t {
        double mag = std::min(1.0, std::abs(v) / MAX_SPEED);
        return static_cast<uint16_t>(mag * MAX_PWM);
    };

    const double linear = msg->linear.x;
    const double angular = msg->angular.z;

    const double leftSpeed = linear - angular;
    const double rightSpeed = linear + angular;

    const double leftClamped =
        std::max(-static_cast<double>(MAX_SPEED), std::min(static_cast<double>(MAX_SPEED), leftSpeed));
    const double rightClamped =
        std::max(-static_cast<double>(MAX_SPEED), std::min(static_cast<double>(MAX_SPEED), rightSpeed));

    const uint16_t leftPwm = pwmFromMag(leftClamped);
    const uint16_t rightPwm = pwmFromMag(rightClamped);

    if (!pca9685.setPwm(leftMotor.pwm, 0, leftPwm)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set PWM for left motor");
    } else if (!setDirection(leftClamped > 0 ? FORWARD : leftClamped < 0 ? BACKWARD : STOP, leftMotor)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set direction for left motor");
    }

    if (!pca9685.setPwm(rightMotor.pwm, 0, rightPwm)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set PWM for right motor");
    } else if (!setDirection(rightClamped > 0 ? FORWARD : rightClamped < 0 ? BACKWARD : STOP, rightMotor)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set direction for right motor");
    }

    // Publish motor status
    auto statusMsg = geometry_msgs::msg::Twist();
    statusMsg.linear.x = leftSpeed;
    statusMsg.angular.z = rightSpeed;
    motorStatusPub->publish(statusMsg);
}

bool MotorDriverNode::setDirection(const MotorDirection direction, const MotorChannels &motor) {
    auto it = motorDirectionMap.find(direction);
    if (it == motorDirectionMap.end()) {
        RCLCPP_ERROR(this->get_logger(), "Invalid motor direction");
        return false;
    }

    const MotorDirectionValues &values = it->second;

    if (!pca9685.setPwm(motor.analogIn1, 0, values.in1)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set IN1 PWM for motor");
        return false;
    }

    if (!pca9685.setPwm(motor.analogIn2, 0, values.in2)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set IN2 PWM for motor");
        return false;
    }

    return true;
}