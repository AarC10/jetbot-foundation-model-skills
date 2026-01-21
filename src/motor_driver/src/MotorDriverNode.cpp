#include <algorithm>
#include <motor_driver/MotorDriverNode.hpp>


MotorDriverNode::MotorDriverNode() : Node("motor_driver_node") {
    // Set PWM frequency to 1600 Hz (standard for DC motor control on Adafruit Motor HAT)
    if (!pca9685.setPwmFrequency(1600)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set PWM frequency on PCA9865");
    }

    cmdVelSub = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&MotorDriverNode::cmdVelCallback, this, std::placeholders::_1));

    motorStatusPub = this->create_publisher<geometry_msgs::msg::Twist>("motor_status", 10);
}

MotorDriverNode::~MotorDriverNode() {}

void MotorDriverNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    static constexpr double MAX_SPEED = 1.0;
    static constexpr double DEAD_BAND = 1e-3;

    auto pwmFromMag = [&](double velocity) -> uint16_t {
        double mag = std::min(1.0, std::abs(velocity) / MAX_SPEED);
        return static_cast<uint16_t>(std::lround(mag * static_cast<double>(MAX_PWM)));
    };

    auto dirFrom = [&](double v) -> MotorDirection {
        if (std::abs(v) <= DEAD_BAND) {
            return STOP;
            // Alternative if motors should coast instead of stop
            // return COAST;
        }
        return (v > 0.0) ? FORWARD : BACKWARD;
    };

    const double linear = msg->linear.x;
    const double angular = msg->angular.z;

    const WheelLinear wheelSpeeds = computeWheelSpeeds(linear, angular);
    const double leftCmd = clamp(wheelSpeeds.leftMps);
    const double rightCmd = clamp(wheelSpeeds.rightMps);

    const MotorDirection leftDir = dirFrom(leftCmd);
    const MotorDirection rightDir = dirFrom(rightCmd);

    const uint16_t leftPwm = (leftDir == STOP || leftDir == COAST) ? 0 : pwmFromMag(leftCmd);
    const uint16_t rightPwm = (rightDir == STOP || rightDir == COAST) ? 0 : pwmFromMag(rightCmd);

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

    // Publish normalized motor speeds ([-1, 1] range) for feedback
    geometry_msgs::msg::Twist statusMsg;
    statusMsg.linear.x = normalizeWheelSpeed(leftCmd, MAX_WHEEL_MPS);
    statusMsg.angular.z = normalizeWheelSpeed(rightCmd, MAX_WHEEL_MPS);
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