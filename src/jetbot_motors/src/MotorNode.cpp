#include "jetbot_motors/MotorNode.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <type_traits>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

MotorNode::MotorNode() : Node("jetbot_motors_node") {
    leftMotorGain = this->declare_parameter<double>("left_motor_gain", 1.0);
    rightMotorGain = this->declare_parameter<double>("right_motor_gain", 1.0);
    distanceScale = this->declare_parameter<double>("distance_scale_factor", DEFAULT_DISTANCE_SCALE);

    if (leftMotorGain <= 0.0 || rightMotorGain <= 0.0 || distanceScale <= 0.0) {
        RCLCPP_WARN(this->get_logger(),
                    "Motor params must be > 0.0. Using defaults for invalid values (left=%.3f, right=%.3f, dist_scale=%.3f).",
                    leftMotorGain, rightMotorGain, distanceScale);
        leftMotorGain = (leftMotorGain > 0.0) ? leftMotorGain : 1.0;
        rightMotorGain = (rightMotorGain > 0.0) ? rightMotorGain : 1.0;
        distanceScale = (distanceScale > 0.0) ? distanceScale : DEFAULT_DISTANCE_SCALE;
    }

    if (!pca9685.setPwmFrequency(1000)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set PWM frequency on PCA9685");
    }

    cmdVelSub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
                                                                     std::bind(&MotorNode::cmdVelCallback, this, _1));

    motorStatusPub = this->create_publisher<geometry_msgs::msg::Twist>("motor_status", 10);

    driveDistanceServer = rclcpp_action::create_server<DriveDistance>(
        this, "drive_distance", std::bind(&MotorNode::handleDriveDistanceGoal, this, _1, _2),
        std::bind(&MotorNode::handleDriveDistanceCancel, this, _1),
        std::bind(&MotorNode::handleDriveDistanceAccepted, this, _1));

    turnAngleServer = rclcpp_action::create_server<TurnAngle>(this, "turn_angle",
                                                              std::bind(&MotorNode::handleTurnAngleGoal, this, _1, _2),
                                                              std::bind(&MotorNode::handleTurnAngleCancel, this, _1),
                                                              std::bind(&MotorNode::handleTurnAngleAccepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "MotorNode initialized");
}

MotorNode::~MotorNode() = default;

void MotorNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    const double velMps = msg->linear.x;
    const double omegaRps = msg->angular.z;

    const WheelLinear wheels = computeWheelSpeeds(velMps, omegaRps);
    const double leftNorm = applyGain(normalizeWheelSpeed(wheels.leftMps, 2.0), leftMotorGain);
    const double rightNorm = applyGain(normalizeWheelSpeed(wheels.rightMps, 2.0), rightMotorGain);

    const MotorDirection leftDir = directionFromCmd(leftNorm);
    const MotorDirection rightDir = directionFromCmd(rightNorm);

    const uint16_t leftPwm = (leftDir == STOP || leftDir == COAST) ? 0 : pwmFromNormalized(leftNorm);
    const uint16_t rightPwm = (rightDir == STOP || rightDir == COAST) ? 0 : pwmFromNormalized(rightNorm);

    if (!setDirection(leftDir, leftMotor)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set direction for left motor");
    } else if (!pca9685.setPwm(leftMotor.pwm, 0, leftPwm)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set PWM for left motor");
    } else {
        RCLCPP_DEBUG(this->get_logger(), "Left motor: dir=%d, pwm=%u", static_cast<int>(leftDir), leftPwm);
    }

    if (!setDirection(rightDir, rightMotor)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set direction for right motor");
    } else if (!pca9685.setPwm(rightMotor.pwm, 0, rightPwm)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set PWM for right motor");
    } else {
        RCLCPP_DEBUG(this->get_logger(), "Right motor: dir=%d, pwm=%u", static_cast<int>(rightDir), rightPwm);
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

    // RCLCPP_INFO(this->get_logger(), "Set motor direction to %d", static_cast<int>(direction));

    return true;
}

void MotorNode::applyVelocity(const double velMps, const double omegaRps) {
    const WheelLinear wheels = computeWheelSpeeds(velMps, omegaRps);
    const double leftNorm = applyGain(normalizeWheelSpeed(wheels.leftMps, 2.0), leftMotorGain);
    const double rightNorm = applyGain(normalizeWheelSpeed(wheels.rightMps, 2.0), rightMotorGain);

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
}

void MotorNode::stopMotors() {
    if (!setDirection(STOP, leftMotor)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to stop left motor");
    }
    
    if (!setDirection(STOP, rightMotor)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to stop right motor");
    }

    (void)pca9685.setPwm(leftMotor.pwm, 0, 0);
    (void)pca9685.setPwm(rightMotor.pwm, 0, 0);
    // RCLCPP_INFO(this->get_logger(), "Motors stopped");
}

double MotorNode::resolveSpeed(const double requestedMagnitude, const double defaultMagnitude, const double limit) {
    const double base = (requestedMagnitude > 0.0) ? requestedMagnitude : defaultMagnitude;
    if (limit <= 0.0) {
        return std::abs(base);
    }
    return std::min(std::abs(base), limit);
}

rclcpp_action::GoalResponse MotorNode::handleDriveDistanceGoal(const rclcpp_action::GoalUUID &uuid,
                                                               std::shared_ptr<const DriveDistance::Goal> goal) {
    (void)uuid;
    if (std::abs(goal->distance_m) <= DEAD_BAND) {
        RCLCPP_WARN(this->get_logger(), "Rejecting DriveDistance goal near zero distance");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
MotorNode::handleDriveDistanceCancel(const std::shared_ptr<DriveDistanceGoalHandle> goalHandle) {
    (void)goalHandle;
    RCLCPP_INFO(this->get_logger(), "Canceling DriveDistance goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MotorNode::handleDriveDistanceAccepted(const std::shared_ptr<DriveDistanceGoalHandle> goalHandle) {
    std::thread{[this, goalHandle]() { executeDriveDistance(goalHandle); }}.detach();
}

void MotorNode::executeDriveDistance(const std::shared_ptr<DriveDistanceGoalHandle> goalHandle) {
    const auto goal = goalHandle->get_goal();
    auto result = std::make_shared<DriveDistance::Result>();

    const double distance = goal->distance_m;
    const double direction = (distance >= 0.0) ? 1.0 : -1.0;
    const double speed = resolveSpeed(goal->speed_mps, DEFAULT_LINEAR_SPEED_MPS, 2.0);

    if (speed <= 0.0) {
        result->success = false;
        result->message = "Resolved speed is zero";
        goalHandle->abort(result);
        return;
    }

    const double durationSec = (std::abs(distance) / speed) * distanceScale;
    rclcpp::Rate rate(ACTION_LOOP_HZ);
    auto feedback = std::make_shared<DriveDistance::Feedback>();
    const auto start = this->get_clock()->now();

    while (rclcpp::ok()) {
        if (goalHandle->is_canceling()) {
            stopMotors();
            result->success = false;
            result->message = "DriveDistance goal canceled";
            goalHandle->canceled(result);
            return;
        }

        applyVelocity(direction * speed, 0.0);

        const double elapsed = (this->get_clock()->now() - start).seconds();
        const double traveled = std::min(elapsed * speed, std::abs(distance));
        const double remaining = (std::abs(distance) - traveled) * direction;
        feedback->remaining_m = remaining;
        goalHandle->publish_feedback(feedback);

        if (elapsed >= durationSec) {
            break;
        }

        rate.sleep();
    }

    stopMotors();
    result->success = true;
    result->message = "Completed drive distance";
    goalHandle->succeed(result);
}

rclcpp_action::GoalResponse MotorNode::handleTurnAngleGoal(const rclcpp_action::GoalUUID &uuid,
                                                           std::shared_ptr<const TurnAngle::Goal> goal) {
    (void)uuid;
    if (std::abs(goal->angle_rad) <= DEAD_BAND) {
        RCLCPP_WARN(this->get_logger(), "Rejecting TurnAngle goal near zero angle");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotorNode::handleTurnAngleCancel(const std::shared_ptr<TurnAngleGoalHandle> goalHandle) {
    (void)goalHandle;
    RCLCPP_INFO(this->get_logger(), "Canceling TurnAngle goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MotorNode::handleTurnAngleAccepted(const std::shared_ptr<TurnAngleGoalHandle> goalHandle) {
    std::thread{[this, goalHandle]() { executeTurnAngle(goalHandle); }}.detach();
}

void MotorNode::executeTurnAngle(const std::shared_ptr<TurnAngleGoalHandle> goalHandle) {
    const auto goal = goalHandle->get_goal();
    auto result = std::make_shared<TurnAngle::Result>();

    const double angle = goal->angle_rad;
    const double direction = (angle >= 0.0) ? 1.0 : -1.0;
    const double speed = resolveSpeed(goal->angular_speed_rps, DEFAULT_ANGULAR_SPEED_RPS, DEFAULT_ANGULAR_SPEED_RPS);

    if (speed <= 0.0) {
        result->success = false;
        result->message = "Resolved angular speed is zero";
        goalHandle->abort(result);
        return;
    }

    const double durationSec = (std::abs(angle) / speed) * distanceScale;
    rclcpp::Rate rate(ACTION_LOOP_HZ);
    auto feedback = std::make_shared<TurnAngle::Feedback>();
    const auto start = this->get_clock()->now();

    while (rclcpp::ok()) {
        if (goalHandle->is_canceling()) {
            stopMotors();
            result->success = false;
            result->message = "TurnAngle goal canceled";
            goalHandle->canceled(result);
            return;
        }

        applyVelocity(0.0, direction * speed);

        const double elapsed = (this->get_clock()->now() - start).seconds();
        const double turned = std::min(elapsed * speed, std::abs(angle));
        const double remaining = (std::abs(angle) - turned) * direction;
        feedback->remaining_rad = remaining;
        goalHandle->publish_feedback(feedback);

        if (elapsed >= durationSec) {
            break;
        }

        rate.sleep();
    }

    stopMotors();
    result->success = true;
    result->message = "Completed turn angle";
    goalHandle->succeed(result);
}
