#pragma once

#include <cmath>
#include <cstdint>
#include <map>

#include <geometry_msgs/msg/twist.hpp>
#include <jetbot_motors/Pca9685.hpp>
#include <jetbot_motors/action/drive_distance.hpp>
#include <jetbot_motors/action/turn_angle.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <algorithm>
#include <thread>

class MotorNode : public rclcpp::Node {
  public:
    MotorNode();
    ~MotorNode();

  private:
    using DriveDistance = jetbot_motors::action::DriveDistance;
    using DriveDistanceGoalHandle = rclcpp_action::ServerGoalHandle<DriveDistance>;
    using TurnAngle = jetbot_motors::action::TurnAngle;
    using TurnAngleGoalHandle = rclcpp_action::ServerGoalHandle<TurnAngle>;

    static constexpr double WHEEL_RADIUS_M = 0.065; // 65 mm
    static constexpr double TRACK_WIDTH_M = 0.12;   // 120 mm
    static constexpr double DEFAULT_DISTANCE_SCALE = 1.0; // Calibration factor for distance traveled

    static constexpr uint16_t MIN_PWM = 0;
    static constexpr uint16_t MAX_PWM = 4095;

    static constexpr double DEAD_BAND = 1e-3;
    static constexpr double DEFAULT_LINEAR_SPEED_MPS = 0.25;
    static constexpr double DEFAULT_ANGULAR_SPEED_RPS = 1.0;
    static constexpr double ACTION_LOOP_HZ = 20.0;

    enum MotorDirection { FORWARD, BACKWARD, COAST, STOP };

    struct WheelLinear {
        double leftMps;
        double rightMps;
    };

    struct MotorChannels {
        uint8_t analogIn1;
        uint8_t analogIn2;
        uint8_t pwm;
    };

    struct MotorDirectionBits {
        bool in1High;
        bool in2High;
    };

    const std::map<MotorDirection, MotorDirectionBits> motorDirectionMap = {
        {FORWARD, {true, false}},
        {BACKWARD, {false, true}},
        {COAST, {false, false}},
        {STOP, {true, true}},
    };

    Pca9685 pca9685;
    rclcpp_action::Server<DriveDistance>::SharedPtr driveDistanceServer;
    rclcpp_action::Server<TurnAngle>::SharedPtr turnAngleServer;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motorStatusPub;

    double leftMotorGain = 1.0;
    double rightMotorGain = 1.0;
    double distanceScale = DEFAULT_DISTANCE_SCALE;

    // Hardcoding for now since this is limited by Adafruit Motor HAT
    // Might need to swap or use 8, 9, 10 and 11, 12, 13 for other side
    // const MotorChannels leftMotor = {4, 3, 2};
    // const MotorChannels rightMotor = {5, 6, 7};
    const MotorChannels leftMotor = {10, 9, 8};
    const MotorChannels rightMotor = {11, 12, 13};

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    bool setDirection(const MotorDirection direction, const MotorChannels &motor);

    rclcpp_action::GoalResponse handleTurnAngleGoal(const rclcpp_action::GoalUUID &uuid,
                                                    std::shared_ptr<const TurnAngle::Goal> goal);
    rclcpp_action::CancelResponse handleTurnAngleCancel(const std::shared_ptr<TurnAngleGoalHandle> goalHandle);

    rclcpp_action::GoalResponse handleDriveDistanceGoal(const rclcpp_action::GoalUUID &uuid,
                                                        std::shared_ptr<const DriveDistance::Goal> goal);
    rclcpp_action::CancelResponse handleDriveDistanceCancel(const std::shared_ptr<DriveDistanceGoalHandle> goalHandle);

    void handleDriveDistanceAccepted(const std::shared_ptr<DriveDistanceGoalHandle> goalHandle);

    void executeDriveDistance(const std::shared_ptr<DriveDistanceGoalHandle> goalHandle);

    void handleTurnAngleAccepted(const std::shared_ptr<TurnAngleGoalHandle> goalHandle);

    void executeTurnAngle(const std::shared_ptr<TurnAngleGoalHandle> goalHandle);

    void applyVelocity(const double velMps, const double omegaRps);

    void stopMotors();

    static double resolveSpeed(const double requestedMagnitude, const double defaultMagnitude, const double limit);

    static inline WheelLinear computeWheelSpeeds(const double velMps, const double omegaRps) {
        const double half = TRACK_WIDTH_M * 0.5;
        return WheelLinear{velMps - (omegaRps * half), velMps + (omegaRps * half)};
    }

    static inline double normalizeWheelSpeed(const double wheelMps, const double maxWheelMps) {
        if (maxWheelMps <= 0.0) {
            return 0.0;
        }

        return std::clamp(wheelMps / maxWheelMps, -1.0, 1.0);
    }

    static inline double applyGain(const double cmdNorm, const double gain) {
        if (gain <= 0.0) {
            return 0.0;
        }

        return std::clamp(cmdNorm * gain, -1.0, 1.0);
    }

    static inline MotorDirection directionFromCmd(const double cmdNorm) {
        if (std::abs(cmdNorm) <= DEAD_BAND) {
            return STOP;
        }

        return (cmdNorm > 0.0) ? FORWARD : BACKWARD;
    }

    static inline uint16_t pwmFromNormalized(const double cmdNorm) {
        const double mag = std::clamp(std::abs(cmdNorm), 0.0, 1.0);
        return static_cast<uint16_t>(std::lround(mag * static_cast<double>(MAX_PWM)));
    }
};
