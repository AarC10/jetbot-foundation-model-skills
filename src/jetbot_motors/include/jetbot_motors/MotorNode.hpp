#pragma once

#include <cmath>
#include <cstdint>
#include <map>

#include <geometry_msgs/msg/twist.hpp>
#include <jetbot_motors/Pca9685.hpp>
#include <rclcpp/rclcpp.hpp>

class MotorDriverNode : public rclcpp::Node {
  public:
    MotorDriverNode();
    ~MotorDriverNode();

  private:
    static constexpr double WHEEL_RADIUS_M = 0.065; // 65 mm
    static constexpr double TRACK_WIDTH_M = 0.1;    // TODO: Measure Jetbot 3d print
    static constexpr double MAX_WHEEL_MPS = 2.0;    // TODO: Measure max wheel speed at full PWM

    static constexpr uint16_t MIN_PWM = 0;
    static constexpr uint16_t MAX_PWM = 4095;

    static constexpr double DEAD_BAND = 1e-3;

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

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motorStatusPub;

    // Hardcoding for now since this is limited by Adafruit Motor HAT
    // Might need to swap or use 8, 9, 10 and 11, 12, 13 for other side
    const MotorChannels leftMotor = {4, 3, 2};
    const MotorChannels rightMotor = {5, 6, 7};

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    bool setDirection(const MotorDirection direction, const MotorChannels &motor);

    static inline WheelLinear computeWheelSpeeds(const double velMps, const double omegaRps) {
        const double half = TRACK_WIDTH_M * 0.5;
        return {.leftMps = velMps - (omegaRps * half), .rightMps = velMps + (omegaRps * half)};
    }

    static inline double normalizeWheelSpeed(const double wheelMps, const double maxWheelMps) {
        if (maxWheelMps <= 0.0) {
            return 0.0;
        }

        return std::clamp(wheelMps / maxWheelMps, -1.0, 1.0);
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
