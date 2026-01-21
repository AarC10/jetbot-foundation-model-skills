#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <motor_driver/Pca9685.hpp>
#include <rclcpp/rclcpp.hpp>

class MotorDriverNode : public rclcpp::Node {
  public:
    MotorDriverNode();

    ~MotorDriverNode();

  private:
    static constexpr double WHEEL_RADIUS_M = 0.065; // 65 mm
    static constexpr double TRACK_WIDTH_M = 0.1; // TODO: Measure Jetbot 3d print
    static constexpr double MAX_WHEEL_MPS = 2.0; // TODO: Measure max wheel speed at full PWM

    static constexpr uint16_t MIN_PWM = 0;
    static constexpr uint16_t MAX_PWM = 4095;

    struct WheelLinear {
        double leftMps;
        double rightMps;
    };

    enum MotorDirection { FORWARD, BACKWARD, COAST, STOP };

    struct MotorChannels {
        uint8_t analogIn1;
        uint8_t analogIn2;
        uint8_t pwm;
    };

    struct MotorDirectionValues {
        uint16_t in1;
        uint16_t in2;
    };

    const std::map<MotorDirection, MotorDirectionValues> motorDirectionMap = {{FORWARD, {MAX_PWM, MIN_PWM}},
                                                                              {BACKWARD, {MIN_PWM, MAX_PWM}},
                                                                              {COAST, {MIN_PWM, MIN_PWM}},
                                                                              {STOP, {MAX_PWM, MAX_PWM}}};

    Pca9685 pca9685;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;
    // TODO: Custom message
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr motorStatusPub;

    // Hardcoding for now since this is limited by Adafruit Motor HAT
    // Might need to swap or use 8, 9, 10 and 11, 12, 13 for other side
    const MotorChannels leftMotor = {4, 3, 2};
    const MotorChannels rightMotor = {5, 6, 7};

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    bool setDirection(const MotorDirection direction, const MotorChannels &motor);

    WheelLinear computeWheelSpeeds(double velMps, double omegaRps) {
        const double half = TRACK_WIDTH_M * 0.5;
        return {
            .leftMps = velMps - (omegaRps * half),
            .rightMps = velMps + (omegaRps * half)
        };
    }

    double normalizeWheelSpeed(double wheelMps, double maxWheelMps) {
        return std::clamp(wheelMps / maxWheelMps, -1.0, 1.0);
    }
};
