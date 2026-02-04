#include "jetbot_motors/Pca9685.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <thread>

#include <fcntl.h>
#include <rclcpp/logging.hpp>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>

Pca9685::Pca9685() : addr(0x60), fd(-1) {
    const std::array<const char *, 4> busCandidates = {"/dev/i2c-1", "/dev/i2c-8"};

    for (const auto *bus : busCandidates) {
        RCLCPP_INFO(rclcpp::get_logger("Pca9685"), "Attempting to open I2C device %s", bus);
        fd = open(bus, O_RDWR);
        if (fd >= 0) {
            break;
        }
        RCLCPP_WARN(rclcpp::get_logger("Pca9685"), "Failed to open %s", bus);
    }
    RCLCPP_INFO(rclcpp::get_logger("Pca9685"), "I2C device opened with fd=%d", fd);

    if (fd < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("Pca9685"), "Unable to open any I2C device for PCA9685");
        return;
    }

    if (ioctl(fd, I2C_SLAVE, addr) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("Pca9685"), "Failed to set I2C address 0x%02X", addr);
        close(fd);
        fd = -1;
        return;
    }

    uint8_t mode1 = 0;
    if (!readI2cRegister(ModeOne, mode1)) {
        RCLCPP_ERROR(rclcpp::get_logger("Pca9685"), "PCA9685 not responding at 0x%02X", addr);
        close(fd);
        fd = -1;
        return;
    }

    // Config MODE1 enabling ALLCALL and auto-increment totem pole outputs non iverted
    const uint8_t mode1Config = ALLCALL;
    const uint8_t mode2Config = OUTDRV;
    if (!writeI2cRegister(ModeOne, mode1Config) || !writeI2cRegister(ModeTwo, mode2Config)) {
        RCLCPP_ERROR(rclcpp::get_logger("Pca9685"), "Failed to configure PCA9685 modes");
        close(fd);
        fd = -1;
        return;
    }

    initialized = true;
}

Pca9685::~Pca9685() {
    if (fd >= 0) {
        close(fd);
    }
}

bool Pca9685::setPwmFrequency(uint32_t freqHz) {
    if (!initialized) {
        RCLCPP_ERROR(rclcpp::get_logger("Pca9685"), "PCA9685 not initialized; cannot set frequency");
        return false;
    }

    if (freqHz == 0) {
        RCLCPP_ERROR(rclcpp::get_logger("Pca9685"), "Requested frequency is zero");
        return false;
    }

    // Clamp frequency to achievable range (~24 Hz to ~1526 Hz for 25 MHz clock)
    const double maxHz = static_cast<double>(OSCILLATOR_FREQUENCY) / (PWM_RESOLUTION * (PRESCALE_MIN + 1));
    const double minHz = static_cast<double>(OSCILLATOR_FREQUENCY) / (PWM_RESOLUTION * (PRESCALE_MAX + 1));
    const double clampedHz = std::clamp(static_cast<double>(freqHz), minHz, maxHz);

    const double prescaleF = (static_cast<double>(OSCILLATOR_FREQUENCY) / (PWM_RESOLUTION * clampedHz)) - 1.0;
    const uint8_t prescaler = static_cast<uint8_t>(std::round(std::clamp(prescaleF, static_cast<double>(PRESCALE_MIN), static_cast<double>(PRESCALE_MAX))));

    RCLCPP_DEBUG(rclcpp::get_logger("Pca9685"), "Setting prescaler=%u for requested %u Hz (clamped %.2f Hz)", prescaler, freqHz,
                 clampedHz);

    // Read MODE1
    uint8_t mode1 = 0;
    if (!readI2cRegister(ModeOne, mode1)) {
        return false;
    }

    // Set SLEEP bit to allow prescaler change
    if (!writeI2cRegister(ModeOne, mode1 | SLEEP)) {
        return false;
    }

    // Write prescaler
    if (!writeI2cRegister(PRESCALE, prescaler)) {
        return false;
    }

    // Clear SLEEP bit to start oscillator
    if (!writeI2cRegister(ModeOne, mode1 & static_cast<uint8_t>(~SLEEP))) {
        return false;
    }

    return true;
}

bool Pca9685::setPwm(uint8_t channel, uint16_t on, uint16_t off) {
    if (!initialized) {
        RCLCPP_ERROR(rclcpp::get_logger("Pca9685"), "PCA9685 not initialized; cannot set PWM");
        return false;
    }

    return writeI2cRegister(LEDX_ON_L + 4 * channel, on & 0xFF) &&
           writeI2cRegister(LEDX_ON_H + 4 * channel, (on >> 8) & 0x0F) &&
           writeI2cRegister(LEDX_OFF_L + 4 * channel, off & 0xFF) &&
           writeI2cRegister(LEDX_OFF_H + 4 * channel, (off >> 8) & 0x0F);
}

bool Pca9685::setAllPwm(uint16_t on, uint16_t off) {
    if (!initialized) {
        RCLCPP_ERROR(rclcpp::get_logger("Pca9685"), "PCA9685 not initialized; cannot set all PWM");
        return false;
    }

    return writeI2cRegister(ALL_LED_ON_L, on & 0xFF) && writeI2cRegister(ALL_LED_ON_H, (on >> 8) & 0x0F) &&
           writeI2cRegister(ALL_LED_OFF_L, off & 0xFF) && writeI2cRegister(ALL_LED_OFF_H, (off >> 8) & 0x0F);
}

bool Pca9685::setPin(uint8_t channel, bool high) {
    if (!initialized) {
        RCLCPP_ERROR(rclcpp::get_logger("Pca9685"), "PCA9685 not initialized; cannot set pin");
        return false;
    }

    const uint8_t onH = high ? FULL_ON_BIT : 0x00;
    const uint8_t offH = high ? 0x00 : FULL_OFF_BIT;

    return writeI2cRegister(LEDX_ON_L + 4 * channel, 0x00) &&
           writeI2cRegister(LEDX_ON_H + 4 * channel, onH) &&
           writeI2cRegister(LEDX_OFF_L + 4 * channel, 0x00) &&
           writeI2cRegister(LEDX_OFF_H + 4 * channel, offH);
}

bool Pca9685::reset() {
    if (!initialized) {
        RCLCPP_ERROR(rclcpp::get_logger("Pca9685"), "PCA9685 not initialized; cannot reset");
        return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("Pca9685"), "Resetting PCA9685 via device MODE1 write");
    if (!writeI2cRegister(ModeOne, ALLCALL)) {
        return false;
    }

    // Small delay to allow oscillator restart per datasheet
    std::this_thread::sleep_for(std::chrono::microseconds(600));

    return true;
}

bool Pca9685::readI2cRegister(uint8_t reg, uint8_t &value) {
    if (fd < 0) {
        return false;
    }
    ioctl(fd, I2C_SLAVE, addr);
    uint8_t buf[1] = {reg};

    if (write(fd, buf, 1) != 1) {
        RCLCPP_ERROR(rclcpp::get_logger("Pca9685"), "Failed to write to I2C register");
        return false;
    }

    if (read(fd, &value, 1) != 1) {
        RCLCPP_ERROR(rclcpp::get_logger("Pca9685"), "Failed to read from I2C register");
        return false;
    }

    return true;
}

bool Pca9685::writeI2cRegister(uint8_t reg, uint8_t value) {
    if (fd < 0) {
        return false;
    }

    ioctl(fd, I2C_SLAVE, addr);
    uint8_t buf[2] = {reg, value};

    if (write(fd, buf, 2) != 2) {
        RCLCPP_ERROR(rclcpp::get_logger("Pca9685"), "Failed to write to I2C register");
        return false;
    }

    return true;
}
