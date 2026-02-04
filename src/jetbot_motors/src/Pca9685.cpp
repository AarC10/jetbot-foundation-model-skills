#include "jetbot_motors/Pca9685.hpp"

#include <fcntl.h>
#include <rclcpp/logging.hpp>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>

Pca9685::Pca9685() : addr(0x40), fd(-1) {
    fd = open("/dev/i2c-8", O_RDWR);
    RCLCPP_INFO(rclcpp::get_logger("Pca9685"), "Attempting to open I2C device /dev/i2c-8");
    if (fd < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("Pca9685"), "Failed to open I2C device");
        return;
    }
    if (ioctl(fd, I2C_SLAVE, addr) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("Pca9685"), "Failed to set I2C address");
        return;
    }

    initialized = true;

    reset();

    // Sets MODE1 to enable auto-increment and no subaddresses
    uint8_t mode1 = 0x01; // ALLCALL enabled
    writeI2cRegister(ModeOne, mode1);

    // Set MODE2 to open drain output no inversion
    uint8_t mode2 = OUTDRV;
    writeI2cRegister(ModeTwo, mode2);
}

Pca9685::~Pca9685() {
    if (fd >= 0) {
        close(fd);
    }
}

bool Pca9685::setPwmFrequency(uint32_t freqHz) {
    uint32_t prescaler = OSCILLATOR_FREQUENCY / (PWM_RESOLUTION * freqHz) - 1;
    if (prescaler < PRESCALE_MIN || prescaler > PRESCALE_MAX) {
        RCLCPP_ERROR(rclcpp::get_logger("Pca9685"), "Frequency out of range for PCA9685");
        return false;
    }

    RCLCPP_DEBUG(rclcpp::get_logger("Pca9685"), "Setting PCA9685 prescaler to %u for %u Hz", prescaler, freqHz);

    // Read MODE1
    uint8_t mode1;
    if (!readI2cRegister(ModeOne, mode1)) {
        return false;
    }

    // Set SLEEP bit to allow prescaler change
    if (!writeI2cRegister(ModeOne, mode1 | SLEEP)) {
        return false;
    }

    // Write prescaler
    if (!writeI2cRegister(PRESCALE, (uint8_t)prescaler)) {
        return false;
    }

    // Clear SLEEP bit to start oscillator
    if (!writeI2cRegister(ModeOne, mode1 & ~SLEEP)) {
        return false;
    }

    return true;
}

bool Pca9685::setPwm(uint8_t channel, uint16_t on, uint16_t off) {
    return writeI2cRegister(LEDX_ON_L + 4 * channel, on & 0xFF) &&
           writeI2cRegister(LEDX_ON_H + 4 * channel, (on >> 8) & 0x0F) &&
           writeI2cRegister(LEDX_OFF_L + 4 * channel, off & 0xFF) &&
           writeI2cRegister(LEDX_OFF_H + 4 * channel, (off >> 8) & 0x0F);
}

bool Pca9685::setAllPwm(uint16_t on, uint16_t off) {
    return writeI2cRegister(ALL_LED_ON_L, on & 0xFF) && writeI2cRegister(ALL_LED_ON_H, (on >> 8) & 0x0F) &&
           writeI2cRegister(ALL_LED_OFF_L, off & 0xFF) && writeI2cRegister(ALL_LED_OFF_H, (off >> 8) & 0x0F);
}

bool Pca9685::setPin(uint8_t channel, bool high) {
    if (high) {
        return setPwm(channel, FULL_ON, FULL_OFF);
    } else {
        return setPwm(channel, FULL_OFF, FULL_ON);
    }
}

bool Pca9685::reset() {
    RCLCPP_INFO(rclcpp::get_logger("Pca9685"), "Resetting PCA9685 via I2C general call");
    if (fd < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("Pca9685"), "I2C device not opened");
        return false;
    }

    ioctl(fd, I2C_SLAVE, 0x00);
    uint8_t buf[2] = {0x00, 0x06};

    if (write(fd, buf, 2) != 2) {
        RCLCPP_ERROR(rclcpp::get_logger("Pca9685"), "Failed to write SWRST to I2C general call address");
        return false;
    }

    ioctl(fd, I2C_SLAVE, addr);


    RCLCPP_INFO(rclcpp::get_logger("Pca9685"), "PCA9685 reset complete");
    return true;
}

bool Pca9685::readI2cRegister(uint8_t reg, uint8_t &value) {
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
    ioctl(fd, I2C_SLAVE, addr);
    uint8_t buf[2] = {reg, value};

    if (write(fd, buf, 2) != 2) {
        RCLCPP_ERROR(rclcpp::get_logger("Pca9685"), "Failed to write to I2C register");
        return false;
    }

    return true;
}
