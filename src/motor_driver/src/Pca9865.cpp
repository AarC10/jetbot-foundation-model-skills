#include "motor_driver/Pca9865.hpp"

#include <fcntl.h>
#include <rclcpp/logging.hpp>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>

Pca9865::Pca9865() : addr(0x40), fd(-1) {
  fd = open("/dev/i2c-1", O_RDWR);
  if (fd < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("Pca9865"), "Failed to open I2C device");
  }
  if (ioctl(fd, I2C_SLAVE, addr) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("Pca9865"), "Failed to set I2C address");
  }

  reset();

  initialized = true;
}

bool Pca9865::setPwmFrequency(uint32_t freqHz) { return true; }

bool Pca9865::setPwm(uint8_t channel, uint16_t on, uint16_t off) {
  return true;
}

bool Pca9865::setAllPwm(uint16_t on, uint16_t off) {
    return true;
}

bool Pca9865::reset() {
  if (!initialized) {
    return false;
  }

  ioctl(fd, I2C_SLAVE, 0x00);
  uint8_t buf[2] = {0x00, 0x06};

  if (write(fd, buf, 2) != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("Pca9865"),
                 "Failed to write SWRST to I2C general call address");
    return false;
  }

  return true;
}

bool Pca9865::readI2cRegister(uint8_t reg, uint8_t &value) {
  ioctl(fd, I2C_SLAVE, addr);
  uint8_t buf[1] = {reg};

  if (write(fd, buf, 1) != 1) {
    RCLCPP_ERROR(rclcpp::get_logger("Pca9865"),
                 "Failed to write to I2C register");
    return false;
  }

  if (read(fd, &value, 1) != 1) {
    RCLCPP_ERROR(rclcpp::get_logger("Pca9865"),
                 "Failed to read from I2C register");
    return false;
  }

  return true;
}

bool Pca9865::writeI2cRegister(uint8_t reg, uint8_t value) {
  ioctl(fd, I2C_SLAVE, addr);
  uint8_t buf[2] = {reg, value};

  if (write(fd, buf, 2) != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("Pca9865"),
                 "Failed to write to I2C register");
    return false;
  }

  return true;
}

bool Pca9865::readI2cRegisters(uint8_t reg, uint8_t *buffer, size_t length) {
  ioctl(fd, I2C_SLAVE, addr);
  if (write(fd, &reg, 1) != 1) {
    RCLCPP_ERROR(rclcpp::get_logger("Pca9865"),
                 "Failed to write to I2C register");
    return false;
  }

  if (read(fd, buffer, length) != (ssize_t)length) {
    RCLCPP_ERROR(rclcpp::get_logger("Pca9865"),
                 "Failed to read from I2C registers");
    return false;
  }

  return true;
}

bool Pca9865::writeI2cRegisters(uint8_t reg, const uint8_t *buffer,
                                size_t length) {
  ioctl(fd, I2C_SLAVE, addr);
  uint8_t buf[length + 1];
  buf[0] = reg;
  memcpy(&buf[1], buffer, length);

  if (write(fd, buf, length + 1) != (ssize_t)(length + 1)) {
    RCLCPP_ERROR(rclcpp::get_logger("Pca9865"),
                 "Failed to write to I2C registers");
    return false;
  }

  return true;
}