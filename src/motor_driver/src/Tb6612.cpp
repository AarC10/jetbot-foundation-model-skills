#include <motor_driver/Tb6612.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <rclcpp/logging.hpp>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>


Tb6612::Tb6612() {
  fd = open("/dev/i2c-1", O_RDWR);
  if (fd < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("TB6612"), "Failed to open I2C device");
    return;
  }
  if (ioctl(fd, I2C_SLAVE, 0x10) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("TB6612"), "Failed to set I2C address");
    close(fd);
    fd = -1;
    return;
  }
}

Tb6612::~Tb6612() {
  if (fd >= 0) {
    close(fd);
  }
}

bool Tb6612::setMotorAForward(bool enable) {
  if (fd < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("TB6612"), "I2C device not opened");
    return false;
  }

  uint8_t command = enable ? 0x01 : 0x00;
  uint8_t buf[2] = {AIN1, command};

  if (write(fd, buf, 2) != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("TB6612"),
                 "Failed to write to AIN1 register");
    return false;
  }

  return true;
}

bool Tb6612::setMotorABackward(bool enable) {
  if (fd < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("TB6612"), "I2C device not opened");
    return false;
  }

  uint8_t command = enable ? 0x01 : 0x00;
  uint8_t buf[2] = {AIN2, command};

  if (write(fd, buf, 2) != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("TB6612"),
                 "Failed to write to AIN2 register");
    return false;
  }

  return true;
}


bool Tb6612::setMotorBForward(bool enable) {
  if (fd < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("TB6612"), "I2C device not opened");
    return false;
  }

  uint8_t command = enable ? 0x01 : 0x00;
  uint8_t buf[2] = {BIN1, command};

  if (write(fd, buf, 2) != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("TB6612"),
                 "Failed to write to BIN1 register");
    return false;
  }

  return true;
}

bool Tb6612::setMotorBBackward(bool enable) {
  if (fd < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("TB6612"), "I2C device not opened");
    return false;
  }

  uint8_t command = enable ? 0x01 : 0x00;
  uint8_t buf[2] = {BIN2, command};

  if (write(fd, buf, 2) != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("TB6612"),
                 "Failed to write to BIN2 register");
    return false;
  }

  return true;
}

bool Tb6612::setStandby(bool enable) {
  if (fd < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("TB6612"), "I2C device not opened");
    return false;
  }

  uint8_t command = enable ? 0x01 : 0x00;
  uint8_t buf[2] = {STBY, command};

  if (write(fd, buf, 2) != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("TB6612"),
                 "Failed to write to STBY register");
    return false;
  }

  return true;
}
