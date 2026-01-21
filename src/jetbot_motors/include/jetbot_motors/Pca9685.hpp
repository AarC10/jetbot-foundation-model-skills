#pragma once

#include <cstdint>
#include <stddef.h>
#include <stdint.h>

class Pca9685 {
  public:
    enum Registers {
        ModeOne = 0x00,
        ModeTwo = 0x01,
        SUBADR1 = 0x02,
        SUBADR2 = 0x03,
        SUBADR3 = 0x04,
        ALLCALLADR = 0x05,
        LEDX_ON_L = 0x06,
        LEDX_ON_H = 0x07,
        LEDX_OFF_L = 0x08,
        LEDX_OFF_H = 0x09,
        ALL_LED_ON_L = 0xFA,
        ALL_LED_ON_H = 0xFB,
        ALL_LED_OFF_L = 0xFC,
        ALL_LED_OFF_H = 0xFD,
        PRESCALE = 0xFE,
    };

    enum BitMasks {
        RESTART = 0x80,
        SLEEP = 0x10,
        ALLCALL = 0x01,
        INVRT = 0x10,
        OUTDRV = 0x04,
    };

    static constexpr uint8_t PRESCALE_MIN = 0x03;
    static constexpr uint8_t PRESCALE_MAX = 0xFF;
    static constexpr uint16_t PWM_RESOLUTION = 4096;
    static constexpr uint32_t OSCILLATOR_FREQUENCY = 25'000'000;

    Pca9685();

    ~Pca9685();

    bool setPwmFrequency(uint32_t freqHz);

    bool setPwm(uint8_t channel, uint16_t on, uint16_t off);

    bool setAllPwm(uint16_t on, uint16_t off);

    bool setPin(uint8_t channel, bool high);

    bool reset();

  private:
    static constexpr uint16_t FULL_OFF = 0;
    static constexpr uint16_t FULL_ON = 4095;

    uint8_t addr;
    int fd;
    bool initialized = false;

    bool readI2cRegister(uint8_t reg, uint8_t &value);

    bool writeI2cRegister(uint8_t reg, uint8_t value);

    bool readI2cRegisters(uint8_t reg, uint8_t *buffer, size_t length);

    bool writeI2cRegisters(uint8_t reg, const uint8_t *buffer, size_t length);
};