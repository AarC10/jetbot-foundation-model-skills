#pragma once

#include 

class Pca9865 {
public:
    enum Registers {
        ModeOne = 0x00,
        ModeTwo,
        SUBADR1,
        SUBADR2,
        SUBADR3,
        PRESCALE,
        LED0_ON_L,
        LED0_ON_H,
        LED0_OFF_L,
        LED0_OFF_H,
        ALL_LED_ON_L,
        ALL_LED_ON_H,
        ALL_LED_OFF_L,
        ALL_LED_OFF_H,
    }

    enum BitMasks {
        RESTART = 0x80,
        SLEEP = 0x10,
        ALLCALL = 0x01,
        INVRT = 0x10,
        OUTDRV = 0x04,
    }

    Pca9865();

    bool setPwmFrequency(int freqHz);

    bool setPwm(int channel, int on, int off);

    bool setAllPwm(int on, int off);

private:
    uint8_t addr;
    int fd;

    bool readI2cRegister(uint8_t reg, uint8_t &value);

    bool writeI2cRegister(uint8_t reg, uint8_t value);

    bool readI2cRegisters(uint8_t reg, uint8_t *buffer, size_t length);

    bool writeI2cRegisters(uint8_t reg, const uint8_t *buffer, size_t length);
}