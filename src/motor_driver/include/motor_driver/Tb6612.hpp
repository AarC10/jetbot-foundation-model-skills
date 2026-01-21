#pragma once

#include <stddef.h>
#include <stdint.h>


class Tb6612 {
public:
    enum ControlPins {
        AIN1,
        AIN2,
        BIN1,
        BIN2,
        STBY
    };

    Tb6612();

    ~Tb6612();

    bool setMotorAForward(bool enable);

    bool setMotorABackward(bool enable);

    bool setMotorBForward(bool enable);

    bool setMotorBBackward(bool enable);

    bool setStandby(bool enable);
private:
    int fd;
};