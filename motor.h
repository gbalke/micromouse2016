#pragma once
#include "mbed.h"

class Motor {
    public:
        Motor(PinName forward, PinName backward);
        void set_speed(float speed);
    private:
        PwmOut forward;
        PwmOut backward;
};
