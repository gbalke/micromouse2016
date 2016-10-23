#pragma once
#include "mbed.h"

class Motor {
    public:
        Motor(PinName forward, PinName backward, float multiplier = 1);
        void set_speed(float speed);
        float get_speed();
    private:
        PwmOut forward;
        PwmOut backward;
        float multiplier;
};
