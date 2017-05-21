#pragma once
#include "timer.h"

class Motor {
    public:
        Motor(HAL::Timer::Channel forward, HAL::Timer::Channel backward);
        void set_speed(int speed);
        void brake();
    private:
        HAL::Timer::Channel forward;
        HAL::Timer::Channel backward;
};
