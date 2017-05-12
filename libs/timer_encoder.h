#pragma once
#include "pin.h"
#include "timer.h"

class TimerEncoder {
    public:
        TimerEncoder(HAL::Timer::TimerNumber t, Pin a, Pin b);
        int count();
        void reset();
    private:
        HAL::Timer timer;
};
