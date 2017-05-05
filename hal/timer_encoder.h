#pragma once
#include "PinNames.h"

#include "timer.h"

class TimerEncoder {
    public:
        TimerEncoder(TimerModule t, PinName a, PinName b);
        int count();
        void reset();
    private:
        volatile timer_register *timer;
};
