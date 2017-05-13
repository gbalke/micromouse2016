#include "pin.h"
#include "timer.h"
#include "timer_encoder.h"

using namespace HAL;

TimerEncoder::TimerEncoder(Timer::TimerNumber t, Pin a, Pin b)
: timer(t, Timer::ENCODER_A)
{
    timer.set_period(~0);
    timer.set_count(0);
    Timer::Channel ch1(timer, Timer::CH1, Timer::Channel::CAPTURE, a);
    Timer::Channel ch2(timer, Timer::CH2, Timer::Channel::CAPTURE, b);

    ch1.enable(true);
    ch2.enable(true);
    timer.enable(true);
}

int TimerEncoder::count()
{
    return timer.get_count();
}

void TimerEncoder::reset()
{
    timer.set_count(0);
}
