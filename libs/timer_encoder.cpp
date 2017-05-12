#include "pin.h"
#include "timer.h"
#include "timer_encoder.h"
#include "alternate_function.h"

// TODO: refactor this to initialize the timer using functions, instead of directly modifying registers

TimerEncoder::TimerEncoder(HAL::Timer::TimerModule t, Pin a, Pin b)
: timer(t, HAL::Timer::ClockSource::ENCODER_AB)
{
    timer.set_period(~0);
    timer.set_count(0);
    timer.set_channel_mode(HAL::Timer::Channel::CH1, HAL::Timer::ChannelMode::CAPTURE);
    timer.set_channel_mode(HAL::Timer::Channel::CH2, HAL::Timer::ChannelMode::CAPTURE);

    timer.enable_channel(HAL::Timer::Channel::CH1, true);
    timer.enable_channel(HAL::Timer::Channel::CH2, true);
    timer.enable_timer(true);

    AlternateFunction af;
    if(t == HAL::Timer::TimerModule::TIMER1 || t == HAL::Timer::TimerModule::TIMER2) {
        af = AF1;
    } else {
        af = AF2;
    }
    alternate_function_set_mode(a, af);
    alternate_function_set_mode(b, af);
}

int TimerEncoder::count()
{
    return timer.get_count();
}

void TimerEncoder::reset()
{
    timer.set_count(0);
}
