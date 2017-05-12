#include "pin.h"
#include "timer.h"
#include "rcc.h"

namespace HAL {

Timer::Timer(TimerModule timer, ClockSource source)
{
    if(timer == TIMER1) {
        rcc->apb2enr |= 1 << TIM1EN;
        this->timer = tim1_base;
    } else {
        rcc->apb1enr |= 1 << (TIM2EN + timer - 2);
            this->timer = tim2_5_base + timer - 2;
    }
    this->timer->smcr |= source << SMS;
}

void Timer::set_channel_mode(Channel channel, ChannelMode mode)
{
    const int CHANNEL_SPACING = 8;
    if(channel <= CH2) {
        timer->ccmr1 |= mode << CHANNEL_SPACING * channel;
    } else {
        timer->ccmr2 |= mode << CHANNEL_SPACING * (channel - CH3);
    }
}

void Timer::enable_channel(Channel channel, bool enabled)
{
    timer->ccer |= enabled << 4 * channel;
}

void Timer::enable_timer(bool enabled)
{
    timer->cr1 |= enabled << CEN;
}

void Timer::set_period(int period)
{
    timer->arr = period;
}

void Timer::set_count(int count)
{
    timer->cnt = count;
}

int Timer::get_count()
{
    return timer->cnt;
}

}
