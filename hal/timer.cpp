#include "pin.h"
#include "timer.h"
#include "rcc.h"
#include "alternate_function.h"

namespace HAL {

Timer::Timer(TimerNumber timer, ClockSource source)
{
    if(timer == TIMER1) {
        rcc->apb2enr |= 1 << TIM1EN;
        this->timer = tim1_base;
    } else {
        rcc->apb1enr |= 1 << (TIM2EN + timer - 2);
            this->timer = tim2_5_base + timer - 2;
    }
    this->timer->smcr |= source << SMS;
    if(timer == TIMER1 || timer == TIMER2) {
        af = AF1;
    } else {
        af = AF2;
    }
}


void Timer::enable(bool enabled)
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
