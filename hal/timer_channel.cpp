#include "pin.h"
#include "timer.h"
#include "rcc.h"

namespace HAL {

Timer::Channel::Channel(Timer timer, ChannelNumber channel, Mode mode, Pin pin)
: timer(timer.timer), channel(channel)
{
    const int CHANNEL_SPACING = 8;
    if(channel <= CH2) {
        this->timer->ccmr1 |= mode << CHANNEL_SPACING * channel;
    } else {
        this->timer->ccmr2 |= mode << CHANNEL_SPACING * (channel - CH3);
    }
    alternate_function_set_mode(pin, timer.af);
}

void Timer::Channel::enable(bool enabled)
{
    timer->ccer |= enabled << 4 * channel;
}

void Timer::Channel::write(int value)
{
    timer->ccr[channel] = value;
}

int Timer::Channel::read()
{
    return timer->ccr[channel];
}

}
