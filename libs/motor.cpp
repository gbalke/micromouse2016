#include "timer.h"
#include "motor.h"

Motor::Motor(HAL::Timer::Channel forward, HAL::Timer::Channel backward)
: forward(forward), backward(backward)
{
    set_speed(0);
    forward.enable(true);
    backward.enable(true);
}

void Motor::set_speed(int speed)
{
    if(speed >= 0) {
        backward.write(0);
        forward.write(speed);
    } else {
        forward.write(0);
        backward.write(-1*speed);
    }
}
