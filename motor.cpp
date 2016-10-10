#include "mbed.h"
#include "motor.h"

Motor::Motor(PinName forward, PinName backward)
: forward(forward), backward(backward)
{
    set_speed(0);
}

void Motor::set_speed(float speed)
{
    if(speed >= 0) {
        backward.write(0);
        forward.write(speed);
    } else {
        forward.write(0);
        backward.write(-1*speed);
    }
}
