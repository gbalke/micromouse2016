#include "mbed.h"
#include "motor.h"

Motor::Motor(PinName forward, PinName backward, int multiplier)
: forward(forward), backward(backward), multiplier(multiplier)
{
    set_speed(0);
}

void Motor::set_speed(float speed)
{
    speed *= multiplier;
    if(speed >= 0) {
        backward.write(0);
        forward.write(speed);
    } else {
        forward.write(0);
        backward.write(-1*speed);
    }
}

float Motor::get_speed()
{
    // One pin should always be zero
    return (forward - backward) / multiplier;
}
