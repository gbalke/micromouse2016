#include "mbed.h"
#include "motor.h"

Motor::Motor(PinName forward, PinName backward, float multiplier)
: forward(forward), backward(backward), multiplier(multiplier)
{
    set_speed(0);
}

void Motor::set_speed(float speed)
{
    const float MAX_SPEED = 0.3;
    if(speed > MAX_SPEED) {
        speed = MAX_SPEED;
    } else if(speed < -1 * MAX_SPEED) {
        speed = -1 * MAX_SPEED;
    }
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
