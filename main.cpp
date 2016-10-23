#include <cmath>
#include "mbed.h"
#include "motor.h"
#include "encoder.h"

Serial serial(PA_9, PA_10);

Motor left_motor(PB_6, PA_7, 3);
Motor right_motor(PC_7, PB_10);

Encoder left_encoder(PA_1, PC_4, Encoder::X4, sqrt(3));
Encoder right_encoder(PB_3, PA_15, Encoder::X4);

void forward(int cells)
{
    const float P = 0;//0.001;
    // Forward 2
    while(left_encoder.count() < 500 * cells) {
        int error = left_encoder.count() - right_encoder.count();
        float correction = P * error;
        left_motor.set_speed(0.1 - correction);
        right_motor.set_speed(0.1 + correction);
        wait(0.1);
    }
    left_motor.set_speed(0);
    right_motor.set_speed(0);
    wait(0.1);
}

void right_turn(int times)
{
    int dir = (times > 0) ? 1 : -1;
    left_encoder.reset();
    right_encoder.reset();
    left_motor.set_speed(dir * 0.05);
    right_motor.set_speed(dir * -0.05);
    while(abs(left_encoder.count() - right_encoder.count()) < 200 * abs(times));
    left_motor.set_speed(0);
    right_motor.set_speed(0);
    wait(0.5);
}

int main()
{
    forward(2);
    right_turn(-1);
    forward(1);
    right_turn(1);
    forward(1);
    right_turn(3);
}
