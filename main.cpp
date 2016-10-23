#include <cmath>
#include "mbed.h"
#include "motor.h"
#include "encoder.h"

Serial serial(PA_9, PA_10);

Motor left_motor(PB_6, PA_7, 3);
Motor right_motor(PC_7, PB_10);

Encoder left_encoder(PA_1, PC_4, Encoder::X4, sqrt(3));
Encoder right_encoder(PB_3, PA_15, Encoder::X4);

int main()
{
    const float P = 0.000001;
    left_motor.set_speed(0.1);
    right_motor.set_speed(0.1);
    while(true) {
        serial.printf("Left: %d\r\n", left_encoder.count());
        serial.printf("Right: %d\r\n", right_encoder.count());
        wait(0.5);
    }
}
