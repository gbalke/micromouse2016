#include "mbed.h"
#include "motor.h"
#include "encoder.h"

Motor left_motor(PB_6, PA_7);
Motor right_motor(PB_10, PC_7);

Encoder left_encoder(PA_1, PC_4, Encoder::X4);
Encoder right_encoder(PB_3, PA_15, Encoder::X4);

Serial serial(PA_9, PA_10);

int main()
{
    while(true) {
        serial.printf("Left: %d\r\n", left_encoder.count());
        serial.printf("Right: %d\r\n", right_encoder.count());
        wait(0.5);
    }
}
