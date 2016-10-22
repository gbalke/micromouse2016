#include "mbed.h"
#include "motor.h"
#include "encoder.h"

Serial serial(PA_9, PA_10);

Motor left_motor(PB_6, PA_7, 3);
Motor right_motor(PB_10, PC_7);

Encoder left_encoder(PA_1, PC_4, Encoder::X4, 1);
//Encoder right_encoder(PB_3, PA_15, Encoder::X4, 1);

int main()
{
    const float P = 0.000001;
    DigitalIn a(PB_3);
    DigitalIn b(PA_15);
    while(true) {
        serial.printf("A: %d\r\n", a.read());
        serial.printf("B: %d\r\n", b.read());
        wait(0.5);
    }
}
