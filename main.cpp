#include "mbed.h"
#include "motor.h"
#include "QEI.h"

Motor left_motor(PB_6, PA_7);
Motor right_motor(PB_10, PC_7);

QEI left_encoder(PA_15, PB_3, NC, 12, QEI::X4_ENCODING);
QEI right_encoder(PA_1, PC_4, NC, 12, QEI::X4_ENCODING);

Serial serial(PA_9, PA_10);

int main()
{
    left_encoder.reset();
    right_encoder.reset();
    while(right_encoder.getPulses() > -10 && right_encoder.getPulses() < 10) {
        left_motor.set_speed(0.1);
        right_motor.set_speed(0.1);
        serial.printf("Left: %d, Right: %d\n", left_encoder.getPulses(), right_encoder.getPulses());
    }
}
