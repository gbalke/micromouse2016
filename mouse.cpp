#include "mbed.h"

#include "motor.h"
#include "timer_encoder.h"
#include "interrupt_encoder.h"
#include "irsensor.h"
#include "gyro.h"

Serial serial(PA_2, PA_3);

Motor left(PC_6, PC_7);
Motor right(PC_9, PC_8);

TimerEncoder left_encoder(TIMER2, PA_1, PA_0);

DigitalInput sw1(PB_12);
DigitalInput sw2(PB_1);
DigitalInput sw3(PB_0);
DigitalInput sw4(PA_7);
DigitalOutput red(PB_15);
DigitalOutput green(PB_14);
DigitalOutput blue(PB_13);

Gyro gyro(PC_10, PC_12, PC_11, PA_4, 1e7);

int main()
{
    gyro.calibrate();
    while(true) {
        if(!sw1.read()) {
            left.set_speed(0.1);
            right.set_speed(0.1);
        } else {
            left.set_speed(0);
            right.set_speed(0);
        }
        serial.printf("Gyro: %d\r\n", gyro.read());
    }
}
