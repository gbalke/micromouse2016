#include <cmath>
#include "mbed.h"
#include "motor.h"
#include "interrupt_encoder.h"
#include "irsensor.h"
#include "hal/gpio.h"

Serial serial(PA_2, PA_3);
SPI spi(PC_12, PC_11, PC_10);

PwmOut lback(PC_7);
PwmOut lfwd(PC_6);
PwmOut rfwd(PC_9);
PwmOut rback(PC_8);

DigitalInput sw1(PB_12);
DigitalInput sw2(PB_1);
DigitalInput sw3(PB_0);
DigitalInput sw4(PA_7);
DigitalOutput ss(PA_4);
DigitalOutput red(PB_15);
DigitalOutput green(PB_14);
DigitalOutput blue(PB_13);

int main()
{
    spi.format(16, 0);
    red.write(1);
    while(true) {
        if(!sw1.read()) {
            lback.write(0);
            rback.write(0);
            lfwd.write(0.1);
            rfwd.write(0.1);
        } else {
            lfwd.write(0);
            rfwd.write(0);
            lback.write(0);
            rback.write(0);
        }
        /*ss.write(0);
        int16_t lsb = spi.write((1 << 15) | (0x02 << 8));
        ss.write(1);
        ss.write(0);
        int16_t msb = spi.write((1 << 15) | (0x03 << 8));
        ss.write(1);
        serial.printf("%d\r\n", (msb << 8) | lsb);
        wait(1);*/
    }
}
