#include "timer.h"
#include "mbed.h"
#include "gpio.h"
#include "rcc.h"

Serial serial(PA_2, PA_3);

#include "PinNames.h"

#include "timer_encoder.h"
#include "mbed.h"

int main()
{
    TimerEncoder t(TIMER2, PA_1, PA_0);
    while(true) {
        serial.printf("CNT: %d\r\n", t.count());
    }
}
