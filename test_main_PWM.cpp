#include "mbed.h"
#include "gpio.h"
#include "rcc.h"
#include "timer.h"

Serial serial(PA_2, PA_3);

// PWM Constants
#define PERIOD 1000

int main()
{
    PinName pwm = PB_9;
    uint8_t port_offset = PORT_OFFSET(pwm);
    uint8_t pin = PIN_NUMBER(pwm);

    rcc->apb1enr |= 1 << TIM4EN;
    tim2_5_base[2].arr = PERIOD; // Sets PWM Period
    tim2_5_base[2].egr |= 1 << UG; // Triggers channel (needed for preload registers)
    tim2_5_base[2].cr1 |= 1 << CEN; // Enables Timer

    tim2_5_base[2].ccmr2 |= COMPARE << CC4S; // Sets channel to output
    tim2_5_base[2].ccmr2 |= PWM1 << OC4M; // sets PWM output
    tim2_5_base[2].ccr4 = 500; // Sets duty cycle
    tim2_5_base[2].ccer |= 1 << CC4E; // Enables Channel


    gpio_enable_clock(port_offset);
    gpio_set_mode(port_offset, pin, ALTERNATE_FUNCTION);
    gpio_base[port_offset].afrh &= ~(0xF << 4 * (pin-8));
    gpio_base[port_offset].afrh |= 2 << 4 * (pin-8);

    while(true) {
        serial.printf("CNT: %d\r\n", tim2_5_base[2].cnt);
        serial.printf("Output: %d\r\n", (gpio_base[port_offset].idr >> pin) & 1);
    }
}
