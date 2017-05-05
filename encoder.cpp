#include "timer.h"
#include "mbed.h"
#include "gpio.h"
#include "rcc.h"

Serial serial(PA_2, PA_3);

int main()
{
    PinName a = PA_1;
    PinName b = PA_0;
    uint8_t a_port = PORT_OFFSET(a);
    uint8_t a_pin = PIN_NUMBER(a);
    uint8_t b_port = PORT_OFFSET(b);
    uint8_t b_pin = PIN_NUMBER(b);

    rcc->apb1enr |= 1 << TIM5EN;
    rcc->apb1enr |= 1 << TIM2EN;
    tim2_5_base[0].arr = ~0;
    tim2_5_base[3].arr = ~0;

    tim2_5_base[0].ccmr1 |= 0b01 << CC1S;
    tim2_5_base[0].ccmr1 |= 0b01 << CC2S;
    tim2_5_base[0].smcr |= 0b011 << SMS;
    tim2_5_base[0].ccer |= 1 << CC1E;
    tim2_5_base[0].ccer |= 1 << CC2E;
    tim2_5_base[0].cr1 |= 1 << CEN;

    gpio_enable_clock(a_port);
    gpio_enable_clock(b_port);
    gpio_set_mode(a_port, a_pin, ALTERNATE_FUNCTION);
    gpio_set_mode(b_port, b_pin, ALTERNATE_FUNCTION);
    gpio_base[b_port].afrl &= ~(0xF << 4 * b_pin);
    gpio_base[b_port].afrl |= 1 << 4 * b_pin;
    gpio_base[a_port].afrl &= ~(0xF << 4 * a_pin);
    gpio_base[a_port].afrl |= 1 << 4 * a_pin;

    while(true) {
        serial.printf("CNT: %d\r\n", tim2_5_base[0].cnt);
        serial.printf("%x\r\n", tim2_5_base[0].ccmr1);
        serial.printf("%x\r\n", tim2_5_base[3].ccmr1);
    }
}
