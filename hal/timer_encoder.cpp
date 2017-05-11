#include "PinNames.h"

#include "rcc.h"
#include "timer.h"
#include "gpio.h"
#include "timer_encoder.h"

// TODO: refactor this to initialize the timer using functions, instead of directly modifying registers

TimerEncoder::TimerEncoder(TimerModule t, PinName a, PinName b)
{
    switch(t) {
        case TIMER1:
            rcc->apb2enr |= 1 << TIM1EN;
            timer = tim1_base;
            break;
        case TIMER2:
            rcc->apb1enr |= 1 << TIM2EN;
            timer = tim2_5_base;
            break;
        case TIMER3:
            rcc->apb1enr |= 1 << TIM3EN;
            timer = tim2_5_base + 1;
            break;
        case TIMER4:
            rcc->apb1enr |= 1 << TIM4EN;
            timer = tim2_5_base + 2;
            break;
        case TIMER5:
        rcc->apb1enr |= 1 << TIM5EN;
            timer = tim2_5_base + 3;
            break;
    }
    timer->arr = ~0;
    timer->cnt = 0;
    timer->ccmr1 |= CAPTURE << CC1S;
    timer->ccmr1 |= CAPTURE << CC2S;

    timer->smcr |= 0b011 << SMS;
    timer->ccer |= 1 << CC1E;
    timer->ccer |= 1 << CC2E;
    timer->cr1 |= 1 << CEN;

    uint8_t a_port = PORT_OFFSET(a);
    uint8_t a_pin = PIN_NUMBER(a);
    uint8_t b_port = PORT_OFFSET(b);
    uint8_t b_pin = PIN_NUMBER(b);
    int af;
    if(t == TIMER1 || t == TIMER2) {
        af = 1;
    } else {
        af = 2;
    }
    gpio_enable_clock(a_port);
    gpio_enable_clock(b_port);
    gpio_set_mode(a_port, a_pin, ALTERNATE_FUNCTION);
    gpio_set_mode(b_port, b_pin, ALTERNATE_FUNCTION);
    if(a_pin < 8) {
        gpio_base[a_port].afrl &= ~(0xF << 4 * a_pin);
        gpio_base[a_port].afrl |= af << 4 * a_pin;
    } else {
        gpio_base[a_port].afrh &= ~(0xF << 4 * (a_pin-8));
        gpio_base[a_port].afrh |= af << 4 * (a_pin-8);
    }
    if(b_pin < 8) {
        gpio_base[b_port].afrl &= ~(0xF << 4 * b_pin);
        gpio_base[b_port].afrl |= af << 4 * b_pin;
    } else {
        gpio_base[b_port].afrh &= ~(0xF << 4 * (b_pin-8));
        gpio_base[b_port].afrh |= af << 4 * (b_pin-8);
    }
}

int TimerEncoder::count()
{
    return timer->cnt;
}

void TimerEncoder::reset()
{
    timer->cnt = 0;
}
