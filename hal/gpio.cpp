#include <stdint.h>

#include "rcc.h"
#include "gpio.h"

void gpio_enable_clock(uint8_t port)
{
    rcc->ahb1enr |= 1 << port;
}

void gpio_set_mode(uint8_t port, uint8_t pin, GpioMode mode)
{
    gpio_base[port].moder &= ~(0b11 << 2 * pin);
    gpio_base[port].moder |= (mode << 2 * pin);
}
