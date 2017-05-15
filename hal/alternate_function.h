#pragma once
#include "pin.h"
#include "gpio.h"

enum AlternateFunction {
    AF0 = 0, AF1, AF2, AF3, AF4, AF5, AF6, AF7, AF8, AF9, AF10, AF11, AF12, AF13, AF14, AF15
};

// Sets pin to the selected alternate function number. Refer to p.47 of the datasheet for which
// module corresponds to each alternate function number.
static inline void alternate_function_set_mode(Pin pin, AlternateFunction af)
{
    uint8_t port = PORT_OFFSET(pin);
    uint8_t pin_number = PIN_NUMBER(pin);
    gpio_enable_clock(port);
    gpio_set_mode(port, pin_number, ALTERNATE_FUNCTION);
    if(pin_number < 8) {
        gpio_base[port].afrl &= ~(0xF << 4 * pin_number);
        gpio_base[port].afrl |= af << 4 * pin_number;
    } else {
        gpio_base[port].afrh &= ~(0xF << 4 * (pin_number-8));
        gpio_base[port].afrh |= af << 4 * (pin_number-8);
    }
}
