#pragma once
#include <stdint.h>
#include "rcc.h"

// Configuration registers for a GPIO port. Refer to p.153 of the reference manual for purpose of
// each register.
struct gpio_port {
    uint32_t moder;
    uint32_t otyper;
    uint32_t ospeedr;
    uint32_t pupdr;
    uint32_t idr;
    uint32_t odr;
    uint32_t bsrr;
    uint32_t lckr;
    uint32_t afrl;
    uint32_t afrh;
    // needed to properly align ports
    uint32_t reserved[246];
};

// Location of GPIO port A in memory. Refer to p.54 of the datasheet
static volatile gpio_port *const gpio_base = (gpio_port *const) 0x40020000;

// Returns the port (A-E, H) of the pin
#define PORT_OFFSET(pin) (((pin) & 0xF0) >> 4)
// Returns the number within the port of a pin (0-15)
#define PIN_NUMBER(pin) ((pin) & 0x0F)

// 4 modes for a GPIO pin. refer to p.149-151 of the reference manual for details of each
// configuration.
enum GpioMode {
    INPUT = 0,
    OUTPUT,
    ALTERNATE_FUNCTION,
    ANALOG
};

// Enables the clock signal for the GPIO port. Required before doing any other GPIO configuration
inline void gpio_enable_clock(uint8_t port)
{
    rcc->ahb1enr |= 1 << port;
}

// Sets the the GPIO pin to the given configuration
inline void gpio_set_mode(uint8_t port, uint8_t pin, GpioMode mode)
{
    gpio_base[port].moder &= ~(0b11 << 2 * pin);
    gpio_base[port].moder |= (mode << 2 * pin);
}
