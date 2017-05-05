#pragma once
#include <stdint.h>

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

static volatile gpio_port *const gpio_base = (gpio_port *const) 0x40020000;

#define PORT_OFFSET(pin) (((pin) & 0xF0) >> 4)
#define PIN_NUMBER(pin) ((pin) & 0x0F)

enum GpioMode {
    INPUT = 0,
    OUTPUT,
    ALTERNATE_FUNCTION,
    ANALOG
};

void gpio_enable_clock(uint8_t port);
void gpio_set_mode(uint8_t port, uint8_t pin, GpioMode mode);
