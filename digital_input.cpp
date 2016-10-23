#include "digital_input.h"
#include "mbed-dev/hal/pinmap.h"

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
    char reserved[984];
};

gpio_port *const gpio_base = (gpio_port *const) 0x40020000;

DigitalInput::DigitalInput(PinName pin)
{
    port_offset = (pin & 0xF0) >> 4;
    this->pin = pin & 0x0F;
    gpio_base[port_offset].moder &= ~(0b11 << 2 * this->pin);
}

bool DigitalInput::read()
{
    return gpio_base[port_offset].idr & (0x1 << this->pin);
}
