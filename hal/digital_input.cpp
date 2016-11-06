#include "digital_input.h"
#include "PinNames.h"
#include "gpio.h"

DigitalInput::DigitalInput(PinName pin)
{
    port_offset = (pin & 0xF0) >> 4;
    this->pin = pin & 0x0F;
    // Sets pin to input mode
    gpio_base[port_offset].moder &= ~(0b11 << 2 * this->pin);
}

bool DigitalInput::read()
{
    return read_pin(port_offset, this->pin);
}

bool DigitalInput::read_pin(uint8_t port, uint8_t pin)
{
    return gpio_base[port].idr & (0x1 << pin);
}
