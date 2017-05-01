#include "digital_input.h"
#include "PinNames.h"
#include "gpio.h"

DigitalInput::DigitalInput(PinName pin)
{
    port_offset = PORT_OFFSET(pin);
    this->pin = PIN_NUMBER(pin);
    gpio_enable_clock(port_offset);
    gpio_set_mode(port_offset, this->pin, INPUT);
}

bool DigitalInput::read()
{
    return read_pin(port_offset, this->pin);
}

bool DigitalInput::read_pin(uint8_t port, uint8_t pin)
{
    return gpio_base[port].idr & (0x1 << pin);
}
