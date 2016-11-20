#include "gpio.h"
#include "digital_output.h"
#include "PinNames.h"

DigitalOutput::DigitalOutput(PinName pin)
{
    port_offset = PORT_OFFSET(pin);
    this->pin = PIN_NUMBER(pin);
    gpio_set_mode(port_offset, this->pin, OUTPUT);
}

void DigitalOutput::write(bool value)
{
    return write_pin(port_offset, this->pin, value);
}

void DigitalOutput::write_pin(uint8_t port, uint8_t pin, bool value)
{
    if(value) {
        gpio_base[port].bsrr |= 1 << pin;
    } else {
        gpio_base[port].bsrr |= 1 << (pin + 16);
    }
}
