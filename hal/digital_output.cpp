#include "gpio.h"
#include "digital_output.h"
#include "PinNames.h"

DigitalOutput::DigitalOutput(PinName pin)
{
    port_offset = (pin & 0xF0) >> 4;
    this->pin = pin & 0x0F;
    // Sets pin to output mode
    gpio_base[port_offset].moder &= ~(0b11 << 2 * this->pin);
    gpio_base[port_offset].moder |= (0b01 << 2 * this->pin);
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
