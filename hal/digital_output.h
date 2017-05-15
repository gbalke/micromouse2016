#pragma once
#include "stdint.h"
#include "pin.h"

class DigitalOutput {
    public:
        DigitalOutput(Pin pin);
        void write(bool value);
        static void write_pin(uint8_t port, uint8_t pin, bool value);
    protected:
        uint8_t port_offset;
        uint8_t pin;
};
