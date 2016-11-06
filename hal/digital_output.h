#pragma once
#include "PinNames.h"

class DigitalOutput {
    public:
        DigitalOutput(PinName pin);
        void write(bool value);
        static void write_pin(uint8_t port, uint8_t pin, bool value);
    protected:
        uint8_t port_offset;
        uint8_t pin;
};
