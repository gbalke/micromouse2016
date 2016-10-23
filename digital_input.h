#pragma once
#include "mbed-dev/hal/pinmap.h"

class DigitalInput {
    public:
        DigitalInput(PinName pin);
        bool read();
        static bool read_pin(uint8_t port, uint8_t pin);
    protected:
        uint8_t port_offset;
        uint8_t pin;
};
