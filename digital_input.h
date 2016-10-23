#pragma once
#include "mbed-dev/hal/pinmap.h"

class DigitalInput {
    public:
        DigitalInput(PinName pin);
        bool read();
    protected:
        uint8_t port_offset;
        uint8_t pin;
};
