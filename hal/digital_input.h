#pragma once
#include "inttypes.h"
#include "pin.h"

class DigitalInput {
    public:
        DigitalInput(Pin pin);
        bool read();
        static bool read_pin(uint8_t port, uint8_t pin);
    protected:
        uint8_t port_offset;
        uint8_t pin;
};
