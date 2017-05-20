#pragma once
#include "inttypes.h"
#include "pin.h"

class DigitalInput {
    public:
        DigitalInput(Pin pin);
        bool read();
        static bool read_pin(uint8_t port, uint8_t pin);
    protected:
        // The port of the pin (A-E, H)
        uint8_t port_offset;
        // the pin number within its port (0-15)
        uint8_t pin;
};
