#pragma once
#include "stdint.h"
#include "pin.h"

class DigitalOutput {
    public:
        DigitalOutput(Pin pin);
        void write(bool value);
        static void write_pin(uint8_t port, uint8_t pin, bool value);
    protected:
        // The port of the pin (A-E, H)
        uint8_t port_offset;
        // the pin number within its port (0-15)
        uint8_t pin;
};
