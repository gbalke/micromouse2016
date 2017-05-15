#pragma once
#include "pin.h"
#include "digital_input.h"

// Enables interrupts on a given pin. Pins with the same pin number but different port share an
// interrupt line, and only one may be enabled at a time. Pins 5-9 share a single interrupt line,
// but multiple may be active at a time. Same for pins 10-15.
class InterruptPin : public DigitalInput {
    public:
        enum Edge {RISING, FALLING, BOTH};
        InterruptPin(Pin pin);
        ~InterruptPin();
        // Sets the callback to be called on the given edge of the interrupt signal.
        void register_edge(Edge edge, void (*callback)());
        // Same as the other register_edge, but data is passed to callback as a parameter.
        void register_edge(Edge edge, void (*callback)(void *), void *data);
        void unregister(Edge edge);
    private:
        // Configures the EXTI module for the pin. Refer to p.198 of the reference manual
        // for information on EXTI.
        void enable_exti(Edge edge);
};
