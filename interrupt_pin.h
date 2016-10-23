#pragma once
#include "mbed-dev/hal/pinmap.h"
#include "digital_input.h"

class InterruptPin : public DigitalInput {
    public:
        enum Edge {RISING, FALLING, BOTH};
        InterruptPin(PinName pin);
        void register_edge(Edge edge, void (*callback)());
        void register_edge(Edge edge, void (*callback)(void *), void *data);
        void unregister(Edge edge);
    private:
        void enable_exti(Edge edge);
};
