#pragma once
#include "pin.h"
#include "digital_input.h"
#include "interrupt_pin.h"

class InterruptEncoder {
    public:
        enum Encoding {X0, X1, X2, X4};
        InterruptEncoder(PinName a, PinName b, Encoding encoding, float divider = 1);
        int count();
        void reset();
    private:
        InterruptPin a, b;
        float divider;
        volatile int steps;
        static void a_rise(InterruptEncoder &e);
        static void a_fall(InterruptEncoder &e);
        static void b_rise(InterruptEncoder &e);
        static void b_fall(InterruptEncoder &e);
        static void inc(InterruptEncoder &e);
};
