#pragma once
#include "mbed-dev/hal/pinmap.h"
#include "digital_input.h"
#include "interrupt_pin.h"

class Encoder {
    public:
        enum Encoding {X0, X1, X2, X4};
        Encoder(PinName a, PinName b, Encoding encoding, float divider = 1);
        int count();
        void reset();
    private:
        InterruptPin a, b;
        float divider;
        volatile int steps;
        static void a_rise(Encoder &e);
        static void a_fall(Encoder &e);
        static void b_rise(Encoder &e);
        static void b_fall(Encoder &e);
        static void inc(Encoder &e);
};
