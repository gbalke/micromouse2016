#pragma once
#include "mbed.h"
#include "digital_input.h"

class Encoder {
    public:
        enum Encoding {X1, X2, X4};
        Encoder(PinName a, PinName b, Encoding encoding, float divider = 1);
        int count();
        void reset();
    private:
        InterruptIn a, b;
        DigitalInput a_in, b_in;
        float divider;
        volatile int steps;
        void a_rise();
        void a_fall();
        void b_rise();
        void b_fall();
};
