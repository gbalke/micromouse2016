#pragma once
#include "mbed.h"

class Encoder {
    public:
        enum Encoding {X1, X2, X4};
        Encoder(PinName a, PinName b, Encoding encoding);
        int count();
        void reset();
    private:
        InterruptIn a, b;
        volatile int steps;
        void rise();
        void fall();
};
