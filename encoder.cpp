#include "mbed.h"
#include "encoder.h"

Encoder::Encoder(PinName a, PinName b, Encoding encoding, int divider)
: a(a), b(b), steps(0), divider(divider)
{
    switch(encoding) {
        case X4:
            this->b.rise(this, &Encoder::b_rise);
            this->b.fall(this, &Encoder::b_fall);
        case X2:
            this->a.fall(this, &Encoder::a_fall);
        case X1:
            this->a.rise(this, &Encoder::a_rise);
            break;
    }
}

int Encoder::count()
{
    return steps / divider;
}

void Encoder::reset()
{
    steps = 0;
}

void Encoder::a_rise()
{
    if(b) {
        steps++;
    } else {
        steps--;
    }
}

void Encoder::a_fall()
{
    if(b) {
        steps--;
    } else {
        steps++;
    }
}

void Encoder::b_rise()
{
    if(a) {
        steps--;
    } else {
        steps++;
    }
}

void Encoder::b_fall()
{
    if(a) {
        steps++;
    } else {
        steps--;
    }
}
