#include "mbed.h"
#include "encoder.h"

Encoder::Encoder(PinName a, PinName b, Encoding encoding)
: a(a), b(b), steps(0)
{
    switch(encoding) {
        case X4:
            this->b.rise(this, &Encoder::rise);
            this->b.fall(this, &Encoder::fall);
        case X2:
            this->a.fall(this, &Encoder::fall);
        case X1:
            this->a.rise(this, &Encoder::rise);
            break;
    }
}

int Encoder::count()
{
    return steps;
}

void Encoder::reset()
{
    steps = 0;
}

void Encoder::rise()
{
    if(b) {
        steps++;
    } else {
        steps--;
    }
}

void Encoder::fall()
{
    if(b) {
        steps--;
    } else {
        steps++;
    }
}
