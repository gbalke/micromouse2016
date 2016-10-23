#include "mbed-dev/hal/pinmap.h"
#include "encoder.h"
#include "digital_input.h"
#include "interrupt_pin.h"

Encoder::Encoder(PinName a, PinName b, Encoding encoding, float divider)
: a(a), b(b), divider(divider), steps(0)
{
    switch(encoding) {
        case X4:
            this->b.register_edge(InterruptPin::RISING, (void (*)(void *)) b_rise, (void*)this);
            this->b.register_edge(InterruptPin::FALLING, (void (*)(void *)) b_fall, (void*)this);
        case X2:
            this->a.register_edge(InterruptPin::FALLING, (void (*)(void *)) a_fall, (void*)this);
        case X1:
            this->a.register_edge(InterruptPin::RISING, (void (*)(void *)) a_rise, (void*)this);
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

void Encoder::a_rise(Encoder &e)
{
    if(e.b.read()) {
        e.steps++;
    } else {
        e.steps--;
    }
}

void Encoder::a_fall(Encoder &e)
{
    if(e.b.read()) {
        e.steps--;
    } else {
        e.steps++;
    }
}

void Encoder::b_rise(Encoder &e)
{
    if(e.a.read()) {
        e.steps--;
    } else {
        e.steps++;
    }
}

void Encoder::b_fall(Encoder &e)
{
    if(e.a.read()) {
        e.steps++;
    } else {
        e.steps--;
    }
}
