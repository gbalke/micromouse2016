#include "pin.h"
#include "interrupt_encoder.h"
#include "digital_input.h"
#include "interrupt_pin.h"

InterruptEncoder::InterruptEncoder(PinName a, PinName b, Encoding encoding, float divider)
: a(a), b(b), divider(divider), steps(0)
{
    switch(encoding) {
        case X0:
            this->a.register_edge(InterruptPin::BOTH, (void (*)(void *)) inc, (void*)this);
            break;
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

int InterruptEncoder::count()
{
    return steps / divider;
}

void InterruptEncoder::reset()
{
    steps = 0;
}

void InterruptEncoder::inc(InterruptEncoder &e)
{
    e.steps++;
}

void InterruptEncoder::a_rise(InterruptEncoder &e)
{
    if(e.b.read()) {
        e.steps++;
    } else {
        e.steps--;
    }
}

void InterruptEncoder::a_fall(InterruptEncoder &e)
{
    if(e.b.read()) {
        e.steps--;
    } else {
        e.steps++;
    }
}

void InterruptEncoder::b_rise(InterruptEncoder &e)
{
    if(e.a.read()) {
        e.steps--;
    } else {
        e.steps++;
    }
}

void InterruptEncoder::b_fall(InterruptEncoder &e)
{
    if(e.a.read()) {
        e.steps++;
    } else {
        e.steps--;
    }
}
