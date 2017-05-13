#include "pid.h"

Pid::Pid(double P, double I, double D)
: P(P), I(I), D(D)
{
    prev_error = 0;
    integral = 0;
}

double Pid::correction(double error)
{
    double derivative = error - prev_error;
    integral += error;
    return P * error + I * integral + D * derivative;
}

void Pid::reset()
{
    integral = 0;
    prev_error = 0;
}
