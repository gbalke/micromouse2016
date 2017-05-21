#include "pid.h"
#include "float.h"

Pid::Pid(double P, double I, double D)
: P(P), I(I), D(D)
{
    prev_error = 0;
    integral = 0;
}

double Pid::correction(double error)
{
    derivative = error - prev_error;
    prev_error = error;
    if(error > 0 && integral >= DBL_MAX - error) {
        integral = DBL_MAX;
    } else if(error < 0 && integral <= -DBL_MAX - error) {
        integral = -DBL_MAX;
    } else {
        integral += error;
    }
    return P * error + I * integral + D * derivative;
}

void Pid::reset()
{
    integral = 0;
    prev_error = 0;
}

double Pid::get_derivative()
{
    return derivative;
}
