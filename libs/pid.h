#pragma once

class Pid {
    public:
        Pid(double P, double I, double D);
        double correction(double error);
        void reset();
        double get_derivative();
    private:
        const double P;
        const double I;
        const double D;
        double prev_error;
        double derivative;
        double integral;
};
