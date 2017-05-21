#pragma once

class DistanceSensor {
    public:
        enum Model {
            LOGISTIC,
            EXPONENTIAL
        };
        DistanceSensor(Pin reciever, Pin emitter, Model model, double scale, double decay_rate,
                        double offset, double midpoint);
        double read();
        int raw_read();
    private:
        IRSensor sensor;
        Model model;
        const double scale;
        const double decay_rate;
        const double offset;
        const double midpoint;
};
