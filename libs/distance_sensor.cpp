#include "irsensor.h"
#include "distance_sensor.h"

static double ir_distance_exp(double reading, double scale, double decay_rate, double min)
{
    double dist = -1 * log((reading - min) / scale) / decay_rate;
    return dist;
}

static double ir_distance_logistic(double reading, double scale, double decay_rate,
                            double max, double midpoint)
{
    double dist = midpoint - log(scale / (max - reading) - 1) / decay_rate;
    return dist;
}

DistanceSensor::DistanceSensor(Pin reciever, Pin emitter, Model model, double scale,
                                double decay_rate, double offset, double midpoint = 0)
: sensor(reciever, emitter), scale(scale), decay_rate(decay_rate), offset(offset),
    midpoint(midpoint)
{
    this->model = model;
}
double DistanceSensor::read()
{
    if(model == LOGISTIC) {
        return ir_distance_logistic(sensor.read(), scale, decay_rate, offset, midpoint);
    } else {
        return ir_distance_exp(sensor.read(), scale, decay_rate, offset);
    }
}
