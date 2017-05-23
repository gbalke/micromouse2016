#include "irsensor.h"
#include "distance_sensor.h"

DistanceSensor::DistanceSensor(Pin reciever, Pin emitter)
: sensor(reciever, emitter)
{
    scale = 1;
}

double DistanceSensor::read()
{
    return (4095 - sensor.read() + sensor.read_ambient()) / scale;
}

uint16_t DistanceSensor::raw_read()
{
    return sensor.read();
}

uint16_t DistanceSensor::raw_ambient()
{
    return sensor.read_ambient();
}

void DistanceSensor::calibrate()
{
    scale = (float)(4095 - sensor.read() + sensor.read_ambient());
}
