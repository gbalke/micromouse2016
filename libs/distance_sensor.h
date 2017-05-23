#pragma once

class DistanceSensor {
    public:
        DistanceSensor(Pin reciever, Pin emitter);
        double read();
        void calibrate();
        uint16_t raw_read();
        uint16_t raw_ambient();
    private:
        IRSensor sensor;
        double scale;
};
