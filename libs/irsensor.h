#ifndef __IRSENSOR_H__
#define __IRSENSOR_H__

#include <mbed.h>
#include "digital_output.h"

class IRSensor
{
	private:
		AnalogIn receiver_pin;
		DigitalOutput emitter_pin;

	public:
		IRSensor (PinName rp, PinName ep);

		uint16_t read();
        // reads without emitting, to measure ambient light
        uint16_t read_ambient();
};

#endif
