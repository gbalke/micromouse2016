#ifndef __IRSENSOR_H__
#define __IRSENSOR_H__

#include <mbed.h>
#include "digital_output.h"

class IRSensor
{
	private:
		AnalogIn receiver_pin;
		DigitalOutput emitter_pin;
		uint16_t lastRead;

	public:
		IRSensor (PinName rp, PinName ep);

		uint16_t read();
};

#endif
