#ifndef __IRSENSOR_H__
#define __IRSENSOR_H__

#include <mbed.h>

class IRSensor
{
	private:
		AnalogIn receiver_pin;
		DigitalOut emitter_pin;
		uint16_t lastRead;

	public:
		IRSensor (PinName rp, PinName ep);

		void read();
		uint16_t getValue();
};

#endif
