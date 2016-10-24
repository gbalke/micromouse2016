#ifndef __IRSENSOR_H__
#define __IRSENSOR_H__

#include <mbed.h>

const float IRDelay = 0.001f;

class IRSensor
{
	private:
		PinName emitter_pin;
		PinName receiver_pin;
		uint16_t lastRead;

	public:
		IRSensor (PinName ep, PinName rp);

		uint16_t pulse();
};

#endif
