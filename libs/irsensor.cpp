#include "irsensor.h"

IRSensor::IRSensor (PinName rp, PinName ep) : receiver_pin(rp), emitter_pin(ep) 
{
	lastRead = 0;
	emitter_pin = 0;
}

void IRSensor::read()
{
	// Read IR intensity from IR receiver.
	emitter_pin = 1;
	wait(0.001);
	lastRead = receiver_pin.read_u16();
	emitter_pin = 0;
}

uint16_t IRSensor::getValue()
{
	return lastRead;
}
