#include "irsensor.h"
#include "digital_output.h"

IRSensor::IRSensor (PinName rp, PinName ep) : receiver_pin(rp), emitter_pin(ep) 
{
	lastRead = 0;
	emitter_pin.write(0);
}

void IRSensor::read()
{
	// Read IR intensity from IR receiver.
	emitter_pin.write(1);
    wait(0.001);
    // Divide by 16 since we only have a 12 bit ADC
	lastRead = receiver_pin.read_u16() / 16;
	emitter_pin.write(0);
}

uint16_t IRSensor::getValue()
{
	return lastRead;
}
