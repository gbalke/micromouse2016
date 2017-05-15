#include "irsensor.h"
#include "digital_output.h"

IRSensor::IRSensor (PinName rp, PinName ep) : receiver_pin(rp), emitter_pin(ep) 
{
	lastRead = 0;
	emitter_pin.write(0);
}

uint16_t IRSensor::read()
{
    const int SAMPLE_SIZE = 20;
	// Read IR intensity from IR receiver.
	emitter_pin.write(1);
    uint32_t total = 0;
    for(int i = 0; i < SAMPLE_SIZE; i++) {
        // Divide by 16 since we only have a 12 bit ADC
        total += receiver_pin.read_u16() / 16;
    }
	emitter_pin.write(0);
    return total / SAMPLE_SIZE;
}
