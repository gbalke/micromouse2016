IRSensor::IRSensor (PinName ep, PinName rp) : emitter_pin(ep), receiver_pin(rp) 
{
	DigitalOut output(emitter_pin);
	AnalogIn input(receiver_pin);
}

uint16_t IRSensor::pulse()
{
	// Turn on IR LED.
	emitter_pin = 1;
	wait (IRDelay);
	// Read IR intensity from IR receiver.
	lastRead = receiver_pin.read_u16();
	// Turn off IR LED.
	wait (IRDelay);
	emitter_pin = 0;
	return lastRead;
}
