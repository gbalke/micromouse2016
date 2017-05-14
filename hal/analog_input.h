#pragma once

#include "PinNames.h"

class AnalogInput {
	public:
		AnalogInput (PinName pin);
		static uint16_t read();
	private:
		uint8_t port_offset;
		uint8_t pin;
		
};
