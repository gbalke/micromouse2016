#include "analog_input.h"
#include "PinNames.h"
#include "dma.h"

AnalogInput::AnalogInput (PinName pin)
{	
	this->pin = pin;
	// Sets up DMA 
	// ADC1 is on DMA2's Streams 0 and 4, channel 0.
	dma_init_struct init_vars = {
		.stream = DMAStream::ST0
		.chsel = DMAChannel::CH0
		.dir = DMATransfer::PM
		.pl = DMAPriority::HIGH
		.circ = false
	};

	dma_init(DMA.DMA_2, init_vars);
}

uint16_t AnalogInput::read()
{
		

}
