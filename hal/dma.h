#pragma once
#include <cstdint>

// STM32F411 Reference Manual Section 9.5: Registers
const uint8_t num_streams = 8;

struct dma_stream {
	uint32_t scr;
	uint32_t sndtr;
	uint32_t spar;
	uint32_t sm0ar;
	uint32_t sm1ar;
	uint32_t sfcr;
};

struct dma_sections {
	uint32_t lisr;
	uint32_t hisr;
	uint32_t lifcr;
	uint32_t hifcr;
	dma_stream stream [num_streams];
	
	uint32_t reserved [6348];
};

enum DMA {
	DMA_1 = 0,
	DMA_2
};

enum DMAStream {
	ST0 = 0,
	ST1,
	ST2,
	ST3,
	ST4,
	ST5,
	ST6,
	ST7
};

enum DMAPriority {
	LOW = 0,
	MED,
	HIGH,
	VHIGH
};

// Section 9.3.3: Channel Selection
enum DMAChannel {
	CH0 = 0,
	CH1,
	CH2,
	CH3,
	CH4,
	CH5,
	CH6,
	CH7
};

// Section 9.3.6: Peripheral and Memory Transfers
enum DMATransfer {
	PM = 0, // Ex: Peripheral to Memory...
	MP,
	MM
};

// Section 9.5.5
struct dma_init_struct {
	DMAStream stream; 
						// Bits in SCR.
	DMAChannel chsel;	// 27:25
	uint8_t mburst = 0;	// 24:23
	uint8_t pburst = 0;	// 22:21
	bool ct = 0;		// 19
	bool dbm = 0;		// 18
	DMAPriority pl;		// 17:16
	bool pincos = 0;	// 15
	uint8_t msize = 1;	// 14:13
	uint8_t psize = 1;	// 12:11
	bool minc = 0;		// 10
	bool pinc = 0;		// 9
	bool circ = false;	// 8
	DMATransfer dir;	// 7:6
	bool pfctrl = 0;	// 5
	bool tcie = 0;		// 4
	bool htie = 0;		// 3
	bool teie = 0;		// 2
	bool dmeie = 0;		// 1

	// Constructing data for stream configuration register.
	uint32_t make_configuration_data ()
	{
		uint32_t data = 0;

		data |= (chsel << 25);
		data |= (mburst << 23);
		data |= (pburst << 21);
		data |= (ct << 19);
		data |= (dbm << 18);
		data |= (pl << 16);
		data |= (pincos << 15);
		data |= (msize << 13);
		data |= (psize << 11);
		data |= (minc << 10);
		data |= (pinc << 9);
		data |= (circ << 8);
		data |= (dir << 6);
		data |= (pfctrl << 5);
		data |= (tcie << 4);
		data |= (htie << 3);
		data |= (teie << 2);
		data |= (dmeie << 1);

		return data;
	}
};


static volatile dma_sections *const dma = (dma_sections *const) 0x40026000;

static inline void dma_init(DMA dma_num, dma_init_struct& init) 
{
	
	
	uint32_t clear_val = 1;
	
	// Enabling register modification.
	dma[dma_num].stream[init.stream].scr &= clear_val;
	// Setting Configuration.
	dma[dma_num].stream[init.stream].scr |= init.make_configuration_data();

	dma[dma_num].stream[init.stream].scr &= 0b0; 
}	
