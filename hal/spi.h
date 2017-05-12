#pragma once
#include "pin.h"

struct spi_register {
    uint32_t cr1;
    uint32_t sr;
    uint32_t dr;
    uint32_t crcpr;
    uint32_t rxcrcr;
    uint32_t txcrcr;
    uint32_t i2scfgr;
    uint32_t i2spr;
    uint32_t reserved[248];
};

volatile spi_register *const spi5_base = (spi_register *const) 0x40015000;
volatile spi_register *const spi1_4_base = (spi_register *const) 0x40013000;
volatile spi_register *const spi2_3_base = (spi_register *const) 0x40003800;

class Spi {
    public:
        enum SpiNumber {
            SPI_NUMBER_1 = 0,
            SPI_NUMBER_2,
            SPI_NUMBER_3,
            SPI_NUMBER_4,
            SPI_NUMBER_5
        };
        enum Mode {
            SPI_MODE_0 = 0,
            SPI_MODE_1,
            SPI_MODE_2,
            SPI_MODE_3
        };
        enum FrameSize {
            ONE_BYTE,
            TWO_BYTE
        };
        Spi(SpiNumber spi, Pin sclk, Pin mosi, Pin miso, Pin ss=NC){}
        void set_mode(Mode mode){}
        void set_frame_size(FrameSize size){}
    private:
        volatile spi_register *spi;
};
