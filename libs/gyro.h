#include "inttypes.h"

#include "mbed.h"
#include "digital_output.h"

class Gyro {
    public:
        Gyro(PinName sclk, PinName mosi, PinName miso, PinName ss, uint32_t frequency);
        void calibrate();
        int16_t read();
    private:
        void write_reg(uint8_t reg, uint8_t value);
        uint8_t read_reg(uint8_t reg);
        SPI spi;
        DigitalOutput ss;
};
