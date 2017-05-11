#include "gyro.h"

Gyro::Gyro(PinName sclk, PinName mosi, PinName miso, PinName ss, uint32_t frequency)
: spi(mosi, miso, sclk), ss(ss)
{
    spi.format(16, 0);
    spi.frequency(frequency);
    // make sure gyro is initialized
    uint16_t chip_id;
    do {
        chip_id = read_reg(0x00);
    } while(chip_id != 0xF);
    // Bandwidth
    uint8_t bw;
    do {
        write_reg(0x10, 0x7);
        bw = read_reg(0x10);
    } while((bw & 0xF) != 0x7);
}

void Gyro::calibrate()
{
    uint8_t fast_offset;
    do {
        write_reg(0x32, 0xFC);
        fast_offset = read_reg(0x32);
    } while(fast_offset != 0xF4);
}

int16_t Gyro::read()
{
    uint8_t lsb = read_reg(0x06);
    uint8_t msb = read_reg(0x07);
    int16_t rate = (msb << 8) | lsb;
    return rate;
}

void Gyro::write_reg(uint8_t reg, uint8_t value)
{
    ss.write(0);
    spi.write((reg << 8) | value);
    ss.write(1);
    wait_ms(1);
}

uint8_t Gyro::read_reg(uint8_t reg)
{
    uint8_t value;
    ss.write(0);
    value = spi.write((1 << 15) | (reg << 8));
    ss.write(1);
    return value;
}
