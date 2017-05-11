#include "gyro.h"

#define CHIP_ID 0x00
#define VALID_ID 0xF
#define BANDWIDTH 0x10
#define LOWPASS 0x7
#define FAST_OFFSET_CALIBRATION 0x32
#define CALIBRATE_Z 0xFC
#define Z_CALIBRATED 0xF4
#define Z_LSB 0x06
#define Z_MSB 0x07

Gyro::Gyro(PinName sclk, PinName mosi, PinName miso, PinName ss, uint32_t frequency)
: spi(mosi, miso, sclk), ss(ss)
{
    spi.format(16, 0);
    spi.frequency(frequency);
    // make sure gyro is initialized
    uint16_t chip_id;
    do {
        chip_id = read_reg(CHIP_ID);
    } while(chip_id != VALID_ID);
    // Bandwidth
    uint8_t bw;
    do {
        write_reg(BANDWIDTH, LOWPASS);
        bw = read_reg(BANDWIDTH);
    } while((bw & 0xF) != LOWPASS);
}

void Gyro::calibrate()
{
    uint8_t fast_offset;
    do {
        write_reg(FAST_OFFSET_CALIBRATION, CALIBRATE_Z);
        fast_offset = read_reg(FAST_OFFSET_CALIBRATION);
    } while(fast_offset != Z_CALIBRATED);
}

int16_t Gyro::read()
{
    uint8_t lsb = read_reg(Z_LSB);
    uint8_t msb = read_reg(Z_MSB);
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
