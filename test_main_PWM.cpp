#include <cmath>
#include "mbed.h"
#include "motor.h"
#include "encoder.h"

#include "gpio.h"

Serial serial(PA_9, PA_10);

Motor left_motor(PB_6, PA_7, 3);
Motor right_motor(PC_7, PB_10);

Encoder left_encoder(PA_1, PC_4, Encoder::X2, 1.72);
Encoder right_encoder(PA_15, PB_3, Encoder::X0);

struct timer_register {
    uint32_t cr1;
    uint32_t cr2; // Not used for TIM9-11
    uint32_t smcr;
    uint32_t dier;
    uint32_t sr;
    uint32_t egr;
    uint32_t ccmr1;
    uint32_t ccmr2; // Not used for TIM9-11
    uint32_t ccer;
    uint32_t cnt;
    uint32_t psc;
    uint32_t arr;
    uint32_t rcr; // Only used for TIM1
    uint32_t ccr1;
    uint32_t ccr2; // Not used for TIM10-11
    uint32_t ccr3; // Not used for TIM9-11
    uint32_t ccr4; // Not used for TIM9-11
    uint32_t bdtr; // Only used for TIM1
    uint32_t dcr; // Not used for TIM9-11
    uint32_t dmar; // Not used for TIM9-11
    uint32_t tim_or; // Only used for TIM2-5 and TIM10-11
    uint32_t reserved3[235];
};

static volatile timer_register *const tim1_base = (timer_register *const) 0x40010000;
static volatile timer_register *const tim2_5_base = (timer_register *const) 0x40000000;
static volatile timer_register *const tim9_11_base = (timer_register *const) 0x40014000;

// Timer Constants
#define CEN 0
#define UG 0

// Channel Output Constants (+8 to reach CH2)
#define OC1CE 7
#define OC1M 4
#define OC1PE 3
#define OC1FE 2

// Channel Input Constants (+8 to reach CH2)
#define IC1F 4
#define IC1PSC 2

// Channel Mode Constants (+8 to reach CH2)
#define CC1S 0

// CCER Constants (+4 for next channel)
#define CC1E 0
#define CC1P 1
#define CC1NP 3

// PWM Constants
#define PERIOD 255

enum ChannelMode {
    COMPARE = 0,
    CAPTURE,
    // Capture other channel's input
    CAPTURE_FLIPPED,
    CAPTURE_TRC,
};

enum ChannelOutputMode {
    FROZEN = 0,
    EQUAL,
    NOT_EQUAL,
    TOGGLE,
    FORCE_LOW,
    FORCE_HIGH,
    PWM1,
    PWM2
};

int main()
{
    PinName pwm = PA_6;
    uint8_t port_offset = PORT_OFFSET(pwm);
    uint8_t pin = PIN_NUMBER(pwm);

    tim2_5_base[1].arr = PERIOD; // Sets PWM Period
    tim2_5_base[1].egr |= 1 << UG; // Triggers channel (needed for preload registers)
    tim2_5_base[1].cr1 |= 1 << CEN; // Enables Timer

    tim2_5_base[1].ccmr1 |= COMPARE << CC1S; // Sets channel to output
    tim2_5_base[1].ccmr1 |= PWM1 << OC1M; // sets PWM output
    tim2_5_base[1].ccr1 = 127; // Sets duty cycle
    tim2_5_base[1].ccer |= 1 << CC1E; // Enables Channel


    gpio_set_mode(port_offset, pin, ALTERNATE_FUNCTION);
    gpio_base[port_offset].afrl &= ~(0xF << 4 * pin);
    gpio_base[port_offset].afrl |= 2 << 4 * pin; // TIM3_CH1

    while(true) {
        serial.printf("CNT: %d\r\n", tim2_5_base[1].cnt);
        serial.printf("Output: %d\r\n", (gpio_base[port_offset].idr >> pin) & 1);
    }
}
