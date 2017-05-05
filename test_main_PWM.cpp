#include "mbed.h"
#include "gpio.h"
#include "rcc.h"

Serial serial(PA_2, PA_3);

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
#define OC2M 12
#define OC4M 12

// Channel Input Constants (+8 to reach CH2)
#define IC1F 4
#define IC1PSC 2

// Channel Mode Constants (+8 to reach CH2)
#define CC1S 0
#define CC2S 8
#define CC4S 8

// CCER Constants (+4 for next channel)
#define CC1E 0
#define CC1P 1
#define CC1NP 3
#define CC2E 4
#define CC4E 12

// PWM Constants
#define PERIOD 1000

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

#define TIM3EN 1
#define TIM4EN 2

int main()
{
    PinName pwm = PB_9;
    uint8_t port_offset = PORT_OFFSET(pwm);
    uint8_t pin = PIN_NUMBER(pwm);

    rcc->apb1enr |= 1 << TIM4EN;
    tim2_5_base[2].arr = PERIOD; // Sets PWM Period
    tim2_5_base[2].egr |= 1 << UG; // Triggers channel (needed for preload registers)
    tim2_5_base[2].cr1 |= 1 << CEN; // Enables Timer

    tim2_5_base[2].ccmr2 |= COMPARE << CC4S; // Sets channel to output
    tim2_5_base[2].ccmr2 |= PWM1 << OC4M; // sets PWM output
    tim2_5_base[2].ccr4 = 500; // Sets duty cycle
    tim2_5_base[2].ccer |= 1 << CC4E; // Enables Channel


    gpio_enable_clock(port_offset);
    gpio_set_mode(port_offset, pin, ALTERNATE_FUNCTION);
    gpio_base[port_offset].afrh &= ~(0xF << 4 * (pin-8));
    gpio_base[port_offset].afrh |= 2 << 4 * (pin-8);

    while(true) {
        serial.printf("CNT: %d\r\n", tim2_5_base[2].cnt);
        serial.printf("Output: %d\r\n", (gpio_base[port_offset].idr >> pin) & 1);
    }
}
