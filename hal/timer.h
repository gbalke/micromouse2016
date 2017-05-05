#pragma once
#include <cstdint>

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

// CR1 Constants
#define CEN 0

// EGR Constants
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

enum Timer {
    TIMER1,
    TIMER2,
    TIMER3,
    TIMER4,
    TIMER5
};

enum ChannelMode {
    COMPARE = 0b00,
    CAPTURE = 0b01,
    // Capture other channel's input
    CAPTURE_FLIPPED = 0b10,
    CAPTURE_TRC = 0b11,
};

enum ChannelOutputMode {
    FROZEN = 0b000,
    EQUAL = 0b001,
    NOT_EQUAL = 0b010,
    TOGGLE = 0b011,
    FORCE_LOW = 0b100,
    FORCE_HIGH = 0b101,
    PWM1 = 0b110,
    PWM2 = 0b111
};

#define TIM1EN 0
#define TIM2EN 0
#define TIM3EN 1
#define TIM4EN 2
#define TIM5EN 3
#define SMS 0
