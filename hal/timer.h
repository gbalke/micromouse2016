#pragma once
#include <cstdint>

// Temporary namespace to deal with mbed name clash
namespace HAL {

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

class Timer {
    public:
        enum TimerModule {
            TIMER1 = 1,
            TIMER2,
            TIMER3,
            TIMER4,
            TIMER5
        };
        enum Channel {
            CH1 = 0, CH2, CH3, CH4
        };
        enum ClockSource {
            CPU = 0,
            ENCODER_B,
            ENCODER_A,
            ENCODER_AB
        };
        enum ChannelMode {
            COMPARE = 0b00,
            CAPTURE = 0b01,
            // Capture other channel's input
            CAPTURE_FLIPPED = 0b10,
            CAPTURE_TRC = 0b11,
        };
        Timer(TimerModule timer, ClockSource source);
        void enable_timer(bool enabled);
        void enable_channel(Channel channel, bool enabled);
        void set_channel_mode(Channel channel, ChannelMode mode);
        void set_period(int period);
        void set_count(int count);
        int get_count();
    private:
        volatile timer_register *timer;
};


static volatile timer_register *const tim1_base = (timer_register *const) 0x40010000;
static volatile timer_register *const tim2_5_base = (timer_register *const) 0x40000000;
static volatile timer_register *const tim9_11_base = (timer_register *const) 0x40014000;

// CR1 Constants
#define CEN 0

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

#define SMS 0
#define TIM1EN 0
#define TIM2EN 0

}
