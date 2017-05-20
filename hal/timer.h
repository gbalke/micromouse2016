#pragma once
#include <cstdint>
#include "alternate_function.h"

// Temporary namespace to deal with mbed name clash
namespace HAL {

#define CEN 0
#define SMS 0
#define TIM1EN 0
#define TIM2EN 0

// Config registers for each timer. Certain registers are only available for certain timers
// Refer to p.278, p.342, p.385, and p.400 of the reference manual
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
    uint32_t ccr[4]; // only 1-2 for TIM9 and 1 for TIM10-11
    uint32_t bdtr; // Only used for TIM1
    uint32_t dcr; // Not used for TIM9-11
    uint32_t dmar; // Not used for TIM9-11
    uint32_t tim_or; // Only used for TIM2-5 and TIM10-11
    uint32_t reserved3[235];
};

class Timer {
    public:
        // Options for Timer modules to use. Currently Timers 9-11 are not supported.
        enum TimerNumber {
            TIMER1 = 1,
            TIMER2,
            TIMER3,
            TIMER4,
            TIMER5
        };
        // The 4 compare/capture channels for each timer. Timers 9 only has 2 channels, and
        // 10-11 only have 1 channel each
        enum ChannelNumber {
            CH1 = 0, CH2, CH3, CH4
        };
        // Supported clock sources for each timer. ENCODER_B counts on the edges of encoder signal
        // B, ENCODER_A on the edges of signal a, and ENCODER_AB on edges of both signals
        enum ClockSource {
            CPU = 0,
            ENCODER_B,
            ENCODER_A,
            ENCODER_AB
        };

        // Creates a disabled timer with a period of 0
        Timer(TimerNumber timer, ClockSource source);
        void enable(bool enabled);
        void set_period(int period);
        // sets the timer's counter
        void set_count(int count);
        // reads the timer's counter
        int get_count();

        class Channel {
            public:
                // Input/output modes supported by each channel. Refer to p.254-261 for Timer 1,
                // p.320-327 for Timers 2-5, and p.375-p.380 of the reference manual details of
                // each mode.
                enum Mode {
                    CAPTURE = 0x1,
                    // Capture other channel's input
                    CAPTURE_FLIPPED = 0x2,
                    CAPTURE_TRC = 0x3,
                    COMPARE_FROZEN = 0x00,
                    COMPARE_EQUAL = 0x10,
                    COMPARE_NOT_EQUAL = 0x20,
                    COMPARE_TOGGLE = 0x30,
                    COMPARE_FORCE_LOW = 0x40,
                    COMPARE_FORCE_HIGH = 0x50,
                    COMPARE_PWM1 = 0x60,
                    COMPARE_PWM2 = 0x70
                };

                // Creates a timer channel in the given mode connected to the given mode.
                // Each channel only supports certain pins; refer to p.47 of the datasheet for
                // details. Channel begins as disabled.
                Channel(Timer timer, ChannelNumber channel, Mode mode, Pin pin);
                void enable(bool enabled);
                // Reads the input from the channel (used for capture channels)
                int read();
                // Configures output for a compare channel
                void write(int value);
            private:
                volatile timer_register *timer;
                ChannelNumber channel;
        };
    private:
        volatile timer_register *timer;
        // The alternate function number associated with this timer.
        AlternateFunction af;
};

// Location of timer registers in memory. Refer to p.55-56 of the datasheet
static volatile timer_register *const tim1_base = (timer_register *const) 0x40010000;
static volatile timer_register *const tim2_5_base = (timer_register *const) 0x40000000;
static volatile timer_register *const tim9_11_base = (timer_register *const) 0x40014000;

}
