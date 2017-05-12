#include "motor.h"
#include "timer_encoder.h"
#include "interrupt_encoder.h"
#include "irsensor.h"
#include "gyro.h"

#include "mbed.h"

Serial serial(PA_2, PA_3);

HAL::Timer motor_timer(HAL::Timer::TIMER3, HAL::Timer::CPU);
HAL::Timer::Channel lfwd(motor_timer, HAL::Timer::CH1, HAL::Timer::Channel::COMPARE_PWM1, PC_6);
HAL::Timer::Channel lbwd(motor_timer, HAL::Timer::CH2, HAL::Timer::Channel::COMPARE_PWM1, PC_7);
HAL::Timer::Channel rfwd(motor_timer, HAL::Timer::CH4, HAL::Timer::Channel::COMPARE_PWM1, PC_9);
HAL::Timer::Channel rbwd(motor_timer, HAL::Timer::CH3, HAL::Timer::Channel::COMPARE_PWM1, PC_8);
Motor left(lfwd, lbwd);
Motor right(rfwd, rbwd);

TimerEncoder left_encoder(HAL::Timer::TIMER2, PA_1, PA_0);

DigitalInput sw1(PB_12);
DigitalInput sw2(PB_1);
DigitalInput sw3(PB_0);
DigitalInput sw4(PA_7);
DigitalOutput red(PB_15);
DigitalOutput green(PB_14);
DigitalOutput blue(PB_13);

Gyro gyro(PC_10, PC_12, PC_11, PA_4, 1e7);

int main()
{
    motor_timer.set_period(255);
    motor_timer.enable(true);
    gyro.calibrate();
    while(true) {
        if(!sw1.read()) {
            left.set_speed(10);
            right.set_speed(10);
        } else {
            left.set_speed(0);
            right.set_speed(0);
        }
        serial.printf("Encoder: %d\r\n", left_encoder.count());
    }
}
