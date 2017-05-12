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
//TimerEncoder right_encoder(HAL::Timer::TIMER5, PA_15, PB_3);
InterruptEncoder right_encoder(PB_3, PA_15, InterruptEncoder::X4);

DigitalInput sw1(PB_12);
DigitalInput sw2(PB_1);
DigitalInput sw3(PB_0);
DigitalInput sw4(PA_7);
DigitalOutput red(PB_15);
DigitalOutput green(PB_14);
DigitalOutput blue(PB_13);

Gyro gyro(PC_10, PC_12, PC_11, PA_4, 1e7);

float battery_level()
{
    AnalogIn voltage_divider(PC_5);
    return (8.3 / 54700) * voltage_divider.read_u16();
}

int main()
{
    motor_timer.set_period(255);
    motor_timer.enable(true);
    gyro.calibrate();
    while(true) {
        if(battery_level() < 7.4) {
            red.write(1);
        } else {
            red.write(0);
        }
        if(!sw1.read()) {
            left.set_speed(30);
            right.set_speed(30);
        } else {
            left.set_speed(0);
            right.set_speed(0);
        }
    }
}
