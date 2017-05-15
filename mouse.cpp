#include "motor.h"
#include "timer_encoder.h"
#include "interrupt_encoder.h"
#include "irsensor.h"
#include "gyro.h"
#include "pid.h"
#include "digital_output.h"

#include "mbed.h"

#ifndef BTSERIAL
Serial serial(PA_2, PA_3);
#endif
#ifdef BTSERIAL
Serial serial(PA_9, PA_10);
#endif 

HAL::Timer motor_timer(HAL::Timer::TIMER3, HAL::Timer::CPU);
HAL::Timer::Channel lbwd(motor_timer, HAL::Timer::CH1, HAL::Timer::Channel::COMPARE_PWM1, PC_6);
HAL::Timer::Channel lfwd(motor_timer, HAL::Timer::CH2, HAL::Timer::Channel::COMPARE_PWM1, PC_7);
HAL::Timer::Channel rbwd(motor_timer, HAL::Timer::CH4, HAL::Timer::Channel::COMPARE_PWM1, PC_9);
HAL::Timer::Channel rfwd(motor_timer, HAL::Timer::CH3, HAL::Timer::Channel::COMPARE_PWM1, PC_8);
Motor left(lfwd, lbwd);
Motor right(rfwd, rbwd);

TimerEncoder left_encoder(HAL::Timer::TIMER2, PA_1, PA_0);
InterruptEncoder right_encoder(PB_3, PA_15, InterruptEncoder::X4);

DigitalInput sw1(PB_12);
DigitalInput sw2(PB_1);
DigitalInput sw3(PB_0);
DigitalInput sw4(PA_7);

DigitalOutput red(PB_15);
DigitalOutput green(PB_14);
DigitalOutput blue(PB_13);

IRSensor left_irsensor(PC_3, PB_6);
IRSensor right_irsensor(PC_0, PB_8);
IRSensor front_left_irsensor(PC_2, PB_7);
IRSensor front_right_irsensor(PC_1, PB_9);

//Gyro gyro(PC_10, PC_12, PC_11, PA_4, 1e7);

Pid controller(0, 0, 0);

float battery_level();

float ir_distance(float reading, float scale, float time_constant, float midpoint, float max);

int main()
{
    motor_timer.set_period(511);
    motor_timer.enable(true);
    //gyro.calibrate();
    while(true) {
        if(battery_level() < 7.4) {
            red.write(1);
        } else {
            red.write(0);
        }
        if(!sw1.read()) {
            const int BASE_SPEED = 20;
            int left_speed = BASE_SPEED;
            int right_speed = BASE_SPEED;
            int correction = (int)controller.correction(left_encoder.count() - right_encoder.count());
            if(correction > 0) {
                left_speed -= correction;
                left_speed = (left_speed < 0) ? 0 : left_speed;
            } else {
                right_speed -= correction;
                right_speed = (right_speed < 0) ? 0 : right_speed;
            }
            left.set_speed(left_speed);
            right.set_speed(right_speed);
        } else {
            left.set_speed(0);
            right.set_speed(0);
        }
        int left = left_irsensor.read();
        int right = right_irsensor.read();
        serial.printf("l %f\r\n", ir_distance(left, -1925.5, 0.4, 7.7, 4073.8));
        serial.printf("r %f\r\n", ir_distance(right, -2298.3, 0.4, 6.5, 4129.8));
        serial.printf("fl %d\r\n", front_left_irsensor.read());
        serial.printf("fr %d\r\n", front_right_irsensor.read());
        wait(1);
    }
}

float battery_level()
{
    AnalogIn voltage_divider(PC_5);
    return (8.3 / 54700) * voltage_divider.read_u16();
}

float ir_distance(float reading, float scale, float time_constant, float midpoint, float max)
{
    return midpoint - log(scale / (reading - max) - 1) / time_constant;
}
