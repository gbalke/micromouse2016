#include "motor.h"
#include "timer_encoder.h"
#include "interrupt_encoder.h"
#include "irsensor.h"
#include "gyro.h"
#include "pid.h"
#include "digital_output.h"
#include "distance_sensor.h"

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
Motor left_motor(lfwd, lbwd);
Motor right_motor(rfwd, rbwd);

TimerEncoder left_encoder(HAL::Timer::TIMER2, PA_1, PA_0);
InterruptEncoder right_encoder(PB_3, PA_15, InterruptEncoder::X4);

DigitalInput sw1(PB_12);
DigitalInput sw2(PB_1);
DigitalInput sw3(PB_0);
DigitalInput sw4(PA_7);

DigitalOutput red(PB_15);
DigitalOutput green(PB_14);
DigitalOutput blue(PB_13);

DistanceSensor left_sensor(PC_3, PB_6, DistanceSensor::LOGISTIC, 1925.5, 0.4, 4073.8, 7.7);
DistanceSensor right_sensor(PC_0, PB_8, DistanceSensor::LOGISTIC, 2298.3, 0.4, 4129.8, 6.5);
DistanceSensor left_side_sensor(PC_2, PB_7, DistanceSensor::EXPONENTIAL, 9879.0, 0.5, 897.0, 0.0);
DistanceSensor right_side_sensor(PC_1, PB_9, DistanceSensor::EXPONENTIAL, 7875.0, 0.4, 1324.6, 0.0);

//Gyro gyro(PC_10, PC_12, PC_11, PA_4, 1e7);

Pid encoder_controller(20, 0.03, 0);
Pid ir_controller(50, 0, 0);

double battery_level();
bool is_side_wall(double reading, double distance);
bool is_front_wall(double left, double right, double distance);

const int CELL_LENGTH = 780;
bool is_green = false;

int main()
{
    motor_timer.set_period(511);
    motor_timer.enable(true);
    int cell_count = 0;
    int encoder_ticks = 0;
    //gyro.calibrate();
    bool ir_pid = true;
    while(true) {
        if(battery_level() < 7.4) {
            red.write(1);
        } else {
            red.write(0);
        }
        if(encoder_ticks + (left_encoder.count() + right_encoder.count()) / 2 >
                CELL_LENGTH * (cell_count + 1)) {
            cell_count++;
            is_green = !is_green;
            green.write(is_green);
        }
        if(!sw1.read()) {
            const int BASE_SPEED = 60;
            const int MAX_SPEED = 200;
            int left_speed = BASE_SPEED;
            int right_speed = BASE_SPEED;
            double left = left_sensor.read();
            double right = right_sensor.read();
            double left_side = left_side_sensor.read();
            double right_side = right_side_sensor.read();
            bool is_left_wall = is_side_wall(left_side, 9.0);
            bool is_right_wall = is_side_wall(right_side, 9.0);
            if(is_front_wall(left, right, 12.0)) {
                left_motor.set_speed(0);
                right_motor.set_speed(0);
                continue;
            }
            if(ir_pid && (!is_left_wall || !is_right_wall)) {
                ir_pid = false;
                encoder_ticks += (left_encoder.count() + right_encoder.count()) / 2;
                left_encoder.reset();
                right_encoder.reset();
                encoder_controller.reset();
            } else if(!ir_pid && is_left_wall && is_right_wall) {
                ir_pid = true;
                ir_controller.reset();
            }
            int correction;
            if(ir_pid) {
                correction = (int)ir_controller.correction(left_side - right_side);
            } else {
                correction = (int)encoder_controller.correction(left_encoder.count() - right_encoder.count());
            }
            if(correction > 0) {
                right_speed += correction;
            } else {
                left_speed += correction;
            }
            left_speed = ((unsigned) left_speed > MAX_SPEED) ? MAX_SPEED : left_speed;
            right_speed = ((unsigned) right_speed > MAX_SPEED) ? MAX_SPEED : right_speed;
            left_motor.set_speed(left_speed);
            right_motor.set_speed(right_speed);
        } else {
            left_motor.set_speed(0);
            right_motor.set_speed(0);
        }
    }
}

double battery_level()
{
    AnalogIn voltage_divider(PC_5);
    return (8.3 / 54700) * voltage_divider.read_u16();
}

bool is_side_wall(double reading, double distance)
{
    return !isnan(reading) && reading < distance;
}

bool is_front_wall(double left, double right, double distance)
{
    return !isnan(left) && !isnan(right) && left < distance && right < distance;
}
