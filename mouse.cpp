#include "motor.h"
#include "timer_encoder.h"
#include "interrupt_encoder.h"
#include "irsensor.h"
#include "gyro.h"
#include "pid.h"
#include "digital_output.h"
#include "distance_sensor.h"

#include "stdlib.h"

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

DistanceSensor left_sensor(PC_3, PB_6);
DistanceSensor right_sensor(PC_0, PB_8);
DistanceSensor left_side_sensor(PC_2, PB_7);
DistanceSensor right_side_sensor(PC_1, PB_9);

Pid combined_controller(20, 0.0, 5);
Pid ir_controller(800, 0, 100);
Pid turn_controller(1, 0, 0.1);

enum Direction {
    FORWARD = 0,
    LEFT = -1,
    RIGHT = 1,
};

double battery_level();
bool is_side_wall(double reading, double distance);
bool is_front_wall(double left, double right, double distance);
void forward(bool is_left_wall, bool is_right_wall, double left_side, double right_side);
void stop();
bool turn(Direction direction);

const int CELL_LENGTH = 795;
bool is_green = true;

int main()
{
    motor_timer.set_period(511);
    motor_timer.enable(true);
    srand(left_side_sensor.raw_read() % 16);
    int cell_count = 0;
    Direction mode = FORWARD;
    bool ir_pid = true;
    int turn_counter = 0;
    int stop_counter = 0;
    bool left_opening = false;
    bool right_opening = false;
    green.write(is_green);
    left_side_sensor.calibrate();
    right_side_sensor.calibrate();
    while(true) {
        if(battery_level() < 7.4) {
            red.write(1);
        } else {
            red.write(0);
        }
        if(!sw1.read()) {
            double left = left_sensor.read();
            double right = right_sensor.read();
            double left_side = left_side_sensor.read();
            double right_side = right_side_sensor.read();
            bool is_left_wall = is_side_wall(left_side, 7.8);
            bool is_right_wall = is_side_wall(right_side, 9.5);
            left_opening |= !is_left_wall;
            right_opening |= !is_right_wall;
            blue.write(ir_pid);
            int pos = (left_encoder.count() + right_encoder.count()) / 2;
            if(mode == FORWARD && pos > (CELL_LENGTH * (cell_count + 1))) {
                cell_count++;
                is_green = !is_green;
                green.write(is_green);
                if(sw2.read() || (sw3.read() && (rand() % 2))) {
                    bool only_one = left_opening ^ right_opening;
                    if(!sw4.read() || only_one) {
                        if(left_opening) {
                            stop();
                            mode = LEFT;
                        } else if(right_opening) {
                            stop();
                            mode = RIGHT;
                        }
                    }
                }
                left_opening = false;
                right_opening = false;
            }
            if(mode == FORWARD && is_front_wall(left, right, 5.0)) {
                if(left_opening) {
                    stop();
                    mode = LEFT;
                } else if(right_opening) {
                    stop();
                    mode = RIGHT;
                } else if(rand() % 2) {
                    stop();
                    mode = LEFT;
                } else {
                    stop();
                    mode = RIGHT;
                }
            }
            if(mode == FORWARD && ir_pid && (!is_left_wall || !is_right_wall)) {
                ir_pid = false;
            } else if(mode == FORWARD && !ir_pid && (is_left_wall && is_right_wall)) {
                ir_pid = true;
            }
            switch(mode) {
                case FORWARD:
                    forward(is_left_wall, is_right_wall, left_side, right_side);
                    break;
                case LEFT:
                case RIGHT:
                    if(stop_counter < 1000) {
                        stop_counter++;
                    } else if(turn(mode)) {
                        turn_counter++;
                    }
                    if(turn_counter == 1000) {
                        mode = FORWARD;
                        stop_counter = 0;
                        turn_counter = 0;
                        left_opening = false;
                        right_opening = false;
                        cell_count = 0;
                        left_encoder.reset();
                        right_encoder.reset();
                        turn_controller.reset();
                        combined_controller.reset();
                    }
                    break;
            }
        } else {
            stop();
            serial.printf("l %f\r\n", (left_side_sensor.read()));
            serial.printf("r %f\r\n", (right_side_sensor.read()));
            wait(1);
        }
    }
}

void stop()
{
    left_motor.brake();
    right_motor.brake();
}

void forward(bool is_left_wall, bool is_right_wall, double left_side, double right_side)
{
    const int BASE_SPEED = 20;
    const int MAX_SPEED = 100;
    const int LEFT_SETPOINT = 3;
    const int RIGHT_SETPOINT = 3;
    int left_speed = BASE_SPEED;
    int right_speed = BASE_SPEED;
    int correction;
    if(is_left_wall && is_right_wall) {
        //correction = (int)combined_controller.correction(100*(left_side-right_side)+left_encoder.count()-right_encoder.count());
        correction = (int)ir_controller.correction(is_left_wall*(left_side - LEFT_SETPOINT) -
                                                    is_right_wall*(right_side - RIGHT_SETPOINT));
    } else {
        correction = (int)combined_controller.correction(left_encoder.count() - right_encoder.count());
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
}

bool turn(Direction direction)
{
    const int MAX_SPEED = 30;
    int left_speed, right_speed;
    double error = left_encoder.count() - right_encoder.count() - direction * 555;
    int correction = (int)turn_controller.correction(error);
    right_speed = correction;
    right_speed = (right_speed > MAX_SPEED) ? MAX_SPEED : right_speed;
    right_speed = (right_speed < -1 * MAX_SPEED) ? -1 * MAX_SPEED : right_speed;
    left_speed = -1 * right_speed;
    left_motor.set_speed(left_speed);
    right_motor.set_speed(right_speed);
    return turn_controller.get_derivative() == 0;
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
