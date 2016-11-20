#include <cmath>
#include "mbed.h"
#include "motor.h"
#include "encoder.h"
#include "irsensor.h"

Serial serial(PA_9, PA_10);

Motor left_motor(PB_6, PA_7, 3);
Motor right_motor(PC_7, PB_10);

Encoder left_encoder(PA_1, PC_4, Encoder::X2, 1.72);
Encoder right_encoder(PA_15, PB_3, Encoder::X0);

Ticker interrupts;

IRSensor left_irsensor(PC_0, PB_7);
IRSensor front_left_irsensor(PC_1, PB_0);
IRSensor front_right_irsensor(PA_4, PC_11);
IRSensor right_irsensor(PA_0, PC_10);

const float IR_READ_DELAY = 0.1;

void forward(int);
void right_turn(int);
void ir_update();

int main()
{
	interrupts.attach(&ir_update, IR_READ_DELAY);
    forward(100000);
}

void forward(int cells)
{
    const float P = 0.002;
    const float I = 0;
    const float D = 0;
    const int CELL_LENGTH = 500;
    const float SPEED = 0.1;
    const float DELAY = 0.005;
    const int LEFT_STOP = 950;
    const int RIGHT_STOP = 1140;
    // Divisor is to ignore small errors between left & right, multiplier is for error weighting
    const int IR_DIVISOR = 100;
    const int IR_MULTIPLIER = 2;

    int prev_error = 0;
    int integral = 0;
    left_encoder.reset();
    right_encoder.reset();
    while(left_encoder.count() < CELL_LENGTH * cells) {
        if (front_left_irsensor.getValue() >= LEFT_STOP ||
                front_right_irsensor.getValue() >= RIGHT_STOP) {
            left_motor.set_speed(0);
            right_motor.set_speed(0);
            continue;
        }
        int error = left_encoder.count() - right_encoder.count();
        error += IR_MULTIPLIER * 
            ((right_irsensor.getValue() - left_irsensor.getValue()) / IR_DIVISOR);
        integral += error;
        float correction = P * error + I * integral + D * (error - prev_error);
        if(correction > SPEED) {
            correction = SPEED;
        } else if(correction < -1 * SPEED) {
            correction = -1 * SPEED;
        }
        prev_error = error;
        left_motor.set_speed(SPEED - correction);
        right_motor.set_speed(SPEED + correction);
        wait(DELAY);
    }
    left_motor.set_speed(0);
    right_motor.set_speed(0);
    wait(0.1);
}

void right_turn(int times)
{
    int dir = (times > 0) ? 1 : -1;
    left_encoder.reset();
    right_encoder.reset();
    left_motor.set_speed(dir * 0.05);
    right_motor.set_speed(dir * -0.05);
    while(abs(left_encoder.count() - right_encoder.count()) < 200 * abs(times));
    left_motor.set_speed(0);
    right_motor.set_speed(0);
    wait(0.5);
}

void ir_update()
{
	left_irsensor.read();
	front_left_irsensor.read();
	front_right_irsensor.read();
	right_irsensor.read();
}
