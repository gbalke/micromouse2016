#include <cmath>
#include "mbed.h"
#include "motor.h"
#include "encoder.h"
#include "irsensor.h"

Serial serial(PA_9, PA_10);

Motor left_motor(PB_6, PA_7, 3);
Motor right_motor(PC_7, PB_10);

Encoder left_encoder(PA_1, PC_4, Encoder::X4, sqrt(3));
Encoder right_encoder(PB_3, PA_15, Encoder::X4);

// IR Sensor Objects/Consts
Ticker interrupts;

IRSensor left_irsensor(PC_0, PB_7);
IRSensor front_left_irsensor(PC_1, PB_0);
IRSensor front_right_irsensor(PA_4, PC_11);
IRSensor right_irsensor(PA_0, PC_10);

const float IR_READ_DELAY = 0.5;
// End IR Sensor Stuff


void forward(int);
void right_turn(int);
void ir_update();

int main()
{
	
	interrupts.attach(&ir_update, IR_READ_DELAY);

    while(true) {
        serial.printf("Left: %d\r\n", left_encoder.count());
        serial.printf("Right: %d\r\n", right_encoder.count());
		serial.printf("Left IRSensor: %d\r\n", left_irsensor.getValue());
		serial.printf("Front Left IRSensor: %d\r\n", front_left_irsensor.getValue());
		serial.printf("Front Right IRSensor: %d\r\n", front_right_irsensor.getValue());
		serial.printf("Right IRSensor: %d\r\n", right_irsensor.getValue());
		wait (0.5);
    }
}

void forward(int cells)
{
    const float P = 0;//0.001;
    while(left_encoder.count() < 500 * cells) {
        int error = left_encoder.count() - right_encoder.count();
        float correction = P * error;
        left_motor.set_speed(0.1 - correction);
        right_motor.set_speed(0.1 + correction);
        wait(0.1);
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
	serial.printf("firing");
	left_irsensor.read();
	front_left_irsensor.read();
	front_right_irsensor.read();
	right_irsensor.read();
}
