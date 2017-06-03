#include "motor.h"
#include "timer_encoder.h"
#include "interrupt_encoder.h"
#include "gyro.h"
#include "pid.h"
#include "digital_output.h"

#include "Solver.h"

#include "stdlib.h"

#include "mbed.h"
#include "Servo.h"

// Enable Serial Communications
// Uncomment SERIAL_ENABLE to enable hardware serial.
// Uncomment SERIAL_ENABLE and BLUETOOTH to enable bluetooth serial.
#define SERIAL_ENABLE
#define BLUETOOTH
const static float DEBUG_WAIT_TIME = 1;	// Wait 2 seconds between each print loop.

// Serial debug output options.
#define ENC_DEBUG	// Encoders

#if !defined(BLUETOOTH) && defined(SERIAL_ENABLE)
Serial serial(PA_2, PA_3);
#endif
#if defined(BLUETOOTH) && defined(SERIAL_ENABLE)
Serial serial(PA_9, PA_10);
#endif 

// Setting up PWM timer and handlers for the motors.
HAL::Timer motor_timer(HAL::Timer::TIMER3, HAL::Timer::CPU);
HAL::Timer::Channel lbwd(motor_timer, HAL::Timer::CH1, HAL::Timer::Channel::COMPARE_PWM1, PC_6);
HAL::Timer::Channel lfwd(motor_timer, HAL::Timer::CH2, HAL::Timer::Channel::COMPARE_PWM1, PC_7);
HAL::Timer::Channel rbwd(motor_timer, HAL::Timer::CH4, HAL::Timer::Channel::COMPARE_PWM1, PC_9);
HAL::Timer::Channel rfwd(motor_timer, HAL::Timer::CH3, HAL::Timer::Channel::COMPARE_PWM1, PC_8);
Motor left_motor(lfwd, lbwd);
Motor right_motor(rfwd, rbwd);

TimerEncoder left_encoder(HAL::Timer::TIMER2, PA_1, PA_0);
InterruptEncoder right_encoder(PB_3, PA_15, InterruptEncoder::X4);

// DIP switch setup.
DigitalInput kill_switch(PB_12);
DigitalInput sw2(PB_1);
DigitalInput sw3(PB_0);
DigitalInput sw4(PA_7);

// LED colors setup.
DigitalOutput red(PB_15);
DigitalOutput green(PB_14);
DigitalOutput blue(PB_13);

Servo scythe(PB_8);

// PID object setup.

// Different modes supported by the mouse. Setting mode to one of these will cause the
// corresponding action
enum Mode {
    IDLE = 0,
    FORWARD = 3,
    LEFT = -1,
    RIGHT = 1,
    BACKWARD = 2,
    STOP = 4,
};

const static float THRESHOLD_VOLTAGE = 7.4; // low voltage threshold

double battery_level();
void forward();
void backward();
bool stop();
void brake();
bool set_pos(int left_pos, int right_pos);
bool turn(Mode direction);
Mode manualDrive(Mode);

// temporary cell counter; eventually we need an (x, y) coordinate
// (also this cell counter doesn't work fully at the moment)
int cell_count = 0;

int main()
{
	// Setting up PWM for motors.
    motor_timer.set_period(511);
    motor_timer.enable(true);

    Mode mode = IDLE;

	// NOTE: This event loop must run very fast, so do not put any long-running
    // or blocking functions in this loop. Instead, break those functions into
    // several iterations, like init(), forward(), etc are.
    while(true) {

        if(battery_level() < THRESHOLD_VOLTAGE) {
            red.write(1);
        } else {
            red.write(0);
        }

        if(kill_switch.read()) {
            brake();
#ifdef SERIAL_ENABLE
#ifdef ENC_DEBUG
            serial.printf("l %d\r\n", left_encoder.count());
            serial.printf("r %d\r\n", right_encoder.count());
#endif
			wait(DEBUG_WAIT_TIME);
#endif
            continue;
        }
	
		mode = manualDrive(mode);

        // FSM for controlling mouse movement
        switch(mode) {
            case IDLE:
               	brake(); 
                break;
            case FORWARD:
                forward();
                break;
            case LEFT:
            case RIGHT:
                if(turn(mode))
					mode = IDLE;				
				break;
			case BACKWARD:
				backward();
			    break;	
            case STOP:
                stop();
                break;
        }

		// Delay to buffer input.
		//wait(0.1);

    }
}

// Performs one iteration of the forward moving PID.
// Returns the next mode that the mouse should enter.
void forward()
{
    static Pid encoder_controller(20, 0.0, 5);
   	const int BASE_SPEED = 100;
    const int MAX_SPEED = 250;

    int left_speed = BASE_SPEED;
    int right_speed = BASE_SPEED;
    int correction;
   
   	correction = (int)encoder_controller.correction(left_encoder.count() - right_encoder.count());
    
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


// Drives backwards half a cell (used for initial IR setup and aligning on walls after reaching dead end).
void backward()
{
	static Pid backward_controller(20,0.0,5);
    const int BASE_SPEED = 20;
    const int MAX_SPEED = 100;

    int left_speed = BASE_SPEED;
    int right_speed = BASE_SPEED;
    int correction;
    correction = (int)backward_controller.correction(std::abs(left_encoder.count()) - std::abs(right_encoder.count()));
    
    if(correction > 0) {
        right_speed += correction;
    } else {
        left_speed += correction;
    }
    left_speed = ((unsigned) left_speed > MAX_SPEED) ? MAX_SPEED : left_speed;
    right_speed = ((unsigned) right_speed > MAX_SPEED) ? MAX_SPEED : right_speed;
    left_motor.set_speed(-left_speed);
    right_motor.set_speed(-right_speed);
}

// Disables the motors. Only use if you absolutely need the motors to stop immediately:
// normally stop() should be used to achieve a smooth and controlled stop.
void brake()
{
    left_motor.brake();
    right_motor.brake();
}

// Performs one iteration of a PID stop. Must be repeated until the stop is complete.
// Returns true if the stop is complete.
bool stop()
{
    static bool initialized = false;
    static int stop_point;

    if(!initialized) {
        stop_point = (left_encoder.count() + right_encoder.count()) / 2;
        initialized = true;
    }
    bool ret = set_pos(stop_point, stop_point);
    if(ret) {
        initialized = false;
    }
    return ret;
}

// Performs one iteration of a PID turn. Must be repeated until the turn is complete.
// LEFT → 90° to the left
// RIGHT → 90° to the right
// FLIP → 180° turn to the right
// All other parameters are undefined (do not use)
// Returns true if the turn is complete
bool turn(Mode direction)
{
    static bool initialized = false;
    // each wheel must turn this much (in opposite directions) to make a 90° turn
    const int ENCODER_COUNT = 285;

    if(!initialized) {
        // Turns are smoother if we stop first.
        initialized = stop();
        if(initialized) {
            left_encoder.reset();
            right_encoder.reset();
        }
        return false;
    }
    bool ret = set_pos(direction * ENCODER_COUNT, -1 * direction * ENCODER_COUNT);
    if(ret) {
        initialized = false;
    }
    return ret;
}

// Performs one iteration of the position-setting PID loop. Should normally be called though a
// wrapper function such as stop() or turn(). Do not use for main movement.
// Returns true if the PID is complete.
bool set_pos(int left_pos, int right_pos)
{
    // be careful about burning out motors
    const int MAX_SPEED = 200;
    static Pid left_position_controller(15, 0, 125); //20,0,80
    static Pid right_position_controller(15, 0, 125);
    static int stable_count = 0;

    int left_speed, right_speed;
    int left_error = left_pos - left_encoder.count();
    int right_error = right_pos - right_encoder.count();
    int left_correction = (int)left_position_controller.correction(left_error);
    int right_correction = (int)right_position_controller.correction(right_error);
    right_speed = right_correction;
    left_speed = left_correction;
    right_speed = (right_speed > MAX_SPEED) ? MAX_SPEED : right_speed;
    right_speed = (right_speed < -1 * MAX_SPEED) ? -1 * MAX_SPEED : right_speed;
    left_speed = (left_speed > MAX_SPEED) ? MAX_SPEED : left_speed;
    left_speed = (left_speed < -1 * MAX_SPEED) ? -1 * MAX_SPEED : left_speed;
    left_motor.set_speed(left_speed);
    right_motor.set_speed(right_speed);
    if(left_correction == 0 && right_correction == 0) {
        stable_count++;
    }
    if(stable_count == 500) {
        left_position_controller.reset();
        right_position_controller.reset();
        stable_count = 0;
        return true;
    }
    return false;
}

// Returns the battery voltage level in Volts
double battery_level()
{
    AnalogIn voltage_divider(PC_5);
    // value found experimentally
    return (8.3 / 54700) * voltage_divider.read_u16();
}

Mode manualDrive (Mode mode)
{
	if (serial.readable()) {
		char in = serial.getc();
		serial.printf("%c",in);

		// Flush serial buffer to make sure it doesn't overflow.
		while(serial.readable())
			serial.getc();

		// Reset encoder counts.
		left_encoder.reset();
        right_encoder.reset();

		switch (in)
		{
			case 'w':
				return FORWARD;
			case 'a':
				return LEFT;
			case 'd':
				return RIGHT;
			case 's':
				return BACKWARD;
			case 'e':
				return IDLE;
            case 'r':
                scythe.write(0);
                return mode;
            case 'f':
                scythe.write(0.9);
                return mode;
		}
	}

	return mode;
}
