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

// Enable Serial Communications
// Uncomment SERIAL_ENABLE to enable hardware serial.
// Uncomment SERIAL_ENABLE and BLUETOOTH to enable bluetooth serial.
//#define SERIAL_ENABLE
//#define BLUETOOTH
const static float DEBUG_WAIT_TIME = 1;	// Wait 2 seconds between each print loop.

// Serial debug output options.
//#define IR_DEBUG	// IR Sensors 
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

// IR sensor setup.
DistanceSensor left_sensor(PC_3, PB_6);
DistanceSensor right_sensor(PC_0, PB_8);
DistanceSensor left_side_sensor(PC_2, PB_7);
DistanceSensor right_side_sensor(PC_1, PB_9);

// PID object setup.
Pid combined_controller(20, 0.03, 5);
Pid ir_controller(800, 0, 100);
Pid left_turn_controller(10, 0, 50);
Pid right_turn_controller(10, 0, 50);
Pid backward_controller(20,0.0,5);

// Different modes supported by the mouse. Setting mode to one of these will cause the
// corresponding action
enum Direction {
    FORWARD = 0,
    LEFT = -1,
    RIGHT = 1,
    FLIP = 2,
	BACKWARD,
	// don't use STOP with turn()
    STOP,

};

// Const defines.
const static float THRESHOLD_VOLTAGE = 7.4; // If the voltage reads below this value then the red LED will turn on.
const int CELL_LENGTH = 806; // Number of encoder counts in one cell.
const static float LEFT_WALL_DIST = 1.5;
const static float RIGHT_WALL_DIST = 1.5;
const static float FRONT_WALL_DIST = 1;

double battery_level();
bool is_side_wall(double reading, double distance);
bool is_front_wall(double left, double right, double distance);
void forward(bool is_left_wall, bool is_right_wall, double left_side, double right_side);
void backward();
bool stop();
void brake();
bool set_pos(int left_pos, int right_pos);
bool turn(Direction direction);

int main()
{
	// Bool for when green LED is high.
	bool is_green = true;
	// Setting up PWM for motors.
    motor_timer.set_period(511);
    motor_timer.enable(true);
	// Seeding random generator.
    srand(left_side_sensor.raw_read() % 16);
	// Keeps track of the number of cells we've traveled through.
    int cell_count = 0;
	// Will be true when there are walls on both sides to be used for PID.
    bool ir_pid = true;
	// Bools keeping track of whether or not there is an opening to the left and right of the mouse.
    bool left_opening = false;
    bool right_opening = false;
	// Indicates that the mouse is on.
    green.write(is_green);

	// Declare Sensor and wall variables.
	double left; 
    double right;
    double left_side;
    double right_side; 
	bool is_left_wall; 
    bool is_right_wall;

	// setup runs on the first loop of the drive code.
	bool setup = true;

	// Calibrating IR on boot.
	// The mouse should be facing towards the back wall of start (will turn around on its own). 
    left_side_sensor.calibrate();
    right_side_sensor.calibrate();

	
	// Set first turn operation (turn around after calibration).
    Direction mode = FLIP;

    while(true) {

		// Check if battery level is below threshhold voltage.
        if(battery_level() < THRESHOLD_VOLTAGE) {
            red.write(1);
        } else {
            red.write(0);
        }
		
		// Disable mouse until switch one is on.
        if(!kill_switch.read()) 
		{
			if (setup == true)
			{
				// Drive half a cell backwards and calibrate.
				while(true) {
					int pos = std::abs((left_encoder.count() + right_encoder.count()) / 2);	
					backward();
			
					if(pos > (CELL_LENGTH/2)) {
						stop();
						break;
					}
				}
				wait(1); // Wait a quarter second to stop moving
				// Calibrate side IRs.
				left_sensor.calibrate();
			    right_sensor.calibrate();
				// Drive the other half of the cell backwards.
				while(true) {
					int pos = std::abs((left_encoder.count() + right_encoder.count()) / 2);
					
					backward();
			
					if(pos > (CELL_LENGTH)) {
						cell_count++;
						stop();
						setup = false;
						break;
				    }
				}

				
			}
			else {
				// Read IR sensor values.
    	        left = left_sensor.read();
    	        right = right_sensor.read();
    	        left_side = left_side_sensor.read();
    	        right_side = right_side_sensor.read();
				// Calculate whether or not there are walls on either side.
    	        is_left_wall = is_side_wall(left_side, LEFT_WALL_DIST);
    	        is_right_wall = is_side_wall(right_side, RIGHT_WALL_DIST);
				// If there is no wall read set to open and maintain. If there is a wall, maintain current state.
				// Must use further verification to make sure there is no opening.
    	        left_opening |= !is_left_wall;
    	        right_opening |= !is_right_wall;

				// Write blue high while using IR for PID.
    	        blue.write(ir_pid);
				// Determining which direction to drive next.
    	        int pos = (left_encoder.count() + right_encoder.count()) / 2;

				// Handling if current state is forward.
				if (mode == FORWARD)
				{
    	       		 if(pos > (CELL_LENGTH * (cell_count + 1))) {
    	       		     cell_count++;
    	       		     is_green = !is_green;
    	       		     green.write(is_green);
			   		 	/*
    	       		     if(sw2.read() || (sw3.read() && (rand() % 2))) {
    	       		         bool only_one = left_opening ^ right_opening;
    	       		         if(!sw4.read() || only_one) {
    	       		             if(left_opening) {
    	       		                 mode = LEFT;
    	       		             } else if(right_opening) {
    	       		                 mode = RIGHT;
    	       		             }
    	       		         }
    	       		     }
			   		 	*/
    	       		     left_opening = false;
    	       		     right_opening = false;
    	       		 }
    	       		 if(is_front_wall(left, right, FRONT_WALL_DIST)) {
    	       		     if(left_opening) {
    	       		         mode = LEFT;
    	       		     } else if(right_opening) {
    	       		         mode = RIGHT;
    	       		     } else {
    	       		         mode = FLIP;
    	       		     }
    	       		 }
    	       		 if(ir_pid && (!is_left_wall || !is_right_wall)) {
    	       		     ir_pid = false;
    	       		 } else if(!ir_pid && (is_left_wall && is_right_wall)) {
    	       		     ir_pid = true;
    	       		 }
				}
				// Handling if current state is backward.
				else if (mode == BACKWARD)
				{
					
				}

				// Switch to handle next movement (actual drive code).
    	        switch(mode) {
    	            case FORWARD:
    	                forward(is_left_wall, is_right_wall, left_side, right_side);
    	                break;
    	            case LEFT:
    	            case RIGHT:
    	            case FLIP:
	                    if(turn(mode)) {
	                        mode = FORWARD;
	                        left_opening = false;
	                        right_opening = false;
	                        cell_count = 0;
	                        left_encoder.reset();
	                        right_encoder.reset();
	                        ir_controller.reset();
	                        combined_controller.reset();
	                    }
	                    break;
					case STOP:
                    	stop();
                    	break;
					case BACKWARD:
						backward();
						break;
    	        }
			}

        } else {
            brake();
#ifdef SERIAL_ENABLE
#ifdef ENC_DEBUG
            serial.printf("l %d\r\n", left_encoder.count());
            serial.printf("r %d\r\n", right_encoder.count());
#endif
#ifdef IR_DEBUG
			serial.printf("l %f\r\n", (left_sensor.read()));
            serial.printf("r %f\r\n", (right_sensor.read()));
            serial.printf("ls %f\r\n", (left_side_sensor.read()));
            serial.printf("rs %f\r\n", (right_side_sensor.read()));
#endif
			wait(DEBUG_WAIT_TIME);
#endif
        }
    }
}

// Performs one iteration of the forward moving PID.
// is_left_wall: is there a visible wall to the left of the mouse
// is_right_wall: is there a visible wall to the right of the mouse
// left_side: IR reading from the left_side (unused if !is_left_wall)
// right_side: IR reading from the right_side (unused if !is_right_wall)
void forward(bool is_left_wall, bool is_right_wall, double left_side, double right_side)
{
    const int BASE_SPEED = 20;
    const int MAX_SPEED = 100;
    int left_speed = BASE_SPEED;
    int right_speed = BASE_SPEED;
    int correction;
    if(is_left_wall && is_right_wall) {
        correction = (int)ir_controller.correction(left_side - right_side);
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


// Drives backwards half a cell (used for initial IR setup and aligning on walls after reaching dead end).
void backward()
{
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
// Returns true if the turn is complete
bool turn(Direction direction)
{
    // each wheel must turn this much (in opposite directions) to make a 90° turn
    const int ENCODER_COUNT = 289;
    static bool initialized = false;
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
    static Pid left_position_controller(20, 0, 80);
    static Pid right_position_controller(20, 0, 80);
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
    bool ret = left_correction == 0 && right_correction == 0;
    if(ret) {
        left_position_controller.reset();
        right_position_controller.reset();
    }
    return ret;
}

// Returns the battery voltage level in Volts
double battery_level()
{
    AnalogIn voltage_divider(PC_5);
    // value found experimentally
    return (8.3 / 54700) * voltage_divider.read_u16();
}

bool is_side_wall(double reading, double distance)
{
    return reading < distance;
}

bool is_front_wall(double left, double right, double distance)
{
    return left < distance && right < distance;
}
