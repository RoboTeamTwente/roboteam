/*
 * control_util.h
 *
 *  Created on: Nov 14, 2018
 *      Author: kjhertenberg
 */

/* Description:
 *
 * Instructions:
 * 1)
 *
 * Extra functions:
 *
 * Notes:
 *
*/

#ifndef UTILS_CONTROL_UTIL_H_
#define UTILS_CONTROL_UTIL_H_

#include <math.h>
#include <stdbool.h>

///////////////////////////////////////////////////// DEFINITIONS

// System
#define TIME_DIFF 0.01F 		// time difference due to 100Hz frequency
#define WIRELESS_RX_COUNT 4000  // count after which wireless should go to timeout after last packet. Multiply with period base (62.5 us) to get to the time in seconds.

// Robot
#define rad_robot 0.081F	// robot radius (m) (from center to wheel contact point)
#define rad_wheel 0.028F 	// wheel radius (m)

#define FRONT_ANGLE 30		// angle of front wheels (deg)
#define BACK_ANGLE 60		// angle of back wheels (deg)
#define cosFront cos(FRONT_ANGLE * M_PI/180)
#define sinFront sin(FRONT_ANGLE * M_PI/180)
#define cosBack cos(BACK_ANGLE * M_PI/180)
#define sinBack sin(BACK_ANGLE * M_PI/180)

// Wheels
#define PWM_CUTOFF 200.0F 		// arbitrary threshold to avoid motor shutdown
#define WHEEL_GEAR_RATIO 2.65F 	// gear ratio between motor and wheel
#define PWM_LIMIT MAX_PWM 		// should be equal to MAX_PWM by default
float MAX_VOLTAGE; 				// [V] see datasheet
#define MAX_VOLTAGE_30W 12.0
#define MAX_VOLTAGE_50W 24.0
float SPEED_CONSTANT; 			//[(rad/s)/V] see datasheet
#define SPEED_CONSTANT_30W 374.0
#define SPEED_CONSTANT_50W 285.0
#define WHEEL_PULSES_PER_ROTATION (float)4*1024 // number of pulses of the encoder per rotation of the motor (see datasheet)

float OMEGAtoPWM; // conversion factor from wheel speed [rad/s] to required PWM on the motor
#define WHEEL_ENCODER_TO_OMEGA (float)2*M_PI/(TIME_DIFF*WHEEL_GEAR_RATIO*WHEEL_PULSES_PER_ROTATION) // conversion factor from number of encoder pulses to wheel speed [rad/s]

// Dribbler
// TODO: Update these constants once the new dribblers and their encoders are installed.
#define DRIBBLER_GEAR_RATIO 0.777777778F // gear ratio between motor and dribbler
#define DRIBBLER_PULSES_PER_ROTATION 4.0 // number of pulses of the encoder per rotation of the motor (see datasheet)4
#define DRIBBLER_TIME_DIFF 0.1
#define DRIBBLER_ENCODER_TO_OMEGA ((float)2*M_PI/(DRIBBLER_TIME_DIFF*DRIBBLER_GEAR_RATIO*DRIBBLER_PULSES_PER_ROTATION)) // conversion factor from number of encoder pulses to dribbler speed [rad/s]

// Control
#define YAW_MARGIN (0.5F/180.0F)*(float)M_PI 	// margin at which the I-value of the PID is reset to 0
float WHEEL_REF_LIMIT; 							// [rad/s] Limit the maximum wheel reference to leave room for the wheels PID
#define WHEEL_REF_LIMIT_PWM 2200 				// [pwm]

// Shoot
#define MIN_KICK_TIME 1.0 				// minimum time [ms] period of kicking
#define MAX_KICK_TIME 25.0 				// maximum time [ms] period of kicking
#define MIN_CHIP_TIME 10 				// minimum time [ms] period of chipping
#define MAX_CHIP_TIME 100 				// maximum time [ms] period of chipping
#define TIMER_FREQ 10000 			// frequency [Hz] of TIM6  (Clock frequency divided by prescaler)
#define READY_CALLBACK_FREQ 1 		// frequency [Hz] of callback when shootState is Ready
#define CHARGING_CALLBACK_FREQ 10 	// frequency [Hz] of callback when shootState is Charging
#define SHOOTING_CALLBACK_FREQ 10 	// frequency [Hz] of callback when shootState is Shooting
#define OFF_CALLBACK_FREQ 1 		// frequency [Hz] of callback when shootState is Off

///////////////////////////////////////////////////// STRUCTS

typedef enum {
	body_x,		// The velocity in the X direction
	body_y,		// The velocity in the Y direction
	body_w,		// The angular velocity
	body_yaw,	// The angle
}body_handles;

typedef enum {
	wheels_RF,	// The right front wheel
	wheels_RB,	// The right back wheel
	wheels_LB,	// The left back wheel
	wheels_LF,	// The left front wheel
}wheel_names;

typedef enum {
	off,		// The PID controller is inactive
	setup,		// Not used at this moment
	on,			// The PID controller is active
	turning,	// Not used at this moment
	idle		// Not used at this moment
}PID_states;// keeps track of the state of the system

struct PIDstruct{
	float kP;			// The gain of the proportional action 
	float kI;			// The gain of the integrating action
	float kD;			// The gain fo the deriving action
	float I;			// The integral of this PID
	float prev_e;		// The previous error
	float timeDiff;		// The time difference due to 100Hz frequency
	float minOutput;	// Not being used at this moment. The minimal output of this PID
	float maxOutput;	// Not being used at this moment. The maximal output of this PID
	float ramp;			// Not being used at this moment. The maximal change of this PID value
	float prev_PID;		// The previous PID value
} static PIDdefault = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, TIME_DIFF, -1000000, 1000000, 1000000, 0};

typedef struct PIDstruct PIDvariables;

///////////////////////////////////////////////////// FUNCTIONS
/**
 * Initializes motor wattage dependent constants
 */
void control_util_Init();

/**
 * Initializes the PID values.
 * 
 * Loads the constants into the struct and sets up the default PID parameters.
 */
void initPID(PIDvariables* PID, float kP, float kI, float kD);

//clamps the input
//static float clamp(float input, float min, float max){
//	if (input<min){
//		return min;
//	} else if (input>max) {
//		return max;
//	} else {
//		return input;
//	}
//}

//limits the change in PID value
//static float ramp(float new_PID, float ramp, float prev_PID){
//	if (new_PID-prev_PID>ramp){
//		return (prev_PID+ramp);
//	} else if (new_PID-prev_PID<-ramp){
//		return (prev_PID-ramp);
//	} else {
//		return new_PID;
//	}
//}

//PID control, inline to not have multiple implementation error
float PID(float err, PIDvariables* K);

//Scales the angle to the range Pi to -Pi in radians
float constrainAngle(float x);

#endif /* UTILS_CONTROL_UTIL_H_ */
