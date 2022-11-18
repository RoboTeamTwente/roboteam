
#include "stateControl.h"

///////////////////////////////////////////////////// VARIABLES

// The current status of the system.
static PID_states status = off;

// The PID values for x, y, w and yaw.
static PIDvariables stateK[4];

// The global x, y, w and yaw velocities to be achieved.
static float stateRef[4] = {0.0f}; 

// The wheel velocities to be achieved.
static float wheelRef[4] = {0.0f, 0.0f, 0.0f, 0.0f};

// The current x, y, w and yaw velocities.
static float state[4] = {0.0f};

// Whether to move to an absolute angle. If true yes, otherwise use angular velocity.
static bool useAbsoluteAngle = true;
static bool previousUseAbsoluteAngle = true;

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

/**
 * Translates the velocity from a body perspective to wheel speeds.
 * 
 * @param wheelSpeed The speed to be achieved for each wheel.
 * @param vel 		 The velocities to be achieved seen from the body perspective.
 */
static void body2Wheels(float wheelSpeed[4], float state[3]);

/**
 * Translates the global coordinate frame to the local coordinate frame
 * 
 * @param global 	The global coordinates
 * @param local 	The local coordinates
 * @param yaw 		Yaw
 */
static void global2Local(float global[4], float local[4], float yaw);

/**
 * Determines the desired wheel speeds given the desired velocities
 * 
 * @param state 			The current x, y, w and yaw speeds seen from the body
 * @param velRef 			The instructed global x, y, w and yaw speeds
 * @param velocityWheelRef 	The resulting wheel speeds that should be achieved for each wheel
 */
static void velocityControl(float state[3], float velRef[4], float velocityWheelRef[4]);

/**
 * Determine the speed that the wheels should achieve in order to move towards a desired angle.
 * 
 * @param angleRef The angle to be achieved.
 * @param angle    The current angle.
 * @return float   The PID value to achieve the set angle.
 */
static float absoluteAngleControl(float angleRef, float angle);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS


int stateControl_Init(){
	status = on;
	initPID(&stateK[body_x], default_P_gain_x, default_I_gain_x, default_D_gain_x);
	initPID(&stateK[body_y], default_P_gain_y, default_I_gain_y, default_D_gain_y);
	initPID(&stateK[body_w], default_P_gain_w, default_I_gain_w, default_D_gain_w); 
	initPID(&stateK[body_yaw], default_P_gain_yaw, default_I_gain_yaw, default_D_gain_yaw);
	HAL_TIM_Base_Start_IT(TIM_CONTROL);
	return 0;
}

int stateControl_DeInit(){
	status = off;
	HAL_TIM_Base_Stop_IT(TIM_CONTROL);
	return 0;
}

void stateControl_Update(){
	if (status == on){
		float velocityWheelRef[4] = {0.0f};
		velocityControl(state, stateRef, velocityWheelRef);

		float angularRef = useAbsoluteAngle ? absoluteAngleControl(stateRef[body_yaw], state[body_yaw]) : 0.0f;

		for (wheel_names wheel=wheels_RF; wheel<=wheels_LF; wheel++){
			wheelRef[wheel] = velocityWheelRef[wheel] + angularRef;
		}
	}
}

void stateControl_SetRef(float input[4]){
	stateRef[body_x] = input[body_x];
	stateRef[body_y] = input[body_y];
	stateRef[body_w] = input[body_w];
	stateRef[body_yaw] = input[body_yaw];
}

float* stateControl_GetWheelRef() {
	return wheelRef;
}

void stateControl_SetState(float input[4]){
	state[body_x] = input[body_x];
	state[body_y] = input[body_y];
	state[body_w] = input[body_w];
	state[body_yaw] = input[body_yaw];
}

void stateControl_GetPIDGains(PIDvariables gains[4]){
	gains[body_x].kP = stateK[body_x].kP;
	gains[body_x].kI = stateK[body_x].kI;
	gains[body_x].kD = stateK[body_x].kD;

	gains[body_y].kP = stateK[body_y].kP;
	gains[body_y].kI = stateK[body_y].kI;
	gains[body_y].kD = stateK[body_y].kD;

	gains[body_w].kP = stateK[body_w].kP;
	gains[body_w].kI = stateK[body_w].kI;
	gains[body_w].kD = stateK[body_w].kD;

	gains[body_yaw].kP = stateK[body_yaw].kP;
	gains[body_yaw].kI = stateK[body_yaw].kI;
	gains[body_yaw].kD = stateK[body_yaw].kD;
}

float stateControl_GetIntegral(body_handles direction) {
	return stateK[direction].I;
}

void stateControl_useAbsoluteAngle(bool angularControl){
	if (angularControl != previousUseAbsoluteAngle){
		stateControl_ResetAngleI();
		stateControl_ResetPID();
	}
	previousUseAbsoluteAngle = angularControl;
    useAbsoluteAngle = angularControl;
}

void stateControl_SetPIDGains(REM_RobotSetPIDGains* PIDGains){
    stateK[body_x].kP = PIDGains->PbodyX;
    stateK[body_x].kI = PIDGains->IbodyX;
    stateK[body_x].kD = PIDGains->DbodyX;

    stateK[body_y].kP = PIDGains->PbodyY;
    stateK[body_y].kI = PIDGains->IbodyY;
    stateK[body_y].kD = PIDGains->DbodyY;

    stateK[body_w].kP = PIDGains->PbodyW;
    stateK[body_w].kI = PIDGains->IbodyW;
    stateK[body_w].kD = PIDGains->DbodyW;

    stateK[body_yaw].kP = PIDGains->PbodyYaw;
    stateK[body_yaw].kI = PIDGains->IbodyYaw;
    stateK[body_yaw].kD = PIDGains->DbodyYaw;
}

void stateControl_ResetAngleI(){
	stateK[body_yaw].I = 0;
	stateK[body_w].I = 0;
}

void stateControl_ResetPID(){
	stateK[body_yaw].prev_e = 0;
	stateK[body_yaw].prev_PID = 0;
	stateK[body_w].prev_e = 0;
	stateK[body_w].prev_PID = 0;
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

static void body2Wheels(float wheelSpeed[4], float vel[3]){
	wheelSpeed[wheels_RF] = (vel[body_x] * cosFront + vel[body_y] * sinFront) / rad_wheel;
	wheelSpeed[wheels_RB] = (vel[body_x] * cosBack + vel[body_y] * -sinBack) / rad_wheel;
	wheelSpeed[wheels_LB] = (vel[body_x] * -cosBack + vel[body_y] * -sinBack) / rad_wheel;
	wheelSpeed[wheels_LF] = (vel[body_x] * -cosFront + vel[body_y] * sinFront) / rad_wheel;

	if (!useAbsoluteAngle) {
        for (wheel_names wheel=wheels_RF; wheel<=wheels_LF; wheel++){
            wheelSpeed[wheel] += vel[body_w] * rad_robot / rad_wheel;
        }
	}
}

static void global2Local(float global[4], float local[4], float  yaw){
	//trigonometry
	local[body_x] = cosf(yaw)*global[body_x]+sinf(yaw)*global[body_y];
	local[body_y] = -sinf(yaw)*global[body_x]+cosf(yaw)*global[body_y];
    local[body_w] = global[body_w];
	local[body_yaw] = global[body_yaw];
}

static void velocityControl(float state[3], float velRef[4], float velocityWheelRef[4]){
	float stateLocalRef[3] = {0, 0, 0};
	global2Local(velRef, stateLocalRef, state[body_yaw]); //transfer global to local

	// Manually adjusting velocity command
	//     Explanation: see Velocity Difference file on drive (https://docs.google.com/document/d/1pGKysiwpu19DKLpAZ4GpluMV7UBhBQZ65YMTtI7bd_8/)
	stateLocalRef[body_x] = 1.12 * stateLocalRef[body_x];
	stateLocalRef[body_y] = 1.1 * stateLocalRef[body_y];

	// Local control
	float velxErr = (stateLocalRef[body_x] - state[body_x]);
	float velyErr = (stateLocalRef[body_y] - state[body_y]);
	float velwErr = (stateLocalRef[body_w] - state[body_w]);

	stateLocalRef[body_x] += PID(velxErr, &stateK[body_x]);
	stateLocalRef[body_y] += PID(velyErr, &stateK[body_y]);
	stateLocalRef[body_w] += PID(velwErr, &stateK[body_w]);

	body2Wheels(velocityWheelRef, stateLocalRef); //translate velocity to wheel speed
}

static float absoluteAngleControl(float angleRef, float angle){
	static float prevangleErr = 0;
	float angleErr = constrainAngle(angleRef - angle);//constrain it to one circle turn
	if (angleErr == 0){
		angleErr = 0.000001*prevangleErr;
	}
	if (fabs(angleErr) < YAW_MARGIN || prevangleErr/angleErr < 0) {
		stateK[body_yaw].I = 0;
	}
	prevangleErr = angleErr;
	return PID(angleErr, &stateK[body_yaw]);// PID control from control_util.h
}