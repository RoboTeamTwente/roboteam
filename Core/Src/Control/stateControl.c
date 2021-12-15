
#include "stateControl.h"

///////////////////////////////////////////////////// VARIABLES

static PID_states status = off;
static PIDvariables stateK[3];
static float stateRef[3] = {0.0f};
static float wheelRef[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static float state[3] = {0.0f};

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

//Transforms body velocity to wheel speed
static void body2Wheels(float wheelSpeed[4], float state[3]);

//Transfer global coordinate frame to local coordinate frame
static void global2Local(float input[3], float output[2], float yaw);

//Determine the desired wheel speed given the desired velocities
static void translationVelControl(float state[3], float velRef[3], float translationalRef[4]);

//Determine the desired wheel speed given the desired angle
static float angleControl(float angleRef, float angle);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

int stateControl_Init(){
	bool MOTORS_50W = true; // Keep this on the offchance that we're going to use the 30W motors again
	status = on;
	if (MOTORS_50W) {
		// 50 W
		initPID(&stateK[body_x], 0.1, 0.0, 0.0);
		initPID(&stateK[body_y], 0.4, 0.0, 0.0);
		initPID(&stateK[body_w], 20.0, 5.0, 0.0);
	} else {
		// 30 W
		initPID(&stateK[body_x], 0.1, 0.0, 0.0);
		initPID(&stateK[body_y], 0.4, 0.0, 0.0);
		initPID(&stateK[body_w], 20.0, 30.0, 0.0);
	}
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
		float translationalRef[4] = {0.0f};
		translationVelControl(state, stateRef, translationalRef);

		float angularRef = angleControl(stateRef[body_w], state[body_w]);

		for (wheel_names wheel=wheels_RF; wheel<=wheels_LF; wheel++){
			wheelRef[wheel] = translationalRef[wheel] + angularRef;
		}
	}
}

void stateControl_SetRef(float input[3]){
	stateRef[body_x] = input[body_x];
	stateRef[body_y] = input[body_y];
	stateRef[body_w] = input[body_w];
}

float* stateControl_GetWheelRef() {
	return wheelRef;
}

void stateControl_SetState(float input[3]){
	state[body_x] = input[body_x];
	state[body_y] = input[body_y];
	state[body_w] = input[body_w];
}

void stateControl_ResetAngleI(){
	stateK[body_w].I = 0;
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

static void body2Wheels(float wheelSpeed[4], float vel[3]){
	// Transformation from translational velocities to wheel speeds
	wheelSpeed[wheels_RF] = (vel[body_x] * cosFront + vel[body_y] * sinFront) / rad_wheel;
	wheelSpeed[wheels_RB] = (vel[body_x] * cosBack + vel[body_y] * -sinBack) / rad_wheel;
	wheelSpeed[wheels_LB] = (vel[body_x] * -cosBack + vel[body_y] * -sinBack) / rad_wheel;
	wheelSpeed[wheels_LF] = (vel[body_x] * -cosFront + vel[body_y] * sinFront) / rad_wheel;
}

static void global2Local(float global[3], float local[3], float  yaw){
	//trigonometry
	local[body_x] = cosf(yaw)*global[body_x]+sinf(yaw)*global[body_y];
	local[body_y] = -sinf(yaw)*global[body_x]+cosf(yaw)*global[body_y];
	local[body_w] = global[body_w];
}

static void translationVelControl(float state[3], float stateRef[3], float translationalRef[4]){
	float stateLocalRef[3] = {0, 0, 0};
	global2Local(stateRef, stateLocalRef, state[body_w]); //transfer global to local

	// Manually adjusting velocity command
	//     Explanation: see Velocity Difference file on drive (https://docs.google.com/document/d/1pGKysiwpu19DKLpAZ4GpluMV7UBhBQZ65YMTtI7bd_8/)
	stateLocalRef[body_x] = 1.12 * stateLocalRef[body_x];
	stateLocalRef[body_y] = 1.1 * stateLocalRef[body_y];

	// Local control
	float velxErr = (stateLocalRef[body_x] - state[body_x]);
	float velyErr = (stateLocalRef[body_y] - state[body_y]);
	stateLocalRef[body_x] += PID(velxErr, &stateK[body_x]);
	stateLocalRef[body_y] += PID(velyErr, &stateK[body_y]);

	body2Wheels(translationalRef, stateLocalRef); //translate velocity to wheel speed
}

static float angleControl(float angleRef, float angle){
	static float prevangleErr = 0;
	float angleErr = constrainAngle(angleRef - angle);//constrain it to one circle turn
	if (angleErr == 0){
		angleErr = 0.000001*prevangleErr;
	}
	if (fabs(angleErr) < YAW_MARGIN || prevangleErr/angleErr < 0) {
		stateK[body_w].I = 0;
	}
	prevangleErr = angleErr;
	return PID(angleErr, &stateK[body_w]);// PID control from control_util.h
}




