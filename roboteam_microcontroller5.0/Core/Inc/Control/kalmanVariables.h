
/* Description: Kalman filter variables
 *
 * Instructions:
 * 1) Initializes the necessary arrays and variables for the Kalman filter
 *
 * Extra functions:
 *
 * Notes:
 *
*/

#ifndef KALMAN_KALMANVARIABLES_H_
#define KALMAN_KALMANVARIABLES_H_
///////////////////////////////////////////////////// DEFINITIONS

#define STATE 4			// The dimensions for the current state
#define OBSERVE 4		// The dimensions for the measured state
#define TIMESTEP 0.01	// The time step between each iteration (100Hz loop)

// certainties
#define VEL_VAR 0.000025F 	// variance in the velocity measurements
#define ACC_VAR 6.25F 		// variance in the acceleration measurements
#define STATE_VAR 0.5F 		// variance in the predicted state
#define RAND_VAR 0.5F 		// variance in the random force

///////////////////////////////////////////////////// VARIABLES

// The state transition matrix.
float aF[STATE*STATE] = {
		1, TIMESTEP, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, TIMESTEP,
		0, 0, 0, 1};

// The observed model	
float aH[OBSERVE*STATE] = {
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1};
	
// The covariance of the observed model	
float aR[OBSERVE*OBSERVE] = {
		VEL_VAR, 0, 0, 0,
		0, ACC_VAR, 0, 0,
		0, 0, VEL_VAR, 0,
		0, 0, 0, ACC_VAR};

// The identity matrix
float aI[STATE*STATE] = {
		1,0,0,0,
		0,1,0,0,
		0,0,1,0,
		0,0,0,1};

// The old prediction matrix
float aPold[STATE*STATE] = {
		STATE_VAR,0,0,0,
		0,STATE_VAR,0,0,
		0,0,STATE_VAR,0,
		0,0,0,STATE_VAR};

// The control input matrix		
float aB[STATE*STATE] = {
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1};

// The covariance of the process model	
float aQ[STATE*STATE] = {
		TIMESTEP*TIMESTEP*RAND_VAR, TIMESTEP*RAND_VAR, 0, 0,
		TIMESTEP*RAND_VAR, RAND_VAR, 0, 0,
		0, 0, TIMESTEP*TIMESTEP*RAND_VAR, TIMESTEP*RAND_VAR,
		0, 0, TIMESTEP*RAND_VAR, RAND_VAR};

float aXold[STATE] = {0.0f};			// The old state
float az[OBSERVE] = {0.0f};				// The observed (measured) state
float aXcurrent[STATE] = {0.0f};		// The current tate
float aFX[STATE] = {0.0f};				// The X and F matrices multiplied
float aFt[STATE*STATE] = {0.0f};		// The transposed state transition matrix
float aFP[STATE*STATE] = {0.0f};		// The F and P matrices multiplied
float aPcurrent[STATE*STATE] = {0.0f};	// The current prediction matrix
float ayold[OBSERVE] = {0.0f};			// The post-fit residual measurement
float aHX[OBSERVE] = {0.0f};			// The H and X matrices multiplied
float aS[OBSERVE*OBSERVE] = {0.0f};		// The innovation covariance
float aHt[STATE*OBSERVE] = {0.0f};		// The transposed observation model matrix
float aPHt[STATE*OBSERVE] = {0.0f};		// The P and Ht matrices multiplied
float aFPFt[STATE*STATE] = {0.0f};		// The F, P and Ft matrices multiplied
float aHPHt[OBSERVE*OBSERVE] = {0.0f};	// The H, P and Ht matrices multiplied
float aK[STATE*OBSERVE] = {0.0f};		// Kalman gain
float aSi[OBSERVE*OBSERVE] = {0.0f};	// The inverse of innovation covariance
float aXnew[STATE] = {0.0f};			// The next state
float aKy[STATE] = {0.0f};				// The K and yold matrices multiplied
float aPnew[STATE*STATE] = {0.0f};		// The next prediction matrix
float aKt[OBSERVE*STATE] = {0.0f};		// The transpose of the kalman gain
float aKR[STATE*OBSERVE] = {0.0f};		// The K and R matrices multiplied
float aKRKt[STATE*STATE] = {0.0f};		// The K, R and Kt matrices multiplied
float aKH[STATE*STATE] = {0.0f};		// The K and H matrices multiplied
float aI_KH[STATE*STATE] = {0.0f};		// The result of subtracting the KH matrix from the I matrix
float aI_KHt[STATE*STATE] = {0.0f};		// The transpose of the aI_KH matrix
float aI_KHP[STATE*STATE] = {0.0f};		// The aI_KH and P matrix multiplied
float aI_KHPI_KHt[STATE*STATE] = {0.0f};// The aI_KHP and aI_KHt matrices multiplied
float aU[STATE] = {0.0f};				// The control vector
float aBU[STATE] = {0.0f};				// The B and U matrices multiplied

#endif /* KALMAN_KALMANV_H_ */
