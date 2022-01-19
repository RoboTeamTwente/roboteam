#include "main.h"
#include "robot.h"

#include "control_util.h"
#include "gpio_util.h"
#include "tim_util.h"
#include "peripheral_util.h"
#include "PuTTY.h"
#include "wheels.h"
#include "stateControl.h"
#include "stateEstimation.h"
#include "dribbler.h"
#include "shoot.h"
#include "Wireless.h"
#include "buzzer.h"
#include "MTi.h"
#include "yawCalibration.h"
#include "iwdg.h"
#include "ballSensor.h"
#include "testFunctions.h"
#include "logging.h"

#include "rem.h"

#include "RobotCommand.h"
#include "RobotFeedback.h"
#include "RobotBuzzer.h"
#include "RobotStateInfo.h"

#include "time.h"
#include <unistd.h>
#include <stdio.h>

#define NO_ROTATION_TIME 6 				// time [s] the robot will halt at startup to let the xsens calibrate
#define XSENS_FILTER XFP_VRU_general 	// filter mode that will be used by the xsens

static bool SEND_ROBOT_STATE_INFO = false;
static const bool USE_PUTTY = false;

SX1280* SX;
MTi_data* MTi;

uint8_t message_buffer_in[127]; // TODO set this to something like MAX_BUF_LENGTH
uint8_t message_buffer_out[127];

RobotCommandPayload robotCommandPayload = {0};
RobotBuzzerPayload robotBuzzerPayload = {0};
RobotFeedback robotFeedback = {0};
RobotStateInfo robotStateInfo = {0};
RobotStateInfoPayload robotStateInfoPayload = {0};


RobotCommand activeRobotCommand = {0};
float activeStateReference[3];

StateInfo stateInfo = {0.0f, false, {0.0}, 0.0f, 0.0f, {0.0}};
bool halt = true;
bool xsens_CalibrationDone = false;
bool xsens_CalibrationDoneFirst = true;
volatile bool REM_last_packet_had_correct_version = true;
IWDG_Handle* iwdg;


volatile uint32_t counter_loop = 0;
volatile uint32_t counter_htim6 = 0;
volatile uint32_t counter_htim7 = 0;
volatile uint32_t counter_htim10 = 0;
volatile uint32_t counter_htim11 = 0;
uint32_t timestamp_initialized = 0;


bool isSerialConnected = false;
uint32_t timeLastPacket = 0;

uint32_t heartbeat_17ms_counter = 0;
uint32_t heartbeat_17ms = 0;
uint32_t heartbeat_100ms = 0;
uint32_t heartbeat_1000ms = 0;




void executeCommands(RobotCommand* robotCommand){
	float stateReference[3];
	stateReference[body_x] = (robotCommand->rho) * cosf(robotCommand->theta);
	stateReference[body_y] = (robotCommand->rho) * sinf(robotCommand->theta);
	stateReference[body_w] = robotCommand->angle;
	stateControl_SetRef(stateReference);
	dribbler_SetSpeed(robotCommand->dribbler);
	shoot_SetPower(robotCommand->kickChipPower);

	if (robotCommand->doKick) {
		if (ballPosition.canKickBall || robotCommand->doForce){
			shoot_Shoot(shoot_Kick);
		}
	}
	else if (robotCommand->doChip) {
		if (ballPosition.canKickBall || robotCommand->doForce) {
			shoot_Shoot(shoot_Chip);
		}
	}
}

void resetRobotCommand(RobotCommand* robotCommand){
	robotCommand->doKick = false;
	robotCommand->doChip = false;
	robotCommand->doForce = false;
	robotCommand->useCameraAngle = false;
	robotCommand->rho = 0.;
	robotCommand->theta = 0.;
	robotCommand->angle = 0.;
	robotCommand->cameraAngle = 0.;
	robotCommand->dribbler = 0;
	robotCommand->kickChipPower = 0;
	robotCommand->angularControl = false;
	robotCommand->feedback = false;
}

void printRobotStateData() {
	// The need for IWDG_Refresh(iwdg) worries me, since it means this function
	// takes a VERY long time. Might screw with other stuff that comes after at
	// wherever this function is called.

	LOG("-------Robot state data--------\n");
	LOG_printf("halt=%u  braking=%u\n", halt, wheels_GetWheelsBraking());
	IWDG_Refresh(iwdg);

	LOG_printf("Wheels refs   RF=%.2f RB=%.2f LB=%.2f LF=%.2f\n", 
	stateControl_GetWheelRef()[wheels_RF], stateControl_GetWheelRef()[wheels_RB], 
	stateControl_GetWheelRef()[wheels_LB], stateControl_GetWheelRef()[wheels_LF]);
	IWDG_Refresh(iwdg);

	uint32_t wheel_PWMs[4];
	wheels_GetPWM(wheel_PWMs);
	LOG_printf("Wheels pwms   RF=%.2f RB=%.2f LB=%.2f LF=%.2f\n", 
	wheel_PWMs[wheels_RF], wheel_PWMs[wheels_RB], 
	wheel_PWMs[wheels_LB], wheel_PWMs[wheels_LF]);
	IWDG_Refresh(iwdg);

	float measured_wheel_speeds[4];
	wheels_GetMeasuredSpeeds(measured_wheel_speeds);
	LOG_printf("Wheels rad/s  RF=%.2f RB=%.2f LB=%.2f LF=%.2f\n", 
	measured_wheel_speeds[wheels_RF], measured_wheel_speeds[wheels_RB], 
	measured_wheel_speeds[wheels_LB], measured_wheel_speeds[wheels_LF]);
	IWDG_Refresh(iwdg);

	LOG_printf("XSens   x=%.2f m/s^2  y=%.2f m/s^2  yaw=%.2f  omega=%.2f rad/s\n", 
	MTi->acc[body_x], MTi->acc[body_y], 
	stateEstimation_GetState()[body_w], MTi->gyr[2]);
	IWDG_Refresh(iwdg);
	
	LOG_printf("Kalman  x=%.2f m/s  y=%.2f m/s\n", 
	stateEstimation_GetState()[body_x], stateEstimation_GetState()[body_y]);
	IWDG_Refresh(iwdg);
}



// ----------------------------------------------------- INIT -----------------------------------------------------
void init(void){

    set_Pin(OUT1_pin, HIGH);  // reference pin for motor wattage
    set_Pin(OUT2_pin, HIGH);  // reference pin for feedback header
	
	// Turn off all leds. Use leds to indicate init() progress
	set_Pin(LED0_pin, 0); set_Pin(LED1_pin, 0); set_Pin(LED2_pin, 0); set_Pin(LED3_pin, 0); set_Pin(LED4_pin, 0); set_Pin(LED5_pin, 0); set_Pin(LED6_pin, 0);
 
	LOG("[init] Last programmed on " __DATE__ "\n");

	/* Read ID from switches */
	ROBOT_ID = get_Id();
	LOG_printf("[init] ROBOT_ID %d\n", ROBOT_ID);
	set_Pin(LED0_pin, 1);

	/* Read jumper */
	SEND_ROBOT_STATE_INFO = !read_Pin(IN1_pin);
	// LOG_printf("[init] SEND_ROBOT_STATE_INFO: %s\n", (SEND_ROBOT_STATE_INFO ? "True" : "False"));

	/* Initialize buzzer */
	buzzer_Init();
	buzzer_Play_QuickBeepUp();
	set_Pin(LED1_pin, 1);

	/* === Wired communication with robot; Either REM to send RobotCommands, or Putty for interactive terminal */
	if(USE_PUTTY){
		/* Initialize Putty. Not possible when REM_UARTinit() is called */
		Putty_Init();
	}else{
		/* Initialize Roboteam_Embedded_Messages. Not possible when Putty_Init() is called */
		/* Can now receive RobotCommands (and other packets) via UART */
		robotCommandIsFresh = 0;
		REM_UARTinit(UART_PC);
	}
	set_Pin(LED2_pin, 1);

    // Initialize control constants
    control_util_Init();
    wheels_Init();
    stateControl_Init();
    stateEstimation_Init();
    shoot_Init();
    dribbler_Init();
    if(ballSensor_Init()) LOG("Ballsensor initialized\n");
    set_Pin(LED3_pin, 1);
		
	/* Initialize the SX1280 wireless chip */
	// TODO figure out why a hardfault occurs when this is disabled
	if(read_Pin(IN2_pin)){
		SX = Wireless_Init(BLUE_CHANNEL, COMM_SPI);
		LOG("[init] BLUE CHANNEL\n");
	}else{
		SX = Wireless_Init(YELLOW_CHANNEL, COMM_SPI);
		LOG("[init] YELLOW CHANNEL\n");
	}
    
	SX->SX_settings->syncWords[0] = robot_syncWord[ROBOT_ID];
    setSyncWords(SX, SX->SX_settings->syncWords[0], 0x00, 0x00);
    setRX(SX, SX->SX_settings->periodBase, WIRELESS_RX_COUNT);
	set_Pin(LED4_pin, 1);

	/* Initialize the XSens chip */
	LOG("[init] Initializing XSens\n");
    MTi = MTi_Init(NO_ROTATION_TIME, XSENS_FILTER);
    if(MTi == NULL){
		LOG("[init] Failed to initialize XSens\n");
		buzzer_Play_WarningOne();
		HAL_Delay(1500);
	}
	set_Pin(LED5_pin, 1);

	LOG("[init] Initialized\n");
    
	/* Initialize watchdog (resets system after it has crashed) */
	IWDG_Init(iwdg); 

	/* Turn of all leds. Will now be used to indicate robot status */
	set_Pin(LED0_pin, 0); set_Pin(LED1_pin, 0); set_Pin(LED2_pin, 0); set_Pin(LED3_pin, 0); set_Pin(LED4_pin, 0); set_Pin(LED5_pin, 0); set_Pin(LED6_pin, 0);
	buzzer_Play_ID(ROBOT_ID);
	
	timestamp_initialized = HAL_GetTick();

	/* Set the heartbeat timers */
	heartbeat_17ms   = timestamp_initialized + 17;
	heartbeat_100ms  = timestamp_initialized + 100;
	heartbeat_1000ms = timestamp_initialized + 1000;
}



// ----------------------------------------------------- MAIN LOOP -----------------------------------------------------
void loop(void){
	uint32_t currentTime = HAL_GetTick();
	counter_loop++;

	/* Send anything in the log buffer over UART */
	if(0 < strlen(logBuffer)){
		HAL_UART_Transmit(UART_PC, (uint8_t*) logBuffer, strlen(logBuffer), 10);
		logBuffer[0] = '\0';
	}
	
	// If a RobotCommand came in via UART
	if(robotCommandIsFresh == 1){
		robotCommandIsFresh = 0;
		timeLastPacket = currentTime;
		decodeRobotCommand(&activeRobotCommand,&robotCommandPayload);

		toggle_Pin(LED6_pin);
	}

	// Play a warning if a REM packet with an incorrect version was received
	if(!REM_last_packet_had_correct_version)
		if(!buzzer_IsPlaying())
			buzzer_Play_WarningTwo();

	// If serial packet is no older than 250ms, assume connected via wire
	isSerialConnected = (currentTime - timeLastPacket) < 250;

    // Refresh Watchdog timer
    IWDG_Refresh(iwdg);
    Putty_Callback();

	// Check XSens
    xsens_CalibrationDone = (MTi->statusword & (0x18)) == 0; // if bits 3 and 4 of status word are zero, calibration is done
    halt = !(xsens_CalibrationDone && (checkWirelessConnection() || isSerialConnected)) || !REM_last_packet_had_correct_version;
    if (halt) {
		// toggle_Pin(LED5_pin);
        stateControl_ResetAngleI();
        resetRobotCommand(&activeRobotCommand);
		REM_last_packet_had_correct_version = true;
    }

    // Unbrake wheels when Xsens calibration is done
    if (xsens_CalibrationDoneFirst && xsens_CalibrationDone) {
        xsens_CalibrationDoneFirst = false;
        wheels_Unbrake();
    }

    // Update test (if active)
    test_Update();
    
    // Go through all commands
    if (!halt) {
        executeCommands(&activeRobotCommand);
    }

    // Create RobotFeedback
	robotFeedback.header = PACKET_TYPE_ROBOT_FEEDBACK;
	robotFeedback.remVersion= LOCAL_REM_VERSION;
    robotFeedback.id = ROBOT_ID;
    robotFeedback.XsensCalibrated = xsens_CalibrationDone;
    // robotFeedback.batteryLevel = (batCounter > 1000);
    robotFeedback.ballSensorWorking = ballSensor_isInitialized();
    robotFeedback.hasBall = ballPosition.canKickBall;
    robotFeedback.ballPos = ballSensor_isInitialized() ? (-.5 + ballPosition.x / 700.) : 0;

    float vx = stateEstimation_GetState()[body_x];
    float vy = stateEstimation_GetState()[body_y];
    robotFeedback.rho = sqrt(vx*vx + vy*vy);
    robotFeedback.angle = stateEstimation_GetState()[body_w];
    robotFeedback.theta = atan2(vy, -vx);
    robotFeedback.wheelBraking = wheels_GetWheelsBraking(); // TODO Locked feedback has to be changed to brake feedback in PC code
    robotFeedback.rssi = SX->Packet_status->RSSISync/2; // TODO scale this between 0 and 15? Check REM packet definition
    
	if(SEND_ROBOT_STATE_INFO){
		robotStateInfo.header = PACKET_TYPE_ROBOT_STATE_INFO;
		robotStateInfo.remVersion = LOCAL_REM_VERSION;
		robotStateInfo.id = ROBOT_ID;
		robotStateInfo.xsensAcc1 = stateInfo.xsensAcc[0];
		robotStateInfo.xsensAcc2 = stateInfo.xsensAcc[1];
		robotStateInfo.xsensYaw = yaw_GetCalibratedYaw();
		robotStateInfo.rateOfTurn = stateInfo.rateOfTurn;
		robotStateInfo.wheelSpeed1 = stateInfo.wheelSpeeds[0];
		robotStateInfo.wheelSpeed2 = stateInfo.wheelSpeeds[1];
		robotStateInfo.wheelSpeed3 = stateInfo.wheelSpeeds[2];
		robotStateInfo.wheelSpeed4 = stateInfo.wheelSpeeds[3];
	}

    // Heartbeat every 17ms	
	if(heartbeat_17ms < HAL_GetTick()){
		uint32_t now = HAL_GetTick();
		while (heartbeat_17ms < now) heartbeat_17ms += 17;
	}	

    // Heartbeat every 100ms	
	if(heartbeat_100ms < HAL_GetTick()){
		uint32_t now = HAL_GetTick();
		while (heartbeat_100ms < now) heartbeat_100ms += 100;
	}

	// Heartbeat every 1000ms
	if(heartbeat_1000ms < HAL_GetTick()){
		uint32_t now = HAL_GetTick();
		while (heartbeat_1000ms < now) heartbeat_1000ms += 1000;
		
		LOG("Tick\n");
		// if(xsens_CalibrationDone) printRobotStateData();

        // Toggle liveliness LED
        toggle_Pin(LED0_pin);
		
        // Check if ballsensor connection is still correct
        if ( !ballSensor_isInitialized() ) {
            ballSensor_Init();
            __HAL_I2C_DISABLE(BS_I2C);
            HAL_Delay(1);
            __HAL_I2C_ENABLE(BS_I2C);
        }
    }

    /*
    * LEDs for debugging
    */

    // LED0 : toggled every second while alive
    // LED1 : on while xsens startup calibration is not finished
    // LED2 : on when braking
    // LED3 : on when halting
    // LED4 : on when ballsensor says ball is within kicking range
    // LED5 : on when battery is empty
    // LED6 : toggled when a packet is received

    // LED0 done in PuTTY prints above
    set_Pin(LED1_pin, !xsens_CalibrationDone);
    set_Pin(LED2_pin, wheels_GetWheelsBraking());
    set_Pin(LED3_pin, halt);
    set_Pin(LED4_pin, ballPosition.canKickBall);
    // set_Pin(LED5_pin, (read_Pin(Bat_pin) && batCounter > 1000));
    // LED6 done in Wireless.c
}


// ----------------------------------------------------- STM HAL CALLBACKS -----------------------------------------------------
/* HAL_SPI_TxRxCpltCallback = Callback for either SPI Transmit or Receive complete */
/* This function is triggered after calling HAL_SPI_TransmitReceive_IT */
/* Since we transmit everything using blocking mode, this function should only be called when we receive something */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi){

	// If we received data from the SX1280
	if(hspi->Instance == SX->SPI->Instance) {

		Wireless_DMA_Handler(SX, message_buffer_in);

		uint8_t total_packet_length = SX->payloadLength;
		uint8_t total_bytes_processed = 0;

		while(total_bytes_processed < total_packet_length){
				
			uint8_t packet_header = message_buffer_in[total_bytes_processed];

			if(packet_header == PACKET_TYPE_ROBOT_COMMAND){
				memcpy(robotCommandPayload.payload, message_buffer_in + total_bytes_processed, PACKET_SIZE_ROBOT_COMMAND);
				REM_last_packet_had_correct_version &= RobotCommand_get_remVersion(&robotCommandPayload) == LOCAL_REM_VERSION;
				decodeRobotCommand(&activeRobotCommand,&robotCommandPayload);

				total_bytes_processed += PACKET_SIZE_ROBOT_COMMAND;
				continue;
			}

			if(packet_header == PACKET_TYPE_ROBOT_BUZZER){
				RobotBuzzerPayload* rbp = (RobotBuzzerPayload*) (message_buffer_in + total_bytes_processed);
				REM_last_packet_had_correct_version &= RobotBuzzer_get_remVersion(rbp) == LOCAL_REM_VERSION;
				uint16_t period = RobotBuzzer_get_period(rbp);
				float duration = RobotBuzzer_get_duration(rbp);
				buzzer_Play_note(period, duration);

				total_bytes_processed += PACKET_SIZE_ROBOT_BUZZER;
				continue;
			}

			sprintf(logBuffer, "[SPI_TxRxCplt] Error! At %d of %d bytes. [@] = %d\n", total_bytes_processed, total_packet_length, packet_header);
			break;
		}
	}

	// If we received data from the XSens
	else if(hspi->Instance == MTi->SPI->Instance){
		MTi_SPI_RxCpltCallback(MTi);
	}
}

/* Callback for when bytes have been received via the UART */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == UART_PC->Instance){
		if(USE_PUTTY){
			Putty_UARTCallback(huart);
		} else {
			REM_UARTCallback(huart);
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == SX_IRQ_pin.PIN) {

		/* TODO all this encoding stuff is done not only when a packet has been received from the basestation RX_DONE (which is intended)
		* It is also triggered when a packet has been sent back to the basestation TX_DONE (so basically this is done double)
		* And it's also done on RX_TIMEOUT (every 250ms, so not that bad)
		* Somehow, make sure that this is only done on RX_DONE
		*/
		uint8_t total_packet_length = 0;

		encodeRobotFeedback( (RobotFeedbackPayload*) (message_buffer_out + total_packet_length), &robotFeedback);
		total_packet_length += PACKET_SIZE_ROBOT_FEEDBACK;

		if(SEND_ROBOT_STATE_INFO){
			encodeRobotStateInfo( (RobotStateInfoPayload*) (message_buffer_out + total_packet_length), &robotStateInfo);
			total_packet_length += PACKET_SIZE_ROBOT_STATE_INFO;
		}

		Wireless_IRQ_Handler(SX, message_buffer_out, total_packet_length);

	}else if(GPIO_Pin == MTi_IRQ_pin.PIN){
		MTi_IRQ_Handler(MTi);
	}else if (GPIO_Pin == BS_IRQ_pin.PIN){
		// TODO: make this work and use instead of the thing in the while loop
		ballSensor_IRQ_Handler();
	}
}

// Handles the interrupts of the different timers.

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{	
	// Old Geneva timer. Needs to be properly disabled in CubeMX
	if(htim->Instance == htim6.Instance){
		counter_htim6++;
	}
	else if(htim->Instance == htim7.Instance) {
		counter_htim7++;

		if(test_isTestRunning(wheels) || test_isTestRunning(normal)) {
            wheels_Update();
            return;
        }

		if( halt && !test_isTestRunning(square)){
			wheels_Stop();
			return;
		}

		// State estimation
		stateInfo.visionAvailable = activeRobotCommand.useCameraAngle;
		stateInfo.visionYaw = activeRobotCommand.cameraAngle; // TODO check if this is scaled properly with the new REM messages
		
		wheels_GetMeasuredSpeeds(stateInfo.wheelSpeeds);

		stateInfo.xsensAcc[body_x] = MTi->acc[body_x];
		stateInfo.xsensAcc[body_y] = MTi->acc[body_y];
		stateInfo.xsensYaw = (MTi->angles[2]*M_PI/180); //Gradients to Radians
		stateInfo.rateOfTurn = MTi->gyr[2];
		stateEstimation_Update(&stateInfo);

		// State control
		stateControl_SetState(stateEstimation_GetState());
		stateControl_Update();

		wheels_SetSpeeds( stateControl_GetWheelRef() );
		wheels_Update();

	}
	else if (htim->Instance == htim10.Instance) {
		counter_htim10++;
		buzzer_Callback();
	}
	else if (htim->Instance == htim11.Instance) {
		counter_htim11++;
		shoot_Callback();
	}
}
