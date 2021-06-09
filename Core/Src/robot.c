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

uint16_t ID;

uint8_t message_buffer_in[127]; // TODO set this to something like MAX_BUF_LENGTH
uint8_t message_buffer_out[127];

RobotCommandPayload robotCommandPayload = {0};
RobotBuzzerPayload robotBuzzerPayload = {0};
RobotFeedback robotFeedback = {0};
RobotStateInfo robotStateInfo = {0};
RobotStateInfoPayload robotStateInfoPayload = {0};

ReceivedData receivedData = {{0.0}, false, 0.0f, 0, 0, false, false};

StateInfo stateInfo = {0.0f, false, {0.0}, 0.0f, 0.0f, {0.0}};
bool halt = true;
bool xsens_CalibrationDone = false;
bool xsens_CalibrationDoneFirst = true;
IWDG_Handle* iwdg;


volatile uint32_t counter_htim6 = 0;
volatile uint32_t counter_htim7 = 0;
volatile uint32_t counter_htim10 = 0;
volatile uint32_t counter_htim11 = 0;
uint32_t timestamp_initialized = 0;

// Set the references from the received data and execute the desired actions.
void executeCommands(ReceivedData* receivedData) {
	stateControl_SetRef(receivedData->stateRef);
	dribbler_SetSpeed(receivedData->dribblerRef);
	shoot_SetPower(receivedData->shootPower);

	if (receivedData->do_kick) {
		if (ballPosition.canKickBall || receivedData->kick_chip_forced){
			shoot_Shoot(shoot_Kick);
		}
	}
	else if (receivedData->do_chip) {
		if (ballPosition.canKickBall || receivedData->kick_chip_forced) {
			shoot_Shoot(shoot_Chip);
		}
	}
}

void clearReceivedData(ReceivedData* receivedData) {
	receivedData->do_chip = false;
	receivedData->do_kick = false;
	receivedData->kick_chip_forced = false;
	receivedData->dribblerRef = 0;
	receivedData->shootPower = 0;
	receivedData->stateRef[body_x] = 0.0f;
	receivedData->stateRef[body_y] = 0.0f;
	receivedData->stateRef[body_w] = 0.0f;
	receivedData->visionAvailable = false;
	receivedData->visionYaw = 0.0f;
}



void printReceivedData(ReceivedData* receivedData) {
	Putty_printf("\n\r");
	Putty_printf("-----Received robot data-------\n\r");
	Putty_printf("velocity:\n\r");
	Putty_printf("  x: %f\n\r", receivedData->stateRef[body_x]);
	Putty_printf("  y: %f\n\r", receivedData->stateRef[body_y]);
	Putty_printf("yaw: %f\n\r", receivedData->stateRef[body_w]);
	Putty_printf("dribbler speed: %d %%\n\r", receivedData->dribblerRef);
	Putty_printf("shooting power: %d %%\n\r", receivedData->shootPower);
	Putty_printf("kick: %u\n\r",receivedData->do_kick);
	Putty_printf("chip: %u\n\r",receivedData->do_chip);
	Putty_printf("vision available: %u\n\r",receivedData->visionAvailable);
	Putty_printf("vision yaw: %f\n\r", receivedData->visionYaw);
	Putty_printf("XSens calibrated: %u\n\r", xsens_CalibrationDone);
	Putty_printf("\n\r");
}

void printRobotStateData() {
	Putty_printf("\n\r");
	Putty_printf("-------Robot state data--------\n\r");
	Putty_printf("halt? %u\n\r", halt);
	Putty_printf("Braking? %u\n\r", wheels_IsBraking());
	Putty_printf("velocity (Kalman):\n\r");
	Putty_printf("  x: %f m/s\n\r", stateEstimation_GetState()[body_x]);
	Putty_printf("  y: %f m/s\n\r", stateEstimation_GetState()[body_y]);
	Putty_printf("acceleration (xsens):\n\r");
	Putty_printf("  x: %f m/s^2\n\r", MTi->acc[body_x]);
	Putty_printf("  y: %f m/s^2\n\r", MTi->acc[body_y]);
	Putty_printf("yaw (calibrated): %f rad\n\r", stateEstimation_GetState()[body_w]);
	Putty_printf("Xsens rate of turn: %f rad/s\n\r", MTi->gyr[2]);
	Putty_printf("wheel refs:\n\r");
	Putty_printf("  RF: %f rad/s\n\r", stateControl_GetWheelRef()[wheels_RF]);
	Putty_printf("  RB: %f rad/s\n\r", stateControl_GetWheelRef()[wheels_RB]);
	Putty_printf("  LB: %f rad/s\n\r", stateControl_GetWheelRef()[wheels_LB]);
	Putty_printf("  LF: %f rad/s\n\r", stateControl_GetWheelRef()[wheels_LF]);
	Putty_printf("wheel speeds (encoders):\n\r");
	Putty_printf("  RF: %f rad/s\n\r", wheels_GetState()[wheels_RF]);
	Putty_printf("  RB: %f rad/s\n\r", wheels_GetState()[wheels_RB]);
	Putty_printf("  LB: %f rad/s\n\r", wheels_GetState()[wheels_LB]);
	Putty_printf("  LF: %f rad/s\n\r", wheels_GetState()[wheels_LF]);
	Putty_printf("wheel pwm:\n\r");
	Putty_printf("  RF: %d \n\r", wheels_GetPWM()[wheels_RF]);
	Putty_printf("  RB: %d \n\r", wheels_GetPWM()[wheels_RB]);
	Putty_printf("  LB: %d \n\r", wheels_GetPWM()[wheels_LB]);
	Putty_printf("  LF: %d \n\r", wheels_GetPWM()[wheels_LF]);
}

void printRobotCommand(RobotCommand* rc){
	Putty_printf("======== RobotCommand ========\r\n");
	Putty_printf("            id : %d\r\n", rc->id);
	Putty_printf("        doKick : %d\r\n", rc->doKick);
	Putty_printf("        doChip : %d\r\n", rc->doChip);
	Putty_printf("       doForce : %d\r\n", rc->doForce);
	Putty_printf("useCameraAngle : %d\r\n", rc->useCameraAngle);
	Putty_printf("           rho : %.4f\r\n", rc->rho);
	Putty_printf("         theta : %.4f\r\n", rc->theta);
	Putty_printf("         angle : %.4f\r\n", rc->angle);
	Putty_printf("   cameraAngle : %.4f\r\n", rc->cameraAngle);
	Putty_printf("      dribbler : %d\r\n", rc->dribbler);
	Putty_printf(" kickChipPower : %d\r\n", rc->kickChipPower);
	Putty_printf("angularControl : %d\r\n", rc->angularControl);
	Putty_printf("      feedback : %d\r\n", rc->feedback);
}

// ----------------------------------------------------- INIT -----------------------------------------------------
void init(void){

    set_Pin(OUT1_pin, HIGH);  // reference pin for motor wattage
    set_Pin(OUT2_pin, HIGH);  // reference pin for feedback header
	
	// Turn off all leds. Use leds to indicate init() progress
	set_Pin(LED0_pin, 0); set_Pin(LED1_pin, 0); set_Pin(LED2_pin, 0); set_Pin(LED3_pin, 0); set_Pin(LED4_pin, 0); set_Pin(LED5_pin, 0); set_Pin(LED6_pin, 0);
 
	ID = get_Id();
	Putty_printf("ID: %d\n", ID);
	set_Pin(LED0_pin, 1);


	/* Read jumper */
	SEND_ROBOT_STATE_INFO = read_Pin(IN1_pin);
	Putty_printf("SEND_ROBOT_STATE_INFO: %s\n", (SEND_ROBOT_STATE_INFO ? "True" : "False"));

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
		set_Pin(LED2_pin, 1);
	}

    // Initialize control constants
    control_util_Init();
    wheels_Init();
    stateControl_Init();
    stateEstimation_Init();
    shoot_Init();
    dribbler_Init();
    if(ballSensor_Init())
		Putty_printf("Ballsensor initialized\n");	
    set_Pin(LED3_pin, 1);
		
	/* Initialize the SX1280 wireless chip */
	// TODO figure out why a hardfault occurs when this is disabled
	Putty_printf("Initializing wireless\n");
    SX = Wireless_Init(WIRELESS_COMMAND_CHANNEL, COMM_SPI);
	SX->SX_settings->syncWords[0] = robot_syncWord[ID];
    setSyncWords(SX, SX->SX_settings->syncWords[0], 0x00, 0x00);
    setRX(SX, SX->SX_settings->periodBase, WIRELESS_RX_COUNT);
	set_Pin(LED4_pin, 1);

	/* Initialize the XSens chip */
	Putty_printf("Initializing XSens\n");
    MTi = MTi_Init(NO_ROTATION_TIME, XSENS_FILTER);
    if(MTi == NULL){
		Putty_printf("Failed to initialize XSens\n");
		buzzer_Play_WarningOne();
		HAL_Delay(1500);
	}
	set_Pin(LED5_pin, 1);

	Putty_printf("Initialized\n");
    IWDG_Init(iwdg); // Initialize watchdog (resets system after it has crashed)

	// Turn of all leds. Will now be used to indicate robot status
	set_Pin(LED0_pin, 0); set_Pin(LED1_pin, 0); set_Pin(LED2_pin, 0); set_Pin(LED3_pin, 0); set_Pin(LED4_pin, 0); set_Pin(LED5_pin, 0); set_Pin(LED6_pin, 0);
	buzzer_Play_ID(ID);
	
	timestamp_initialized = HAL_GetTick();
}

// ----------------------------------------------------- MAIN LOOP -----------------------------------------------------

bool isSerialConnected = false;
uint32_t timeLastPacket = 0;

volatile int commandCounter = 0;

float fakeAngle = 0.;
uint32_t heartbeat_17ms_counter = 0;
uint32_t heartbeat_17ms = 0;
uint32_t heartbeat_100ms = 0;
uint32_t heartbeat_1000ms = 0;

void loop(void){
	uint32_t currentTime = HAL_GetTick();

	/* Send anything in the log buffer over UART */
	if(0 < strlen(logBuffer)){
		HAL_UART_Transmit(UART_PC, (uint8_t*) logBuffer, strlen(logBuffer), 10);
		logBuffer[0] = '\0';
	}
	
	if(robotCommandIsFresh == 1){
		robotCommandIsFresh = 0;
		timeLastPacket = currentTime;
		packetToRoboData(&myRobotCommandPayload, &receivedData);
		
		toggle_Pin(LED6_pin);
		
		// printRobotCommand(&myRobotCommand);
		// printReceivedData(&receivedData);	
	}

	// If serial packet is no older than 250ms, assume connected via wire
	isSerialConnected = (currentTime - timeLastPacket) < 250;

    // Refresh Watchdog timer
    IWDG_Refresh(iwdg);
    Putty_Callback();

	// Check XSens
    xsens_CalibrationDone = (MTi->statusword & (0x18)) == 0; // if bits 3 and 4 of status word are zero, calibration is done
    halt = !(xsens_CalibrationDone && (checkWirelessConnection() || isSerialConnected));
    if (halt) {
		toggle_Pin(LED5_pin);
        stateControl_ResetAngleI();
        clearReceivedData(&receivedData);
    }

    // Unbrake wheels when Xsens calibration is done
    if (xsens_CalibrationDoneFirst && xsens_CalibrationDone) {
        xsens_CalibrationDoneFirst = false;
        wheels_Brake(false);
    }

    // Update test (if active)
    // test_Update(&receivedData);
    
    // Go through all commands
    executeCommands(&receivedData);


    // Create RobotFeedback
	robotFeedback.header = PACKET_TYPE_ROBOT_FEEDBACK;
    robotFeedback.id = ID;
    robotFeedback.XsensCalibrated = xsens_CalibrationDone;
    // robotFeedback.batteryLevel = (batCounter > 1000);
    robotFeedback.ballSensorWorking = ballSensor_isInitialized();
    robotFeedback.hasBall = ballPosition.canKickBall;
    robotFeedback.ballPos = ballSensor_isInitialized() ? (-.5 + ballPosition.x / 700.) : 0;

    float vx = stateEstimation_GetState()[body_x];
    float vy = stateEstimation_GetState()[body_y];
    robotFeedback.rho = sqrt(vx*vx + vy*vy);
    robotFeedback.angle = stateEstimation_GetState()[body_w];
    robotFeedback.theta = atan2(vy, vx) / 0.0062; // range is [-512, 511] instead of [-1024, 1023]
    robotFeedback.wheelBraking = wheels_IsBraking(); // TODO Locked feedback has to be changed to brake feedback in PC code
    robotFeedback.rssi = SX->Packet_status->RSSISync/2;
    
	if(SEND_ROBOT_STATE_INFO){
		robotStateInfo.header = PACKET_TYPE_ROBOT_STATE_INFO;
		robotStateInfo.id = ID;
		robotStateInfo.xsensAcc1 = stateInfo.xsensAcc[0];
		robotStateInfo.xsensAcc2 = stateInfo.xsensAcc[1];
		// robotStateInfo.xsensYaw = stateInfo.xsensYaw;
		robotStateInfo.xsensYaw = yaw_GetCalibratedYaw();
		robotStateInfo.rateOfTurn = stateInfo.rateOfTurn;
		robotStateInfo.wheelSpeed1 = stateInfo.wheelSpeeds[0];
		robotStateInfo.wheelSpeed2 = stateInfo.wheelSpeeds[1];
		robotStateInfo.wheelSpeed3 = stateInfo.wheelSpeeds[2];
		robotStateInfo.wheelSpeed4 = stateInfo.wheelSpeeds[3];
	}

    // Heartbeat every 17ms	
	if(heartbeat_17ms + 17 < HAL_GetTick()){
		heartbeat_17ms += 17;

		// if(!halt){
		// 	isSerialConnected = heartbeat_17ms_counter++ < 1200;
		// 	if(isSerialConnected) toggle_Pin(LED6_pin);
			
		// 	fakeAngle += 0.1;
		// 	if(M_PI <= fakeAngle) fakeAngle -= M_PI * 2;
		// 	RobotCommand_set_angle(&robotCommandPayload, fakeAngle);
		// 	packetToRoboData(&robotCommandPayload, &receivedData);
		// }
	}	

    // Heartbeat every 100ms	
	if(heartbeat_100ms + 100 < HAL_GetTick()){
		heartbeat_100ms += 100;
		// Putty_printf("100ms %d\n", halt);
		// Putty_printf("ball.x %d ball.y %d\n", ballPosition.x, ballPosition.y);
		// encodeRobotStateInfo(&robotStateInfoPayload, &robotStateInfo);
	}

	// Heartbeat every 1000ms
	if(heartbeat_1000ms + 1000 < HAL_GetTick()){
		heartbeat_1000ms += 1000;
		Putty_printf("1000ms\n");

		uint32_t msPassed = HAL_GetTick() - timestamp_initialized;
		float hz_7 = 1000 * counter_htim7 / msPassed; 
		Putty_printf("6: %d, 7: %d, 10: %d, 11: %d | hz = %2f\n", 
		counter_htim6, counter_htim7, counter_htim10, counter_htim11, hz_7);

        // Toggle liveliness LED
        toggle_Pin(LED0_pin);
		
        // Check if ballsesnor connection is still correct
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
    set_Pin(LED2_pin, wheels_IsBraking());
    set_Pin(LED3_pin, halt);
    set_Pin(LED4_pin, ballPosition.canKickBall);
    // set_Pin(LED5_pin, (read_Pin(Bat_pin) && batCounter > 1000));
    // LED6 done in Wireless.c
}


// ----------------------------------------------------- STM HAL CALLBACKS -----------------------------------------------------
/* HAL_SPI_TxRxCpltCallback = Callback for either SPI Transmit or Receive complete */
/* This function is triggered after calling HAL_SPI_TransmitReceive_IT */
/* Since we transmit everything using blocking mode, this function should only be called when we receive something */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
	// If we received data from the SX1280
	if(hspi->Instance == SX->SPI->Instance) {
		Wireless_DMA_Handler(SX, message_buffer_in);

		uint8_t total_packet_length = SX->payloadLength;
		uint8_t total_bytes_processed = 0;

		while(total_bytes_processed < total_packet_length){
				
			if(message_buffer_in[total_bytes_processed] == PACKET_TYPE_ROBOT_COMMAND){
				memcpy(robotCommandPayload.payload, message_buffer_in + total_bytes_processed, PACKET_SIZE_ROBOT_COMMAND);
				packetToRoboData(&robotCommandPayload, &receivedData);
				commandCounter++;
				total_bytes_processed += PACKET_SIZE_ROBOT_COMMAND;
				continue;
			}

			if(message_buffer_in[total_bytes_processed] == PACKET_TYPE_ROBOT_BUZZER){
				RobotBuzzerPayload* rbp = (RobotBuzzerPayload*) (message_buffer_in + total_bytes_processed);
				uint16_t period = RobotBuzzer_get_period(rbp);
				float duration = RobotBuzzer_get_duration(rbp);
				buzzer_Play_note(period, duration);
				sprintf(logBuffer, "Buz! %d %.2f\n", period, duration);

				total_bytes_processed += PACKET_SIZE_ROBOT_BUZZER;
				continue;
			}

			sprintf(logBuffer, "[SPI_TxRxCplt] Error! At %d of %d bytes. [@] = %d\n", total_bytes_processed, total_packet_length, message_buffer_in[total_bytes_processed]);
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

		if(test_isTestRunning())
			return;

		if( halt ){
			float emptyRef[4] = {0.0f, 0.0f, 0.0f, 0.0f};
			wheels_SetRef(emptyRef);
			wheels_Update();
			return;
		}

		// State estimation
		stateInfo.visionAvailable = receivedData.visionAvailable;
		stateInfo.visionYaw = receivedData.visionYaw; // TODO check if this is scaled properly with the new REM messages
		for (wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++) {
			stateInfo.wheelSpeeds[wheel] = wheels_GetState()[wheel];
		}

		stateInfo.xsensAcc[body_x] = MTi->acc[body_x];
		stateInfo.xsensAcc[body_y] = MTi->acc[body_y];
		stateInfo.xsensYaw = (MTi->angles[2]*M_PI/180); //Gradients to Radians
		stateInfo.rateOfTurn = MTi->gyr[2];
		stateEstimation_Update(&stateInfo);

		// State control
		stateControl_SetState(stateEstimation_GetState());
		stateControl_Update();

		wheels_SetRef(stateControl_GetWheelRef());
		wheels_Update();


		///////////////// OLD CODE. Revert to this if stuff seems to be going wrong
		// if (xsens_CalibrationDone) {	// don't do control until xsens calibration is done
		// 	if (!test_isTestRunning()) {
				
		// 		// State estimation
		// 		stateInfo.visionAvailable = receivedData.visionAvailable;
		// 		stateInfo.visionYaw = receivedData.visionYaw; // TODO check if this is scaled properly with the new REM messages
		// 		for (wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++) {
		// 			stateInfo.wheelSpeeds[wheel] = wheels_GetState()[wheel];
		// 		}

		// 		stateInfo.xsensAcc[body_x] = MTi->acc[body_x];
		// 		stateInfo.xsensAcc[body_y] = MTi->acc[body_y];
		// 		stateInfo.xsensYaw = (MTi->angles[2]*M_PI/180); //Gradients to Radians
		// 		stateInfo.rateOfTurn = MTi->gyr[2];
		// 		stateEstimation_Update(&stateInfo);

		// 		// State control
		// 		stateControl_SetState(stateEstimation_GetState());
		// 		stateControl_Update();

		// 		if (halt){// || !yaw_hasCalibratedOnce()) {
		// 			float emptyRef[4] = {0.0f, 0.0f, 0.0f, 0.0f};
		// 			wheels_SetRef(emptyRef);
		// 		}
		// 		else {
		// 			// Wheel control
		// 			wheels_SetRef(stateControl_GetWheelRef());
		// 		}
				
		// 		// wheels_SetRef(stateControl_GetWheelRef());
		// 	}

		// 	static int wirelessCounter = 0;
		// 	if (!checkWirelessConnection() && wirelessCounter > 1.25/TIME_DIFF && !test_isTestRunning()){
		// 		yaw_ResetCalibration();
		// 		sprintf(logBuffer, "cal triggered\n");
		// 		Putty_printf("cal triggered\n");
		// 	} else if (!checkWirelessConnection()){
		// 		wheels_Update();
		// 		wirelessCounter += 1;
		// 	} else {
		// 		wirelessCounter = 0;
		// 		wheels_Update();
		// 	}
		// }
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
