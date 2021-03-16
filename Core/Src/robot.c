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

#include "RobotFeedback.h"

#include "time.h"
#include <unistd.h>
#include <stdio.h>

#define NO_ROTATION_TIME 6 				// time [s] the robot will halt at startup to let the xsens calibrate
#define XSENS_FILTER XFP_VRU_general 	// filter mode that will be used by the xsens

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

uint16_t ID;

SX1280* SX;
MTi_data* MTi;
int counter = 0;
int strength = 0;


RobotFeedbackPayload Bot_to_PC;
RobotFeedback robotFeedback = {0};
RobotCommandPayload PC_to_Bot;
RobotCommand robotCommand = {0};
ReceivedData receivedData = {{0.0}, false, 0.0f, 0, 0, false, false};



StateInfo stateInfo = {0.0f, false, {0.0}, 0.0f, 0.0f, {0.0}};
bool halt = true;
bool xsens_CalibrationDone = false;
bool xsens_CalibrationDoneFirst = true;

IWDG_Handle* iwdg;


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

    // Check if robot has 30 W or 50 W motors (jumper = 50 W, no jumper = 30 W)
    MOTORS_50W = true;//read_Pin(IN1_pin);
    // TODO: remove this bool, it should not be here or used

	ID = get_Id();

	buzzer_Init();
	buzzer_Play_ID(ID);
	HAL_Delay(1500);

	Putty_Init();

	Putty_printf("ID: %d\n", ID);

    // Initialize control constants
    control_util_Init();

    wheels_Init();
    stateControl_Init();
    stateEstimation_Init();
    shoot_Init();
    dribbler_Init();
    ballSensor_Init();
    
    
	
    robotCommandIsFresh = 0;
    REM_UARTinit(UART_PC);

	Putty_printf("Initializing wireless\n");
    SX = Wireless_Init(COMMAND_CHANNEL, COMM_SPI);
	Putty_printf("Initializing XSens\n");
    MTi = MTi_Init(NO_ROTATION_TIME, XSENS_FILTER);
    if(MTi == NULL){
		Putty_printf("Failed to initialize XSens\n");
		buzzer_Play_WarningOne();
		HAL_Delay(1500);
	}
    
	// start the wireless receiver
    // transmit feedback packet for every received packet if wirelessFeedback==true
    SX->SX_settings->syncWords[0] = robot_syncWord[ID];
    setSyncWords(SX, SX->SX_settings->syncWords[0], 0x00, 0x00);
    setRX(SX, SX->SX_settings->periodBase, WIRELESS_RX_COUNT);

	Putty_printf("Wireless initialized\n");

	buzzer_Play_QuickBeepUp();
    IWDG_Init(iwdg); // Initialize watchdog (resets system after it has crashed)

}

// ----------------------------------------------------- MAIN LOOP -----------------------------------------------------

bool isSerialConnected = false;
uint32_t timeLastPacket = 0;

volatile int commandCounter = 0;
void loop(void){

	if(0 < strlen(logBuffer)){
		HAL_UART_Transmit(UART_PC, (uint8_t*) logBuffer, strlen(logBuffer), 10);
		logBuffer[0] = '\0';
	}
	
	uint32_t currentTime = HAL_GetTick();

	if(robotCommandIsFresh == 1){
		robotCommandIsFresh = 0;
		timeLastPacket = currentTime;
		packetToRoboData(&myRobotCommandPayload, &receivedData);
		
		toggle_Pin(LED6_pin);
		
		printRobotCommand(&myRobotCommand);
		// printReceivedData(&receivedData);	
	}

	// If serial packet is no older than 250ms
	isSerialConnected = (currentTime - timeLastPacket) < 250;
	

	// if(robotCommandIsFresh_wireless == 1){
	// 	robotCommandIsFresh_wireless = 0;
	// 	decodeRobotCommand(&myRobotCommand, &PC_to_Bot);
	// 	printRobotCommand(&myRobotCommand);
	// 	printReceivedData(&receivedData);
		
	// 	for(int i = 0; i < PACKET_SIZE_ROBOT_COMMAND; i++){
	// 		Putty_printf("%d\t "BYTE_TO_BINARY_PATTERN" %d\r\n", i, BYTE_TO_BINARY(PC_to_Bot.payload[i]), PC_to_Bot.payload[i]);
	// 	}
	// }

    /*
    * Check for empty battery
    */
    static int batCounter = 0;
    if (read_Pin(Bat_pin) && batCounter > 1000){
        Putty_printf("battery empty\n\r");
        buzzer_Play_ImperialMarch();
        set_Pin(LED5_pin, 1);
        Putty_DeInit();
        wheels_DeInit();
        stateControl_DeInit();
        stateEstimation_DeInit();
        shoot_DeInit();
        dribbler_DeInit();
        ballSensor_DeInit();
        buzzer_DeInit();
        MTi_DeInit(MTi);
        Wireless_DeInit();
    }else if (read_Pin(Bat_pin)) {
        batCounter += 1;
    } else {
        batCounter = 0;
    }

    // Refresh Watchdog timer
    IWDG_Refresh(iwdg);
    Putty_Callback();

    // Check for new ballsensor data
    if (read_Pin(BS_IRQ_pin)){
        ballSensor_IRQ_Handler();
    }

    /*
    * Check for wireless data
    */
    xsens_CalibrationDone = (MTi->statusword & (0x18)) == 0; // if bits 3 and 4 of status word are zero, calibration is done
    halt = !(xsens_CalibrationDone && (checkWirelessConnection() || isSerialConnected));
    if (halt) {
        stateControl_ResetAngleI();
        clearReceivedData(&receivedData);
    }

    /*
    * Unbrake wheels when Xsens calibration is done
    */
    if (xsens_CalibrationDoneFirst && xsens_CalibrationDone) {
        xsens_CalibrationDoneFirst = false;
        wheels_Brake(false);
    }

    // Update test (if active)
    test_Update(&receivedData);
    
    // Go through all commands
    executeCommands(&receivedData);

    /*
    * Feedback
    */
	robotFeedback.header = PACKET_TYPE_ROBOT_FEEDBACK;
    robotFeedback.id = ID;
    robotFeedback.XsensCalibrated = xsens_CalibrationDone;
    robotFeedback.batteryLevel = (batCounter > 1000);
    robotFeedback.ballSensorWorking = ballSensor_isWorking();
    robotFeedback.hasBall = ballPosition.canKickBall;
    robotFeedback.ballPos = ballPosition.x/100 & ballSensor_isWorking();

    float vx = stateEstimation_GetState()[body_x];
    float vy = stateEstimation_GetState()[body_y];
    robotFeedback.rho = sqrt(vx*vx + vy*vy);
    robotFeedback.angle = stateEstimation_GetState()[body_w];
    robotFeedback.theta = atan2(vy, vx) / 0.0062; // range is [-512, 511] instead of [-1024, 1023]
    robotFeedback.wheelBraking = wheels_IsBraking(); // TODO Locked feedback has to be changed to brake feedback in PC code
    robotFeedback.rssi = SX->Packet_status->RSSISync/2;
    
    
    static int printTime = 0;
    if (HAL_GetTick() > printTime + 1000) {
        printTime = HAL_GetTick();
        // Toggle liveliness LED
        toggle_Pin(LED0_pin);

        // Check if ballsesnor connection is still correct
        // TODO: check if statemachine is correct (!ballPosition.canSeeBall)
        if ((!ballSensorInitialized && init_attempts < 5)) {
            init_attempts++;
            ballSensor_Init();
            __HAL_I2C_DISABLE(BS_I2C);
            HAL_Delay(1);
            __HAL_I2C_ENABLE(BS_I2C);
        } else if (init_attempts == 5) {
            init_attempts++;
            Putty_printf("too many BS_INIT attempts. Quit!\n\r");
            // buzzer_Play_PowerUp();
        } else if (ballSensorInitialized) {
            init_attempts = 0;
        }
//		  printBaseStationData();
//		  printReceivedData(&receivedData);
//		  printRobotStateData();
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
    set_Pin(LED5_pin, (read_Pin(Bat_pin) && batCounter > 1000));
    // LED6 done in Wireless.c
}


// ----------------------------------------------------- STM HAL CALLBACKS -----------------------------------------------------
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
	if(hspi->Instance == SX->SPI->Instance) {
		Wireless_DMA_Handler(SX, PC_to_Bot.payload);
        packetToRoboData(&PC_to_Bot, &receivedData);
		robotCommandIsFresh_wireless = 1;
		commandCounter++;
		strength+= SX->Packet_status->RSSISync;
	}
	else if(hspi->Instance == MTi->SPI->Instance){
		MTi_SPI_RxCpltCallback(MTi);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == UART_PC->Instance){
		// Putty_UARTCallback(huart);
		REM_UARTCallback(huart);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == SX_IRQ_pin.PIN) {
        encodeRobotFeedback(&Bot_to_PC, &robotFeedback);
		Wireless_IRQ_Handler(SX, Bot_to_PC.payload, PACKET_SIZE_ROBOT_FEEDBACK);
	}else if(GPIO_Pin == MTi_IRQ_pin.PIN){
		MTi_IRQ_Handler(MTi);
	}else if (GPIO_Pin == BS_IRQ_pin.PIN){
		// TODO: make this work and use instead of the thing in the while loop
		ballSensor_IRQ_Handler();
	}
}

volatile uint32_t timcnt = 0;

// Handles the interrupts of the different timers.
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{	
	// Old Geneva timer. Needs to be properly disabled in CubeMX
	if(htim->Instance == htim6.Instance){

	}
	else if(htim->Instance == htim7.Instance) {
		timcnt += 2;
		

		if (xsens_CalibrationDone) {	// don't do control until xsens calibration is done
			if (!test_isTestRunning()) {
				
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

				if (halt){// || !yaw_hasCalibratedOnce()) {
					// if(10000 < timcnt) Putty_printf("[Tim] empty\n");
					float emptyRef[4] = {0.0f, 0.0f, 0.0f, 0.0f};
					wheels_SetRef(emptyRef);
				}
				else {
					// Wheel control
					wheels_SetRef(stateControl_GetWheelRef());
				}
				
				// wheels_SetRef(stateControl_GetWheelRef());
			}
			static int wirelessCounter = 0;
			if (!checkWirelessConnection() && wirelessCounter > 1.25/TIME_DIFF && !test_isTestRunning()){
				yaw_ResetCalibration();
			} else if (!checkWirelessConnection()){
				wheels_Update();
				wirelessCounter += 1;
			} else {
				wirelessCounter = 0;
				wheels_Update();
			}
		}

		if(300 < timcnt){
			// sprintf(logBuffer, "[Tim]\n");
			// Putty_printf("[Tim] cc = %d\n", commandCounter);
			timcnt = 0;
		}
	}
	else if (htim->Instance == htim10.Instance) {
		buzzer_Callback();
	}
	else if (htim->Instance == htim11.Instance) {
		shoot_Callback();
	}
}
