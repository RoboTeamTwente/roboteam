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
#include "SX1280_Constants.h"

#include "rem.h"

#include "REM_RobotCommand.h"
#include "REM_RobotFeedback.h"
#include "REM_RobotBuzzer.h"
#include "REM_RobotStateInfo.h"
#include "REM_RobotGetPIDGains.h"
#include "REM_RobotPIDGains.h"
#include "REM_SX1280Filler.h"

#include "time.h"
#include <unistd.h>
#include <stdio.h>

#define NO_ROTATION_TIME 6 				// time [s] the robot will halt at startup to let the xsens calibrate
#define XSENS_FILTER XFP_VRU_general 	// filter mode that will be used by the xsens

static bool SEND_ROBOT_STATE_INFO = false;
static const bool USE_PUTTY = false;

MTi_data* MTi;

REM_RobotCommandPayload robotCommandPayload = {0};
REM_RobotBuzzerPayload robotBuzzerPayload = {0};
REM_RobotFeedback robotFeedback = {0};
REM_RobotFeedbackPayload robotFeedbackPayload = {0};
REM_RobotStateInfo robotStateInfo = {0};
REM_RobotStateInfoPayload robotStateInfoPayload = {0};
REM_RobotPIDGains robotPIDGains = {0};

REM_RobotCommand activeRobotCommand = {0};
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
volatile uint32_t counter_RobotCommand = 0;
volatile uint32_t counter_RobotBuzzer = 0;
uint32_t timestamp_initialized = 0;

bool flagSendPIDGains = false;
bool is_connected_serial = false;
bool is_connected_wireless = false;
uint8_t last_valid_RSSI = 0;
uint32_t time_last_packet_serial = 0;
uint32_t time_last_packet_wireless = 0;

uint32_t heartbeat_17ms_counter = 0;
uint32_t heartbeat_17ms = 0;
uint32_t heartbeat_100ms = 0;
uint32_t heartbeat_1000ms = 0;

/* SX data */
// TODO: Maybe move all configs to its own file? (basestation_config.c/h???)
extern SX1280_Settings SX1280_DEFAULT_SETTINGS;
static Wireless SX1280 = {0};
static Wireless* SX = &SX1280;
static uint8_t SX_TX_buffer[MAX_PAYLOAD_SIZE + 3] __attribute__((aligned(4))) = {0};
static uint8_t SX_RX_buffer[MAX_PAYLOAD_SIZE + 3] __attribute__((aligned(4))) = {0};

static volatile Wireless_Packet txPacket;
static volatile Wireless_Packet rxPacket;

// The pins cannot be set at this point as they are not "const" enough for the compiler, so set them in the init
SX1280_Interface SX_Interface = {.SPI= COMM_SPI, .TXbuf= SX_TX_buffer, .RXbuf= SX_RX_buffer /*, .logger=LOG_printf*/,};

void Wireless_Writepacket_Cplt(void){
	if(TransmitPacket(SX) != WIRELESS_OK)
		LOG("[Wireless_Writepacket_Cplt] TransmitPacket error!\n");
}

void Wireless_Readpacket_Cplt(void){
	toggle_Pin(LED6_pin);
	time_last_packet_wireless = HAL_GetTick();
	handlePacket(rxPacket.message,rxPacket.payloadLength);

	/* TODO all this encoding stuff is done not only when a packet has been received from the basestation RX_DONE (which is intended)
	* It is also triggered when a packet has been sent back to the basestation TX_DONE (so basically this is done double)
	* And it's also done on RX_TIMEOUT (every 250ms, so not that bad)
	* Somehow, make sure that this is only done on RX_DONE
	*/
	txPacket.payloadLength = 0;

	encodeREM_RobotFeedback( (REM_RobotFeedbackPayload*) (txPacket.message + txPacket.payloadLength), &robotFeedback);
	txPacket.payloadLength += PACKET_SIZE_REM_ROBOT_FEEDBACK;

	if(SEND_ROBOT_STATE_INFO){
		encodeREM_RobotStateInfo( (REM_RobotStateInfoPayload*) (txPacket.message + txPacket.payloadLength), &robotStateInfo);
		txPacket.payloadLength += PACKET_SIZE_REM_ROBOT_STATE_INFO;
	}

	// TODO ensure this is only done when a packet is actually being sent
	// Both the RX_TIMEOUT and TX_DONE reset the flagSendPIDGains, and then the data isn't actually being sent
	// Maybe wait for Cas his rerwite? For now just always send PID values. There is space left in the packet
	// if(flagSendPIDGains){
		encodeREM_RobotPIDGains( (REM_RobotPIDGainsPayload*) (txPacket.message + txPacket.payloadLength), &robotPIDGains);
		txPacket.payloadLength += PACKET_SIZE_REM_ROBOT_PIDGAINS;
		flagSendPIDGains = false;
	// }

	// TODO insert REM_SX1280Filler packet if total_packet_length < 6. Fine for now since feedback is already more than 6 bytes
	WritePacket_DMA(SX, &txPacket, &Wireless_Writepacket_Cplt);
};

void Wireless_Default(){
	WaitForPacket(SX);
}

void Wireless_RXDone(SX1280_Packet_Status *status){
  /* It is possible that random noise can trigger the syncword.
   * Correct syncword from noise have a very weak signal.
   * Threshold is at -160/2 = -80 dBm. */
  if (status->RSSISync < 160) {
    ReadPacket_DMA(SX, &rxPacket, &Wireless_Readpacket_Cplt);
	last_valid_RSSI = status->RSSISync;
  }else{
	WaitForPacket(SX);
  }
}

Wireless_IRQcallbacks SX_IRQcallbacks = { .rxdone = &Wireless_RXDone, .default_callback = &Wireless_Default };

// Wireless_IRQcallbacks SX_IRQcallbacks = {0};

void executeCommands(REM_RobotCommand* robotCommand){
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



void resetRobotCommand(REM_RobotCommand* robotCommand){
	memset(robotCommand, 0, sizeof(REM_RobotCommand));
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

void printRobotCommand(REM_RobotCommand* rc){
	LOG_printf("======== RobotCommand ========\r\n");
	LOG_printf("            id : %d\n", rc->id);
	LOG_printf("        doKick : %d\n", rc->doKick);
	LOG_printf("        doChip : %d\r\n", rc->doChip);
	LOG_printf("       doForce : %d\r\n", rc->doForce);
	LOG_printf("useCameraAngle : %d\r\n", rc->useCameraAngle);
	LOG_printf("           rho : %.4f\r\n", rc->rho);
	LOG_printf("         theta : %.4f\r\n", rc->theta);
	LOG_printf("         angle : %.4f\r\n", rc->angle);
	LOG_printf("   cameraAngle : %.4f\r\n", rc->cameraAngle);
	LOG_printf("      dribbler : %d\r\n", rc->dribbler);
	LOG_printf(" kickChipPower : %d\r\n", rc->kickChipPower);
	LOG_printf("angularControl : %d\r\n", rc->angularControl);
	LOG_printf("      feedback : %d\r\n", rc->feedback);
}



// ----------------------------------------------------- INIT -----------------------------------------------------
void init(void){

	/* Enable the watchdog timer and set the threshold at 5 seconds. It should not be needed in the initialization but
	 sometimes for some reason the code keeps hanging when powering up the robot using the power switch. It's not nice
	 but its better than suddenly having non-responding robots in a match */
	IWDG_Init(iwdg, 5000);
	
	// Turn off all leds. Use leds to indicate init() progress
	set_Pin(LED0_pin, 0); set_Pin(LED1_pin, 0); set_Pin(LED2_pin, 0); set_Pin(LED3_pin, 0); set_Pin(LED4_pin, 0); set_Pin(LED5_pin, 0); set_Pin(LED6_pin, 0);
 
	/* Read ID from switches */
	ROBOT_ID = get_Id();
	set_Pin(LED0_pin, 1);

	LOG_init();

	LOG("[init:"STRINGIZE(__LINE__)"] Last programmed on " __DATE__ "\n");
	LOG("[init:"STRINGIZE(__LINE__)"] GIT: " STRINGIZE(__GIT_STRING__) "\n");
	LOG_printf("[init:"STRINGIZE(__LINE__)"] LOCAL_REM_VERSION: %d\n", LOCAL_REM_VERSION);
	LOG_printf("[init:"STRINGIZE(__LINE__)"] ROBOT_ID: %d\n", ROBOT_ID);
	LOG_sendAll();
	
	/* Read jumper */
	SEND_ROBOT_STATE_INFO = !read_Pin(FT0_pin);

	/* Initialize buzzer */
	buzzer_Init();
	buzzer_Play_QuickBeepUp();
	HAL_Delay(300);
	set_Pin(LED1_pin, 1);

	/* Play a warning sound if the robot is not programmed with the development branch */
	if(!__GIT_DEVELOPMENT__){
		buzzer_Play_WarningGit();
		HAL_Delay(300);
	}

	/* === Wired communication with robot; Either REM to send RobotCommands, or Putty for interactive terminal */
	if(USE_PUTTY){
		/* Initialize Putty. Not possible when REM_UARTinit() is called */
		Putty_Init();
	}else{
		/* Initialize Roboteam_Embedded_Messages. Not possible when Putty_Init() is called */
		/* Can now receive RobotCommands (and other packets) via UART */
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
    if(ballSensor_Init()) LOG("[init:"STRINGIZE(__LINE__)"] Ballsensor initialized\n");
    set_Pin(LED3_pin, 1);

	/* Initialize the SX1280 wireless chip */
	SX1280_Settings set = SX1280_DEFAULT_SETTINGS;
	set.periodBaseCount = WIRELESS_RX_COUNT;
	Wireless_Error err;
	SX_Interface.BusyPin = SX_BUSY_pin;
	SX_Interface.CS = SX_NSS_pin;
	SX_Interface.Reset = SX_RST_pin;
	// err |= Wireless_setPrint_Callback(SX, LOG_printf);
    err = Wireless_Init(SX, set, &SX_Interface);
    if(err != WIRELESS_OK){ LOG("[init:"STRINGIZE(__LINE__)"] SX1280 error\n"); LOG_sendAll(); while(1); }
	err = Wireless_setIRQ_Callbacks(SX,&SX_IRQcallbacks);
    if(err != WIRELESS_OK){ LOG("[init:"STRINGIZE(__LINE__)"] SX1280 error\n"); LOG_sendAll(); while(1); }
	LOG_sendAll();
    
	if(read_Pin(FT1_pin)){
		Wireless_setChannel(SX, YELLOW_CHANNEL);
		LOG("[init:"STRINGIZE(__LINE__)"] BLUE CHANNEL\n");
	}else{
		Wireless_setChannel(SX, YELLOW_CHANNEL);
		LOG("[init:"STRINGIZE(__LINE__)"] YELLOW CHANNEL\n");
	}
	LOG_sendAll();
    
	Wireless_setTXSyncword(SX,robot_syncWord[16]);
	uint32_t syncwords[2] = {robot_syncWord[ROBOT_ID],0};
	Wireless_setRXSyncwords(SX, syncwords);
	WaitForPacket(SX);
	set_Pin(LED4_pin, 1);

	/* Initialize the XSens chip */
	LOG("[init:"STRINGIZE(__LINE__)"] Initializing XSens\n");
    MTi = MTi_Init(NO_ROTATION_TIME, XSENS_FILTER);
    if(MTi == NULL){
		LOG("[init:"STRINGIZE(__LINE__)"] Failed to initialize XSens\n");
		buzzer_Play_WarningOne();
		HAL_Delay(1500);
	}
	
	set_Pin(LED5_pin, 1);

	LOG("[init:"STRINGIZE(__LINE__)"] Initialized\n");
	LOG_sendAll();

	/* Reset the watchdog timer and set the threshold at 200ms */
	IWDG_Refresh(iwdg);
	IWDG_Init(iwdg, 200);

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
	LOG_send();
	
	// Play a warning if a REM packet with an incorrect version was received
	if(!REM_last_packet_had_correct_version)
		if(!buzzer_IsPlaying())
			buzzer_Play_WarningTwo();

	// If serial packet is no older than 250ms, assume connected via wire
	is_connected_serial = (currentTime - time_last_packet_serial) < 250;
	is_connected_wireless = (currentTime - time_last_packet_wireless) < 250;
    // Refresh Watchdog timer
    IWDG_Refresh(iwdg);
    Putty_Callback();

	// Check XSens
    xsens_CalibrationDone = (MTi->statusword & (0x18)) == 0; // if bits 3 and 4 of status word are zero, calibration is done
    halt = !xsens_CalibrationDone || !(is_connected_wireless || is_connected_serial) || !REM_last_packet_had_correct_version;
    if (halt) {
		// LOG_printf("HALT %d %d %d\n", xsens_CalibrationDone, checkWirelessConnection(), isSerialConnected);
		// toggle_Pin(LED5_pin);
        stateControl_ResetAngleI();
        resetRobotCommand(&activeRobotCommand);
		// Quick fix to also stop the dribbler from rotating when the command is reset
		// TODO maybe move executeCommand to TIMER_7?
		dribbler_SetSpeed(0);
		REM_last_packet_had_correct_version = true;
    }

    // Unbrake wheels when Xsens calibration is done
    if (xsens_CalibrationDoneFirst && xsens_CalibrationDone) {
        xsens_CalibrationDoneFirst = false;
        wheels_Unbrake();
		LOG_printf("[loop:"STRINGIZE(__LINE__)"] XSens calibrated after %dms\n", currentTime-timestamp_initialized);
    }

    // Update test (if active)
    test_Update();
    
    // Go through all commands
    if (!halt) {
        executeCommands(&activeRobotCommand);
    }

    // Create RobotFeedback
	robotFeedback.header = PACKET_TYPE_REM_ROBOT_FEEDBACK;
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
    robotFeedback.rssi = last_valid_RSSI; // Should be divided by two to get dBm but RSSI is 8 bits so just send all 8 bits back
    
	if(SEND_ROBOT_STATE_INFO){
		robotStateInfo.header = PACKET_TYPE_REM_ROBOT_STATE_INFO;
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
	{
		robotPIDGains.header = PACKET_TYPE_REM_ROBOT_PIDGAINS;
		robotPIDGains.remVersion = LOCAL_REM_VERSION;
		robotPIDGains.id = ROBOT_ID;
		robotPIDGains.PbodyX = 2;
		robotPIDGains.IbodyX = 0;
		robotPIDGains.DbodyX = 1;
		robotPIDGains.PbodyY = 2;
		robotPIDGains.IbodyY = 0;
		robotPIDGains.DbodyY = 1;
		robotPIDGains.PbodyYaw = 2;
		robotPIDGains.IbodyYaw = 0;
		robotPIDGains.DbodyYaw = 1;
		robotPIDGains.Pwheels = 0;
		robotPIDGains.Iwheels = 0;
		robotPIDGains.Dwheels = 0;
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

		// encodeREM_RobotFeedback( &robotFeedbackPayload, &robotFeedback );
		// HAL_UART_Transmit(UART_PC, robotFeedbackPayload.payload, PACKET_SIZE_REM_ROBOT_FEEDBACK, 10);

		// encodeREM_RobotStateInfo( &robotStateInfoPayload, &robotStateInfo);
		// HAL_UART_Transmit(UART_PC, robotStateInfoPayload.payload, PACKET_SIZE_REM_ROBOT_STATE_INFO, 10);

	}

	// Heartbeat every 1000ms
	if(heartbeat_1000ms < HAL_GetTick()){
		uint32_t now = HAL_GetTick();
		while (heartbeat_1000ms < now) heartbeat_1000ms += 1000;

        // Toggle liveliness LED
        toggle_Pin(LED0_pin);

		// switch(SX->state){
		// case WIRELESS_RESET: LOG_printf("%d: state=WIRELESS_RESET\n", now); break;
		// case WIRELESS_INIT: LOG_printf("%d: state=WIRELESS_INIT\n", now); break;
		// case WIRELESS_READY: LOG_printf("%d: state=WIRELESS_READY\n", now); break;
		// case WIRELESS_WRITING: LOG_printf("%d: state=WIRELESS_WRITING\n", now); break;
		// case WIRELESS_READING: LOG_printf("%d: state=WIRELESS_READING\n", now); break;
		// case WIRELESS_TRANSMITTING: LOG_printf("%d: state=WIRELESS_TRANSMITTING\n", now); break;
		// case WIRELESS_RECEIVING: LOG_printf("%d: state=WIRELESS_RECEIVING\n", now); break;
		// default: LOG_printf("%d: default\n", now); break;
		// }

		// Check if ballsensor connection is still correct
        if ( !ballSensor_isInitialized() ) {
            ballSensor_Init();
            __HAL_I2C_DISABLE(BS_I2C);
            HAL_Delay(1);
            __HAL_I2C_ENABLE(BS_I2C);
        }
    }

    /* LEDs for debugging */
    // LED0 : toggled every second while alive
    set_Pin(LED1_pin, !xsens_CalibrationDone);		// On while xsens startup calibration is not finished
    set_Pin(LED2_pin, wheels_GetWheelsBraking());   // On when braking 
    set_Pin(LED3_pin, halt);						// On when halting
    set_Pin(LED4_pin, ballPosition.canKickBall);    // On when ballsensor says ball is within kicking range
	// LED5 unused
    // LED6 Wireless_Readpacket_Cplt : toggled when a packet is received
}

void handleRobotCommand(uint8_t* packet_buffer){
	memcpy(robotCommandPayload.payload, packet_buffer, PACKET_SIZE_REM_ROBOT_COMMAND);
	REM_last_packet_had_correct_version &= REM_RobotCommand_get_remVersion(&robotCommandPayload) == LOCAL_REM_VERSION;
	decodeREM_RobotCommand(&activeRobotCommand,&robotCommandPayload);
}

void handleRobotBuzzer(uint8_t* packet_buffer){
	REM_RobotBuzzerPayload* rbp = (REM_RobotBuzzerPayload*) (packet_buffer);
	REM_last_packet_had_correct_version &= REM_RobotBuzzer_get_remVersion(rbp) == LOCAL_REM_VERSION;
	uint16_t period = REM_RobotBuzzer_get_period(rbp);
	float duration = REM_RobotBuzzer_get_duration(rbp);
	buzzer_Play_note(period, duration);
}

void handleRobotGetPIDGains(uint8_t* packet_buffer){
	REM_RobotGetPIDGainsPayload* rgpidgp = (REM_RobotGetPIDGainsPayload*) (packet_buffer);
	REM_last_packet_had_correct_version &= REM_RobotGetPIDGains_get_remVersion(rgpidgp) == LOCAL_REM_VERSION;
	flagSendPIDGains = true;
}

void robot_setRobotCommandPayload(REM_RobotCommandPayload* rcp){
	decodeREM_RobotCommand(&activeRobotCommand, rcp);
	time_last_packet_serial = HAL_GetTick();
}

bool handlePacket(uint8_t* packet_buffer, uint8_t packet_length){
	uint8_t total_bytes_processed = 0;
	uint8_t packet_header;

	while(total_bytes_processed < packet_length){

		packet_header = packet_buffer[total_bytes_processed];

		switch(packet_header){

			case PACKET_TYPE_REM_ROBOT_COMMAND:
				handleRobotCommand(packet_buffer + total_bytes_processed);
				total_bytes_processed += PACKET_SIZE_REM_ROBOT_COMMAND;
				break;

			case PACKET_TYPE_REM_ROBOT_BUZZER: 
				handleRobotBuzzer(packet_buffer + total_bytes_processed);
				total_bytes_processed += PACKET_SIZE_REM_ROBOT_BUZZER;
				break;
			
			case PACKET_TYPE_REM_ROBOT_GET_PIDGAINS:
				handleRobotGetPIDGains(packet_buffer + total_bytes_processed);
				total_bytes_processed += PACKET_SIZE_REM_ROBOT_GET_PIDGAINS;
				break;

			case PACKET_TYPE_REM_SX1280FILLER:
				total_bytes_processed += PACKET_SIZE_REM_SX1280FILLER;
				break;

			default:
				LOG_printf("[SPI_TxRxCplt] Error! At %d of %d bytes. [@] = %d\n", total_bytes_processed, packet_length, packet_header);
				return false;
		}
	}

	return true;
}

// ----------------------------------------------------- STM HAL CALLBACKS -----------------------------------------------------
/* HAL_SPI_TxRxCpltCallback = Callback for either SPI Transmit or Receive complete */
/* This function is triggered after calling HAL_SPI_TransmitReceive_IT */
/* Since we transmit everything using blocking mode, this function should only be called when we receive something */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi){

	// If we received data from the SX1280
	if(hspi->Instance == SX->Interface->SPI->Instance) {
		Wireless_DMA_Handler(SX);

	}

	// If we received data from the XSens
	else if(/*MTi != NULL &&*/ hspi->Instance == MTi->SPI->Instance){
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
		Wireless_IRQ_Handler(SX);
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

		// if(MTi == NULL) return;

		// State estimation		
		stateInfo.visionAvailable = activeRobotCommand.useCameraAngle;
		stateInfo.visionYaw = activeRobotCommand.cameraAngle; // TODO check if this is scaled properly with the new REM messages
		
		wheels_GetMeasuredSpeeds(stateInfo.wheelSpeeds);
		stateInfo.xsensAcc[body_x] = MTi->acc[body_x];
		stateInfo.xsensAcc[body_y] = MTi->acc[body_y];
		stateInfo.xsensYaw = (MTi->angles[2]*M_PI/180); //Gradients to Radians
		stateInfo.rateOfTurn = MTi->gyr[2];
		stateEstimation_Update(&stateInfo);



		if(test_isTestRunning(wheels) || test_isTestRunning(normal)) {
            wheels_Update();
            return;
        }

		if( halt && !test_isTestRunning(square)){
			wheels_Stop();
			return;
		}

		// // State estimation
		// stateInfo.visionAvailable = activeRobotCommand.useCameraAngle;
		// stateInfo.visionYaw = activeRobotCommand.cameraAngle; // TODO check if this is scaled properly with the new REM messages
		
		// wheels_GetMeasuredSpeeds(stateInfo.wheelSpeeds);
		// stateInfo.xsensAcc[body_x] = MTi->acc[body_x];
		// stateInfo.xsensAcc[body_y] = MTi->acc[body_y];
		// stateInfo.xsensYaw = (MTi->angles[2]*M_PI/180); //Gradients to Radians
		// stateInfo.rateOfTurn = MTi->gyr[2];
		// stateEstimation_Update(&stateInfo);

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
