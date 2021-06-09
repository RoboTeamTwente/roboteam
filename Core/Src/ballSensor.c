#include "ballSensor.h"
#include "limits.h"
#include "gpio_util.h"
#include "PuTTY.h"
#include "string.h"
#include "peripheral_util.h"
#include "main.h"

// 7.3.2 I2C : Read XX amount of bytes (number of bytes to read is indicated by second byte of first I2C Read Operation).

uint32_t error; // for i2c errors
uint8_t ball_debug = 0; // enable ball print statements
volatile uint8_t ballSensorInitialized = 0; // ball sensor initialization status
uint8_t next_message_length = 2; // default length of next message is 2 bytes
uint8_t init_attempts = 0;

// Important. Values above 127 are represented by two bytes, even though it fits in one.
// 7.4.1 Serialization Protocol Quick Start - Encoding Integers
// * If the integer is between 0 and 127, it is represented by one byte ( 00 to 7F ).
// * If the integer is between 128 and 32767, it is represented by two bytes ( 00 80 to 7F FF ).

/* ball sensor boot complete response. 7.3.2 I2C, Page 44 */
// 0xF0 0x11 = 11 1 10000 [PRIVATE 16] Notification 17 bytes
// 0x40, 0x02, 0x00, 0x00 = 01 0 00000 = [Application 0] DeviceAddress, 2 bytes, 0 0?
// 0x63, 0x0B = 01 1 00011 = [APPLICATION 3] BootCompleteNotification, 11 bytes
	// 0x80, 0x01, 0x00 = 10 0 00000 = [0] asicStatus, 1 byte, value 0 = asicExists
	// 0x81, 0x02, 0x03, 0x00 = 10 0 00001 = [1] resetSource, 2 bytes, 0000 0011 0000 0000
	// 0x82, 0x02, 0x00, 0x00 = 10 0 00010 = [2] globalState, 2 bytes, 0000 0000 0000 0000

uint8_t bootcomplete_response[] = {0xF0, 0x11, 0x40, 0x02, 0x00, 0x00, 0x63, 0x0B, 0x80, 0x01, 0x00, 0x81, 0x02, 0x03,
									0x00, 0x82, 0x02, 0x00, 0x00};

/* ball sensor configuration command, to be sent before enabling device */
/* page 82 */
/* minX 0, minY 0, maxX 700, maxY 440, minSize 0, reported&tracked touch 1, everything else off */
/* Device Configuration */
uint8_t config_command[] = {
	// ProtocolMessage request [PRIVATE 14] : 7:6=Private=11, 5=SEQUENCE=1, 4:0=14=01110 => 11101110 => 0xEE
	BS_REQUEST, 0x40, // ID for request message (0xEE), number of bytes (64)
	BS_REQUEST, 0x3E, // ID for request followed by length of total payload (0x40 = 62 bytes)
	BS_DEVICE_ADDRESS_AIR,
	// 0x40, 0x02, 0x02, 0x00, // Device address (always the same for the zForce AIR Touch Sensor)
	// Page 86. 0x40 = 0100 0000 => 7:6=01=Application, 5=0=primitive, 4:0=00000=tag=0
	//          0x02 = number of bytes that follow
	//          0x02 = Device type (Zforce Air), 0x00 = Device index (always 0 on Zforce Air)

	// deviceConfiguration [APPLICATION 19] SEQUENCE : 7:6=Application=01, 5=SEQUENCE=1, 4:0=19=10011 => 01110011 => 0x73
	BS_DEVICE_CONFIGURATION, 0x38, // ID for deviceConfiguration (0x73), number of bytes (56)
		// numberOfTrackedTouches [0] INTEGER (0..255) : 7:6=Contextspecific=10, 5=PRIMITIVE=0, 4:0=0=00000 => 10000000 => 0x80
		BS_NUMBEROFTRACKEDTOUCHES, 1, 1, // ID for numberOfTrackedTouches (0x80), number of bytes (1), value (1)
		// subTouchActiveArea [2] SEQUENCE : 7:6=Contextspecific=10, 5=SEQUENCE=1, 4:0=2=00010 => 10100010 => 0xA2
		BS_SUBTOUCHACTIVEAREA, 29, // ID for subTouchActiveArea (0xA2), number of bytes (29)
			// lowBoundX [0] INTEGER (0..16383) : 7:6=Contextspecific=10, 5=PRIMITIVE=0, 4:0=0=00000 => 10000000 => 0x80
			BS_LOWBOUNDX, 1, 0, // ID for lowBoundX (0x80), number of bytes (1), value (0)
			// lowBoundY [1] INTEGER (0..16383) : 7:6=Contextspecific=10, 5=PRIMITIVE=0, 4:0=1=00001 => 10000001 => 0x81
			BS_LOWBOUNDY, 1, 0, // ID for lowBoundY (0x81), number of bytes (1), value (0)
			// highBoundX [2] INTEGER (0..16383) : 7:6=Contextspecific=10, 5=PRIMITIVE=0, 4:0=2=00010 => 10000010 => 0x82
			BS_HIGHBOUNDX, 2, 700 >> 8, 700 & 0xFF, // ID for highBoundX (0x82) number of bytes (2), value (700)
			// highBoundY [3] INTEGER (0..16383) : 7:6=Contextspecific=10, 5=PRIMITIVE=0, 4:0=3=00011 => 10000011 => 0x83
			BS_HIGHBOUNDY, 2, 440 >> 8, 440 & 0xFF, // ID for highBoundY (0x83) number of bytes (2), value (440)
			// reverseX [4] BOOLEAN : 7:6=Contextspecific=10, 5=PRIMITIVE=0, 4:0=4=00100 => 10000100 = 0x84
			BS_REVERSEX, 1, 0, // ID for reverseX (0x84), number of bytes (1), value (0)
			// reverseY [5] BOOLEAN : 7:6=Contextspecific=10, 5=PRIMITIVE=0, 4:0=5=00101 => 10000101 = 0x85
			BS_REVERSEY, 1, 0, // ID for reverseY (0x85), number of bytes (1), value (0)
			// flipXY [6] BOOLEAN : 7:6=Contextspecific=10, 5=PRIMITIVE=0, 4:0=6=00110 => 10000110 = 0x86
			BS_FLIPXY, 1, 0, // ID for flipXY (0x86), number of bytes (1), value (0)
			// offsetX [7] BOOLEAN : 7:6=Contextspecific=10, 5=PRIMITIVE=0, 4:0=7=00111 => 10000111 = 0x87
			BS_OFFSETX, 1, 0, // ID for offsetX (0x87), number of bytes (1), value (0)
			// offsetY [8] BOOLEAN : 7:6=Contextspecific=10, 5=PRIMITIVE=0, 4:0=8=01000 => 10001000 = 0x88
			BS_OFFSETY, 1, 0, // ID for offsetY (0x88), number of bytes (1), value (0)
		// sizeRestriction [4] SEQUENCE : 7:6=Contextspecific=10, 5=SEQUENCE=1, 4:0=4=00100, => 10100100 => 0xA4
		BS_SIZERESTRICTION, 9, // ID for sizeRestriction (0xA4), number of bytes (9)
			// maxSizeEnabled [0] BOOLEAN : 7:6=Contextspecific=10, 5=PRIMITIVE=0, 4:0=0=00000 => 10000000 => 0x80
			BS_MAXSIZEENABLED, 1, 0, // ID for maxSizeEnabled (0x80), number of bytes (1), value (0)
			// minSizeEnabled [2] BOOLEAN : 7:6=Contextspecific=10, 5=PRIMITIVE=0, 4:0=2=00010 => 10000010 => 0x82
			BS_MINSIZEENABLED, 1, 0, // ID for minSizeEnabled (0x82), number of bytes (1), value (0)
			// minSize [3] INTEGER (0..32767) : 7:6=Contextspecific=10, 5=PRIMITIVE=0, 4:0=3=00011 => 10000011 => 0x83
			BS_MINSIZE, 1, 0, // ID for minSize (0x83), number of bytes (1), value (0)		
			// ?????????????????????????????????????????????????????????????????????????
			0x86, 0x01, 0x01, // ?????????????????????????????????????????????????????????????????????
			// TODO the fuck is 0x86?? The sizeRestriction only has 4 fields.
		
		// TODO figure out why we set this? Doesn't make any sense to set scaling to 0?
		// Page 82 : hidDisplaySize: Scaling the coordinate system when using the sensor module in HID Touch Digitizer mode.
		// hidDisplaySize [7] SEQUENCE : 7:6=Contextspecific=10, 5=SEQUENCE=1, 4:0=7=00111, => 10100111 => 0xA7
		0xA7, 0x06, // ID for hidDisplaySize (0xA7), number of bytes (6)
			// x [0] INTEGER (0..32767) : 7:6=Contextspecific=10, 5=PRIMITIVE=0, 4:0=0=00000 => 10000000 => 0x80
			0x80, 0x01, 0x00, // ID for x (0x80), number of bytes (1), value (0)
			// y [1] INTEGER (0..32767) : 7:6=Contextspecific=10, 5=PRIMITIVE=0, 4:0=1=00001 => 10000001 => 0x81
			0x81, 0x01, 0x00, // ID for y (0x81), number of bytes (1), value (0)
	};

// 0xEF, 0x43 = response [PRIVATE 15] Message
// 0x40, 0x02, 0x02, 0x00 = device address
// 0x73, 0x3D, device configuration
// 0xA2, 0x1D, subtouchactivearea
	// 0x80, 0x01, 0x24, lowBoundX
	// 0x81, 0x01, 0x00, lowBoundY
	// 0x82, 0x02, 0x02, 0xBC, highBoundX
	// 0x83, 0x02, 0x01, 0xB8, highBoundY
	// 0x84, 0x01, 0x00, reverseX
	// 0x85, 0x01, 0x00, reverseY
	// 0x86, 0x01, 0x00, flipXY
	// 0x87, 0x01, 0x00, offsetX
    // 0x88, 0x01, 0x00, offsetY
// 0xA4, 0x0C, sizeRestriction
	// 0x80, 0x01, 0x00, maxSizeEnabled
	// 0x81, 0x01, 0x00, maxSize
	// 0x82, 0x01, 0x00, minSizeEnabled
	// 0x83, 0x01, 0x00, minSize
	// 0x85, 0x01, 0x00, ????????
	// 0x86, 0x01, 0x01, ????????
// 0xA7, 0x08, hidDisplaySize
	// 0x80, 0x02, 0x02, 0x98, x
	// 0x81, 0x02, 0x01, 0xB8, y

uint8_t config_response[] = {BS_RESPONSE, 0x43, BS_DEVICE_ADDRESS_AIR, 0x73, 0x3D, 0xA2, 0x1D,
							0x80, 0x01, 0x24, 0x81, 0x01, 0x00, 0x82, 0x02, 0x02, 0xBC, 0x83, 0x02, 0x01,
							0xB8, 0x84, 0x01, 0x00, 0x85, 0x01, 0x00, 0x86, 0x01, 0x00, 0x87, 0x01, 0x00,
							0x88, 0x01, 0x00, 0xA4, 0x0C, 0x80, 0x01, 0x00, 0x81, 0x01, 0x00, 0x82, 0x01,
							0x00, 0x83, 0x01, 0x00, 0x85, 0x01, 0x00, 0x86, 0x01, 0x01, 0xA7, 0x08, 0x80,
							0x02, 0x02, 0x98, 0x81, 0x02, 0x01, 0xB8};

/* ball sensor frequency command */
/* set scanning frequency to 100Hz (0x64) & idle frequency to 100Hz */
//uint8_t freq_command[] = {0xEE, 0x0E, 0xEE, 0x0C, 0x40, 0x02, 0x02, 0x00, 0x68, 0x06, 0x80, 0x01, 0x64, 0x82, 0x01, 0x64};
//uint8_t freq_response[] = {0xEF, 0x0C, 0x40, 0x02, 0x00, 0x00, 0x68, 0x06, 0x80, 0x01, 0x64, 0x82, 0x01, 0x64};

// 0xEE => 11 1 01110 -> PRIVATE SEQUENCE 14, 0x10 => 16 bytes
// 0xEE => 11 1 01110 -> PRIVATE SEQUENCE 14, 0x0E => 14 bytes
	// 0x40 => 01 0 00000 -> [APPLICATION 0] PRIMITIVE = DeviceAddress, 0x02 => 2 bytes, 0x02 = zForce Air, 0x00 => 0
	// 0x68 => 01 1 01000 -> [APPLICATION 8] SEQUENCE = frequency, 0x08 => 8 bytes,
		// 0x80 => 10 0 00000 -> [0] = finger, 0x02 = 2 bytes, 0x01, 0x90 = 400
		// 0x82 => 10 0 00010 -> [2] = idle, 0x02 = 2 bytes, 0x01, 0x90 = 400
		    // TODO What the fuck 400Hz??? Why? Looks a little much to me tbh...


uint8_t freq_command[] = {BS_REQUEST, 14, BS_REQUEST, 12, BS_DEVICE_ADDRESS_AIR, BS_FREQUENCY, 6, BS_FINGER, 1, 100, BS_IDLE, 1, 100};
uint8_t freq_response[] = {BS_RESPONSE, 12, BS_DEVICE_ADDRESS_PLATFORM, BS_FREQUENCY, 6, BS_FINGER, 1, 100, BS_IDLE, 1, 100};

uint8_t get_touch_format[] = {BS_REQUEST, 8, BS_REQUEST, 6, BS_DEVICE_ADDRESS_AIR, BS_TOUCH_FORMAT, 0};

/* Ballsensor enable device command. 7.4.3 I2C Transport, Page 69 */
/* 7.4.1 Serialization Protocol Quick Start -> Enabling Touch Sensor Modules, page 50 */
uint8_t enable_command[] = 	{BS_REQUEST, 11, BS_REQUEST, 9, BS_DEVICE_ADDRESS_AIR, BS_ENABLE_SEQUENCE, 3, BS_ENABLE, 1, BS_ENABLE_CONTINUOUS};
uint8_t enable_response[] = {BS_RESPONSE, 9, BS_DEVICE_ADDRESS_AIR, BS_ENABLE_SEQUENCE, 3, BS_ENABLE, 1, BS_ENABLE_CONTINUOUS};

/* ball sensor receive data notification */
uint8_t measurement_rx[] = {BS_NOTIFICATION, 17, BS_DEVICE_ADDRESS_AIR, BS_TOUCH_NOTIFICATION_SEQUENCE, 11, BS_TOUCH_NOTIFICATION, 9};

// F0 11 BS_DEVICE_ADDRESS_AIR BS_TOUCH_NOTIFICATION_SEQUENCE 11   42 09   09 01 01 E0 00 F4 00 44 00

// BS_DEVICE_ADDRESS?

bool ballSensor_Init()
{
	Putty_printf("[ballsensor] init\n");
	
	ballSensor_NoBall();
	HAL_Delay(20); // timing specs
	ballSensor_Reset();
	next_message_length = 2;
	int currentTime = HAL_GetTick(); // avoid lockup when initializing
	while(!read_Pin(BS_IRQ_pin)){
		// wait for DR
		if (HAL_GetTick()-currentTime > 100) {
			Putty_printf("[ballsensor] timeout\n");
			return false;
		}
	}
	HAL_Delay(100); // timing specs

	// boot procedure, only check for bootcomplete response
	if (!bs_Boot()){
		Putty_printf("[ballsensor] boot failed\n");
		return false; // boot failed, leave
	}

	// config procedure (set&check)
	while(read_Pin(BS_IRQ_pin)); // do not proceeed to Tx unless DR is low
	if (!bs_SetConfig()){
		Putty_printf("[ballsensor] set config failed\n");
		return false; // set config failed, leave
	}
	currentTime = HAL_GetTick(); // avoid lockup when initializing
	while(!read_Pin(BS_IRQ_pin)  &&  (HAL_GetTick()-currentTime < 5)); // wait for DR
	if (!bs_CheckConfig()){
		Putty_printf("[ballsensor] check config failed\n");
		return false;
	}

	// freq procedure (set&check)
	while(read_Pin(BS_IRQ_pin)); // do not proceeed to Tx unless DR is low
	if (!bs_SetFreq()){
		return false; // set freq failed, leave
	}
	currentTime = HAL_GetTick(); // avoid lockup when initializing
	while(!read_Pin(BS_IRQ_pin)  &&  (HAL_GetTick()-currentTime < 5)); // wait for DR
	if (!bs_CheckFreq()){
		return false;
	}

	
	// Touch format
	while(read_Pin(BS_IRQ_pin)); // do not proceeed to Tx unless DR is low
	if (!bs_requestTouchFormat()){
		return false; // set freq failed, leave
	}
	currentTime = HAL_GetTick(); // avoid lockup when initializing
	// while(!read_Pin(BS_IRQ_pin)  &&  (HAL_GetTick()-currentTime < 5)); // wait for DR
	while(!read_Pin(BS_IRQ_pin)){}; // wait for DR
	if (!bs_readTouchFormat()){
		return false;
	}


	// enable device (set&check)
	while(read_Pin(BS_IRQ_pin)); // do not proceeed to Tx unless DR is low
	if (!bs_EnableDevice()){
		return false; // set enable failed, leave
	}
	currentTime = HAL_GetTick(); // avoid lockup when initializing
	while(!read_Pin(BS_IRQ_pin)  &&  (HAL_GetTick()-currentTime < 5)); // wait for DR
	if (!bs_CheckEnable()){
		return false;
	} else{
		// end of init procedure
		return (ballSensorInitialized) ? true : false;
	}
}

void ballSensor_DeInit(){
	ballSensorInitialized = 0;
	set_Pin(BS_RST_pin, 0);
	ballSensor_NoBall();
}

void ballSensor_Reset() {
	//Putty_printf ("BS_RST\n\r");
	ballSensorInitialized = 0;
	ballSensor_NoBall();
	set_Pin(BS_RST_pin, 0);
	HAL_Delay(1);
	set_Pin(BS_RST_pin, 1);
}

void ballSensorDeInit() {
	//Putty_printf ("BS_DEINIT\n\r");
	ballSensorInitialized = 0;
	ballSensor_NoBall();
	set_Pin(BS_RST_pin, 0);
}

void ballSensor_IRQ_Handler() {

	if (ballSensorInitialized) {
		if (ballSensor_I2C_Rx()){ // this I2C_Rx() does not have a timeout! it is meant for init

			// sprintf(logBuffer, "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
			// data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], 
			// data[10], data[11], data[12], data[13], data[14], data[15], data[16], data[17], data[18], data[19]);


			bs_CheckMeasurement();
		}
	}
}

bool ballSensor_isWorking() {
	return ballSensorInitialized;
}

void updatePosition(uint8_t data[]) {

	ballPosition.x = data[12] << 8 | data[13];
	ballPosition.y = data[14] << 8 | data[15];;
	ballPosition.lastSeen = HAL_GetTick();
	ballPosition.id = data[10];
	ballPosition.canKickBall = (ballPosition.y<500) ? 1 : 0;
	//ballPosition.canKickBall &= x > 50 && x < 700;
//	ballPosition.canKickBall &= x > 185 && x < 560;
	ballPosition.canSeeBall = 1;

	// printBallPosition();
}

void ballSensor_NoBall() {
	ballPosition.x = BALLSENSOR_NO_BALL;
	ballPosition.y = BALLSENSOR_NO_BALL;
	ballPosition.lastSeen = 0;
	ballPosition.id = -1;
	ballPosition.canKickBall = 0;
	ballPosition.canSeeBall = 0;
}

bool ballSensor_I2C_Rx() {
	/* Getting the data that the ballsensor has for the robot is done in two steps
	1. Figure out how much data the ballsensor has ready
		Read two bytes from the ballsensor. The first byte of the response should be 0xEE. The second byte of the response indicates the number of bytes that the ballsensor has ready
	2. Read the number of bytes figured out in step 1
	*/

	int currentTime = HAL_GetTick(); // avoid lockup
	while (BS_I2C->State != HAL_I2C_STATE_READY){
		if (HAL_GetTick()-currentTime > 5) {
			ballSensor_NoBall();
			ballSensorInitialized = 0;
			Putty_printf("[ballsensor][I2C_Rx] Timeout!\n", error);
			return false;
		}
	}

	/* 7.3.2 I2C, Page 44. */
	/* 3. Initiate 2 byte I2C read operation. Payload of this read should be EE XX where XX is the amount of bytes to read in a second I2C read operation. */
	uint8_t response[2];

	error = HAL_I2C_Master_Receive(BS_I2C, BS_I2C_ADDR, response, 2, 5);
	if(error != HAL_OK){
		ballSensorInitialized = 0;
		ballSensor_NoBall();
		Putty_printf("[ballsensor][I2C_Rx] Error at reading message length : %d!\n", error);
		return false;
	}
	uint8_t length_message = response[1];

	error = HAL_I2C_Master_Receive(BS_I2C, BS_I2C_ADDR, data, length_message, 5);
	if(error != HAL_OK){
		ballSensorInitialized = 0;
		ballSensor_NoBall();
		Putty_printf("[ballsensor][I2C_Rx] Error at reading full message : %d!\n", error);
		return false;
	}

	return true;
}

void I2CTx_IT(uint8_t tosend[], uint8_t length) {
    if(HAL_OK != (error = HAL_I2C_Master_Transmit_IT(BS_I2C, BS_I2C_ADDR, tosend, length))){
    	ballSensorInitialized = 0;
    	ballSensor_NoBall();
        Putty_printf("BALLSENSOR - i2c transmit failed with error [%d]!\n\rzForce stopped\n\r", error);
    }
}

// void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
// 	//Putty_printf("ballsensor tx callback\n\r");
// }

void I2C_Rx_DMA() {
	if(HAL_OK != (error = HAL_I2C_Master_Receive_DMA(BS_I2C, BS_I2C_ADDR, data, next_message_length))){
		ballSensor_NoBall();
		ballSensorInitialized = 0;
		Putty_printf("I2C_Rx_DMA - read failed with error [%d]!\n\r", error);
	}
}

// void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
// 	//Putty_printf("ballsensor rx callback\n\r");
// 	bs_CheckMeasurement();
// }

bool bs_Boot() {
	/* 7.3.2 I2C, Page 44.
	* 1. Power on the sensor.
	* 2. Wait for sensor to assert Data Ready pin (DR).
	* 3. Initiate 2 byte I2C read operation. Payload of this read should be EE XX where XX is the amount of bytes to read in a second I2C read operation.
	* 4. Read XX amount of bytes (number of bytes to read is indicated by second byte of first I2C Read Operation).
	  Now read a message called BootComplete. The message should be:
	  F0 11 40 02 00 00 63 0B 80 01 YY 81 02 03 YY 82 02 00 YY
	  where YY is usually "00" but can have another value. This signals that the sensor is now booted.
	  5. To enable the sensor to start sending touch notifications, do the following:
	  a. Send an Enable command: EE 09 40 02 02 00 65 03 81 01 00
	  b. Read the response. The response should be: EF 09 40 02 02 00 65 03 81 01 00
	*/ 
	ballSensor_I2C_Rx();
	// certain bytes of the response can change according to datasheet, so compare parts of the byte array
	return !memcmp( data, bootcomplete_response, 10) 
		&& !memcmp( data+11, bootcomplete_response+11, 3) 
		&& !memcmp( data+13, bootcomplete_response+13, 3);
}

bool bs_SetConfig() {
	error = HAL_I2C_Master_Transmit(BS_I2C, BS_I2C_ADDR, config_command, sizeof(config_command), 5);

	if (error != HAL_OK){
		ballSensorInitialized = 0;
		ballSensor_NoBall();
		Putty_printf("[ballsensor][bs_SetConfig] - i2c transmit failed with error [%d]!", error);
		return false;
	}
	
	return true;
}

bool bs_CheckConfig() {
	ballSensor_I2C_Rx();
	if (!memcmp(data, config_response, sizeof(config_response))) {
		//Putty_printf("BS_CONF\n\r");
		return true;
	} else {
		return false;
	}
}

bool bs_requestTouchFormat(){

	error = HAL_I2C_Master_Transmit(BS_I2C, BS_I2C_ADDR, get_touch_format, sizeof(get_touch_format), 5);

	if (error != HAL_OK)
		Putty_printf("[ballsensor][bs_getTouchFormat] I2C error %d\n", error);

	return error == HAL_OK;
}

bool bs_readTouchFormat(){

	if(!ballSensor_I2C_Rx()){
		Putty_printf("[ballsensor][bs_readTouchFormat] Read error\n");
		return false;
	}

	Putty_printf("%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
	data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], 
	data[10], data[11], data[12], data[13], data[14], data[15], data[16], data[17], data[18], data[19]);

	return true;
}

bool bs_SetFreq() {
	if (HAL_OK != HAL_I2C_Master_Transmit(BS_I2C, BS_I2C_ADDR, freq_command, sizeof(freq_command), 5)){
		ballSensorInitialized = 0;
		ballSensor_NoBall();
		Putty_printf("bs_SetFreq - i2c transmit failed with error [%d]!\n\r", error);
		return false;
	} else {
		return true;
	}
}

bool bs_CheckFreq() {
	ballSensor_I2C_Rx();
	if (!memcmp(data, freq_response, sizeof(freq_response))) {
		//Putty_printf("BS_FREQ\r\n");
		return true;
	} else {
		return false;
	}
}

bool bs_EnableDevice() {
	if (HAL_OK != HAL_I2C_Master_Transmit(BS_I2C, BS_I2C_ADDR, enable_command, sizeof(enable_command), 5)){
		ballSensorInitialized = 0;
		ballSensor_NoBall();
		Putty_printf("bs_EnableDevice - i2c transmit failed with error [%d]!\n\r", error);
		return false;
	} else {
		return true;
	}
}

bool bs_CheckEnable() {
	ballSensor_I2C_Rx();
	if (!memcmp(data, enable_response, sizeof(enable_response))) {
        //Putty_printf("BS_INIT COMPLETE\n\r");
        ballSensorInitialized = 1;
		return true;
	} else {
		return false;
	}
}

void bs_CheckMeasurement() {
	if(!memcmp(data, measurement_rx, sizeof(measurement_rx)) && data[11] != 2) {
		// ball detected if event ID (data[11]) is 0 or 1. data[11]==2 is for lost objects
		updatePosition(data);
		// if (true) Putty_printf ("ball: x=%d, y=%d\n\r", ballPosition.x, ballPosition.y);
	} else {
		//ignore any other data
		ballSensor_NoBall();
	}
}

void printRawData(uint8_t data[]) {
    Putty_printf("\n\rdata = [");
    for(uint32_t i = 0; i < next_message_length; i++){
      Putty_printf("0x%02X, ", data[i]);
    }
    Putty_printf("]\n\r");
}

void printBallPosition() {
	Putty_printf("\n\rball at\n\rx,y: %d  %d\n\r", ballPosition.x, ballPosition.y);
	Putty_printf("id is %d\n\r", ballPosition.id);
	Putty_printf("canKickBall: %d, canSeeBall: %d\n\r", ballPosition.canKickBall, ballPosition.canSeeBall);
	//Putty_printf("size %hu, %hu\n\r", data[16], data[17]);
}

