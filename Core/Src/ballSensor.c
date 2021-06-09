/* NNAMC0720PC01 */
/* Page 114. Detection area = 72mm * 44.7mm */
/* Page 105. Touch resolution = 0.1mm */


#include "ballSensor.h"
#include "limits.h"
#include "gpio_util.h"
#include "PuTTY.h"
#include "string.h"
#include "peripheral_util.h"
#include "main.h"

// Static to keep everything local to this file
static uint32_t error; // for i2c errors
static volatile bool initialized = false; // ball sensor initialization status
uint8_t data[255] __attribute__((aligned(16))); // byte array for received messages





// ====================== REQUESTS / RESPONSES / NOTIFICATIONS ====================== //

// Important. Values above 127 are represented by two bytes, even though it fits in one.
// 7.4.1 Serialization Protocol Quick Start - Encoding Integers
// * If the integer is between 0 and 127, it is represented by one byte ( 00 to 7F ).
// * If the integer is between 128 and 32767, it is represented by two bytes ( 00 80 to 7F FF ).

/* Ball sensor boot complete response. 7.3.2 I2C, Page 44 */
uint8_t response_bootComplete[] = {BS_NOTIFICATION, 17, BS_DEVICE_ADDRESS_PLATFORM, 
	BS_BOOT_COMPLETE_NOTIFICATION, 11, BS_ASIC_STATUS, 1, 0, 0x81, 0x02, 0x03, 0x00, 0x82, 0x02, 0x00, 0x00};

/* Device Configuration */
uint8_t request_deviceConfiguration[] = {
	BS_REQUEST, 27, /* Really not sure why I need to send BS_REQUEST twice, but it's the only way this works */
	BS_REQUEST, 25, /* Asked on Github, but no response.. https://github.com/neonode-inc/zforce-arduino/issues/52 */
	BS_DEVICE_ADDRESS_AIR,
	BS_DEVICE_CONFIGURATION, 19,
		/* Make sure that only one object is tracked. The code is not set up to deal with messages thar report multiple touches */
		BS_NUMBER_OF_REPORTED_TOUCHES, 1, 1, 
		/* This is not really needed, these are default values. However, let's leave this here in case we need to change it */
		BS_SUBTOUCHACTIVEAREA, 14,
			BS_LOWBOUNDX, 1, 0, BS_LOWBOUNDY, 1, 0,
			BS_HIGHBOUNDX, 2, 700 >> 8, 700 & 0xFF,
			BS_HIGHBOUNDY, 2, 440 >> 8, 440 & 0xFF
	};
/* Page 81, Device Configuration response */
/* I'm not going to completely check the response. As long as the ballsensor works, it's fine */
uint8_t response_deviceConfiguration[] = {BS_DEVICE_ADDRESS_AIR, BS_DEVICE_CONFIGURATION};

/* A ball can move at most 6.5 m/s. Page 29 https://robocup-ssl.github.io/ssl-rules/sslrules.pdf */
/* The plunger of the robot is 45mm in width. 6500 / 45 gives a required scanning frequency of at least 144.44. Let's stick with 200 */
/* 9.2.4 Scanning Frequency, page 109 */
uint8_t request_scanningFrequency[] = {BS_REQUEST, 16, BS_REQUEST, 14, BS_DEVICE_ADDRESS_AIR, BS_FREQUENCY, 8, BS_FINGER, 2, 200 >> 8, 200 & 0xFF, BS_IDLE, 2, 200 >> 8, 200 & 0xFF};
uint8_t response_scanningFrequency[] = {BS_RESPONSE, 14, BS_DEVICE_ADDRESS_PLATFORM, BS_FREQUENCY, 8, BS_FINGER, 2, 200 >> 8, 200 & 0xFF, BS_IDLE, 2, 200 >> 8, 200 & 0xFF};

/* Still haven't completely figured out the response that this gives... Page 76 */
uint8_t request_touchFormat[] = {BS_REQUEST, 8, BS_REQUEST, 6, BS_DEVICE_ADDRESS_AIR, BS_TOUCH_FORMAT, 0};

/* Ballsensor enable device command. 7.4.3 I2C Transport, Page 69 */
/* 7.4.1 Serialization Protocol Quick Start -> Enabling Touch Sensor Modules, page 50 */
uint8_t request_enable[] = 	{BS_REQUEST, 11, BS_REQUEST, 9, BS_DEVICE_ADDRESS_AIR, BS_ENABLE_SEQUENCE, 3, BS_ENABLE, 1, BS_ENABLE_CONTINUOUS};
uint8_t response_enable[] = {BS_RESPONSE, 9, BS_DEVICE_ADDRESS_AIR, BS_ENABLE_SEQUENCE, 3, BS_ENABLE, 1, BS_ENABLE_CONTINUOUS};

/* ball sensor receive data notification */
uint8_t notification_touch[] = {BS_NOTIFICATION, 17, BS_DEVICE_ADDRESS_AIR, BS_TOUCH_NOTIFICATION_SEQUENCE, 11, BS_TOUCH_NOTIFICATION, 9};

// ================================================================================== //





// ====================== PUBLIC FUNCTIONS ====================== //

bool ballSensor_Init(){
	Putty_printf("[ballsensor] init\n");
	
	bs_NoBall();
	HAL_Delay(20); // timing specs
	ballSensor_Reset();
	
	int currentTime = HAL_GetTick(); // avoid lockup when initializing
	while(!read_Pin(BS_IRQ_pin)){
		if (HAL_GetTick() - currentTime > 100) {
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

	// Set configuration
	while(read_Pin(BS_IRQ_pin)); // do not proceeed to Tx unless DR is low
	if (!bs_setDeviceConfiguration()){
		Putty_printf("[ballsensor] set config failed\n");
		return false; // set config failed, leave
	}
	// Check configuration
	currentTime = HAL_GetTick(); // avoid lockup when initializing
	while(!read_Pin(BS_IRQ_pin) && (HAL_GetTick()-currentTime < 5)); // wait for DR
	if (!bs_checkDeviceConfiguration()){
		Putty_printf("[ballsensor] check config failed\n");
		return false;
	}

	// Set scanning frequency
	while(read_Pin(BS_IRQ_pin)); // do not proceeed to Tx unless DR is low
	if (!bs_setScanningFrequency()){
		return false; // set freq failed, leave
	}
	// Check scanning frequency
	currentTime = HAL_GetTick(); // avoid lockup when initializing
	while(!read_Pin(BS_IRQ_pin)  &&  (HAL_GetTick()-currentTime < 5)); // wait for DR
	if (!bs_checkScanningFrequency()){
		return false;
	}
	
	// Touch format
	while(read_Pin(BS_IRQ_pin)); // do not proceeed to Tx unless DR is low
	if (!bs_getTouchFormat()){
		return false; // set freq failed, leave
	}
	currentTime = HAL_GetTick(); // avoid lockup when initializing
	while(!read_Pin(BS_IRQ_pin) && (HAL_GetTick()-currentTime < 5)); // wait for DR
	if (!bs_checkTouchFormat()){
		return false;
	}

	// Enable device
	while(read_Pin(BS_IRQ_pin)); // do not proceeed to Tx unless DR is low
	if (!bs_setEnableDevice()){
		return false; // set enable failed, leave
	}
	currentTime = HAL_GetTick(); // avoid lockup when initializing
	while(!read_Pin(BS_IRQ_pin) && (HAL_GetTick() - currentTime < 5)); // wait for DR
	return bs_checkEnableDevice();
}

void ballSensor_DeInit(){
	initialized = false;
	set_Pin(BS_RST_pin, 0);
	bs_NoBall();
}

void ballSensor_Reset() {
	initialized = false;
	bs_NoBall();
	set_Pin(BS_RST_pin, 0);
	HAL_Delay(1);
	set_Pin(BS_RST_pin, 1);
}

void ballSensor_IRQ_Handler() {

	if (initialized) {
		if (ballSensor_I2C_Rx()){ // this I2C_Rx() does not have a timeout! it is meant for init

			// If interrupt is handled by CheckMeasurement, then its all good
			if(bs_CheckMeasurement())
				return;

			// Received interrupt that can't be handled
			uint8_t message_size = data[1];
			for(uint16_t at = 0; at < message_size; at += 10){
				Putty_printf("[%d] %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
				at, data[at+0], data[at+1], data[at+2], data[at+3], data[at+4], data[at+5], data[at+6], data[at+7], data[at+8], data[at+9]);
			}
		}
	}
}

bool ballSensor_isInitialized() {
	return initialized;
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
			bs_NoBall();
			initialized = false;
			Putty_printf("[ballsensor][I2C_Rx] Timeout!\n", error);
			return false;
		}
	}

	/* 7.3.2 I2C, Page 44. */
	/* 3. Initiate 2 byte I2C read operation. Payload of this read should be EE XX where XX is the amount of bytes to read in a second I2C read operation. */
	uint8_t response[2];

	error = HAL_I2C_Master_Receive(BS_I2C, BS_I2C_ADDR, response, 2, 5);
	if(error != HAL_OK){
		initialized = false;
		bs_NoBall();
		Putty_printf("[ballsensor][I2C_Rx] Error at reading message length : %d!\n", error);
		return false;
	}
	uint8_t length_message = response[1];

	error = HAL_I2C_Master_Receive(BS_I2C, BS_I2C_ADDR, data, length_message, 5);
	if(error != HAL_OK){
		initialized = false;
		bs_NoBall();
		Putty_printf("[ballsensor][I2C_Rx] Error at reading full message : %d!\n", error);
		return false;
	}

	return true;
}

// ====================== PRIVATE FUNCTIONS ====================== //

void bs_I2C_error(uint8_t error){
	initialized = false;
	bs_NoBall();
	Putty_printf("[bs_I2C_error] %d\n", error);
}

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
	return !memcmp( data, response_bootComplete, 10) 
		&& !memcmp( data+11, response_bootComplete+11, 3) 
		&& !memcmp( data+13, response_bootComplete+13, 3);
}


bool bs_setDeviceConfiguration() {
	error = HAL_I2C_Master_Transmit(BS_I2C, BS_I2C_ADDR, request_deviceConfiguration, sizeof(request_deviceConfiguration), 5);

	if (error != HAL_OK)
		bs_I2C_error(error);

	return error == HAL_OK;
}

bool bs_checkDeviceConfiguration() {
	ballSensor_I2C_Rx();

	// Putty_printf("[CC] %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
	// data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15], data[16], data[17], data[18], data[19], data[20], data[21], data[22], data[23], data[24], data[25], data[26], data[27], data[28], data[29]);

	// Putty_printf("[CC] %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
	// data[30], data[31], data[32], data[33], data[34], data[35], data[36], data[37], data[38], data[39], data[40], data[41], data[42], data[43], data[44], data[45], data[46], data[47], data[48], data[49], data[50], data[51], data[52], data[53], data[54], data[55], data[56], data[57], data[58], data[59]);
	
	// Putty_printf("[CC] %02X %02X %02X %02X %02X %02X %02X %02X %02X\n", 
	// data[60], data[61], data[62], data[63], data[64], data[65], data[66], data[67], data[68]);

	bool isEqual = !memcmp(data + 2, response_deviceConfiguration, sizeof(response_deviceConfiguration));

	return isEqual;
}


bool bs_getTouchFormat(){

	error = HAL_I2C_Master_Transmit(BS_I2C, BS_I2C_ADDR, request_touchFormat, sizeof(request_touchFormat), 5);

	if (error != HAL_OK)
		bs_I2C_error(error);

	return error == HAL_OK;
}

bool bs_checkTouchFormat(){
	// Still haven't figured out the response that this gives ...
	if(!ballSensor_I2C_Rx()){
		Putty_printf("[ballsensor][bs_readTouchFormat] Read error\n");
		return false;
	}

	// Putty_printf("%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
	// data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], 
	// data[10], data[11], data[12], data[13], data[14], data[15], data[16], data[17], data[18], data[19]);

	return true;
}


bool bs_setScanningFrequency() {
	uint8_t error = HAL_I2C_Master_Transmit(BS_I2C, BS_I2C_ADDR, request_scanningFrequency, sizeof(request_scanningFrequency), 5);

	if (error != HAL_OK)
		bs_I2C_error(error);

	return error == HAL_OK;
}

bool bs_checkScanningFrequency() {
	ballSensor_I2C_Rx();

	bool isEqual = !memcmp(data, response_scanningFrequency, sizeof(response_scanningFrequency));
	
	return isEqual;
}


bool bs_setEnableDevice() {
	uint8_t error = HAL_I2C_Master_Transmit(BS_I2C, BS_I2C_ADDR, request_enable, sizeof(request_enable), 5);
	
	if (error != HAL_OK)
		bs_I2C_error(error);

	return error == HAL_OK;
}

bool bs_checkEnableDevice() {
	ballSensor_I2C_Rx();
	
	bool isEqual = !memcmp(data, response_enable, sizeof(response_enable));

	initialized = isEqual;
	
	return isEqual;
}


bool bs_CheckMeasurement() {
	bool isEqual = !memcmp(data, notification_touch, sizeof(notification_touch));
	
	if(isEqual){
		if(data[11] == 2)
			bs_NoBall();
		else
			bs_updatePosition(data);
	}

	return isEqual;
}


void bs_updatePosition(uint8_t data[]) {
	ballPosition.x = data[12] << 8 | data[13];
	ballPosition.y = data[14] << 8 | data[15];;
	ballPosition.lastSeen = HAL_GetTick();
	ballPosition.id = data[10];
	ballPosition.canKickBall = (0 <= ballPosition.y && ballPosition.y <= 500) ? 1 : 0;
	ballPosition.canSeeBall = 1;
}

void bs_NoBall() {
	ballPosition.x = BALLSENSOR_NO_BALL;
	ballPosition.y = BALLSENSOR_NO_BALL;
	ballPosition.lastSeen = 0;
	ballPosition.id = -1;
	ballPosition.canKickBall = 0;
	ballPosition.canSeeBall = 0;
}
