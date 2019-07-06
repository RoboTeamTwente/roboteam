#include "ballSensor.h"
#include "limits.h"
#include "gpio_util.h"
#include "PuTTY.h"
#include "string.h"
#include "peripheral_util.h"

uint error; // for i2c errors
uint8_t ball_debug = 0; // enable ball print statements
volatile uint8_t ballSensorInitialized = 0; // ball sensor initialization status
uint8_t next_message_length = 2; // default length of next message is 2 bytes
uint8_t init_attempts = 0;

/* ball sensor boot complete response */
uint8_t bootcomplete_response[] = {0xF0, 0x11, 0x40, 0x02, 0x00, 0x00, 0x63, 0x0B, 0x80, 0x01, 0x00, 0x81, 0x02, 0x03,
									0x00, 0x82, 0x02, 0x00, 0x00};

/* ball sensor configuration command, to be sent before enabling device */
/* minX 0, minY 0, maxX 700, maxY 440, minSize 0, reported&tracked touch 1, everything else off */
uint8_t config_command[] = {0xEE, 0x40, 0xEE, 0x3E, 0x40, 0x02, 0x02, 0x00, 0x73, 0x38, 0x80, 0x01, 0x01, 0xA2,
							0x1D, 0x80, 0x01, 0x00, 0x81, 0x01, 0x00, 0x82, 0x02, 0x02, 0xBC, 0x83, 0x02, 0x01,
							0xB8, 0x84, 0x01, 0x00, 0x85, 0x01, 0x00, 0x86, 0x01, 0x00, 0x87, 0x01, 0x00, 0x88,
							0x01, 0x00, 0xA4, 0x09, 0x80, 0x01, 0x00, 0x82, 0x01, 0x00, 0x83, 0x01, 0x00, 0x86,
							0x01, 0x01, 0xA7, 0x06, 0x80, 0x01, 0x00, 0x81, 0x01, 0x00};
uint8_t config_response[] = {0xEF, 0x43, 0x40, 0x02, 0x02, 0x00, 0x73, 0x3D, 0xA2, 0x1D,
							0x80, 0x01, 0x24, 0x81, 0x01, 0x00, 0x82, 0x02, 0x02, 0xBC, 0x83, 0x02, 0x01,
							0xB8, 0x84, 0x01, 0x00, 0x85, 0x01, 0x00, 0x86, 0x01, 0x00, 0x87, 0x01, 0x00,
							0x88, 0x01, 0x00, 0xA4, 0x0C, 0x80, 0x01, 0x00, 0x81, 0x01, 0x00, 0x82, 0x01,
							0x00, 0x83, 0x01, 0x00, 0x85, 0x01, 0x00, 0x86, 0x01, 0x01, 0xA7, 0x08, 0x80,
							0x02, 0x02, 0x98, 0x81, 0x02, 0x01, 0xB8};

/* ball sensor frequency command */
/* set scanning frequency to 100Hz (0x64) & idle frequency to 100Hz */
//uint8_t freq_command[] = {0xEE, 0x0E, 0xEE, 0x0C, 0x40, 0x02, 0x02, 0x00, 0x68, 0x06, 0x80, 0x01, 0x64, 0x82, 0x01, 0x64};
//uint8_t freq_response[] = {0xEF, 0x0C, 0x40, 0x02, 0x00, 0x00, 0x68, 0x06, 0x80, 0x01, 0x64, 0x82, 0x01, 0x64};
uint8_t freq_command[] = {0xEE, 0x10, 0xEE, 0x0E, 0x40, 0x02, 0x02, 0x00, 0x68, 0x08, 0x80, 0x02, 0x01, 0x90, 0x82, 0x02, 0x01, 0x90};
uint8_t freq_response[] = {0xEF, 0x0E, 0x40, 0x02, 0x00, 0x00, 0x68, 0x08, 0x80, 0x02, 0x01, 0x90, 0x82, 0x02, 0x01, 0x90};


/* ball sensor enable device command */
uint8_t enable_command[] = 	{0xEE, 0x0B, 0xEE, 0x09, 0x40, 0x02, 0x02, 0x00, 0x65, 0x03, 0x81, 0x01, 0x00};
uint8_t enable_response[] = {0xEF, 0x09, 0x40, 0x02, 0x02, 0x00, 0x65, 0x03, 0x81, 0x01, 0x00};

/* ball sensor receive data notification */
uint8_t measurement_rx[] = {0xF0, 0x11, 0x40, 0x02, 0x02};


bool ballSensor_Init()
{
	Putty_printf ("BS_INIT BEGIN\n\r");
	noBall();
	HAL_Delay(20); // timing specs
	ballSensor_Reset();
	next_message_length = 2;
	int currentTime = HAL_GetTick(); // avoid lockup when initializing
	while(!read_Pin(BS_IRQ_pin)){
		// wait for DR
		if (HAL_GetTick()-currentTime > 100) {
			Putty_printf ("BS_INIT TIMEOUT FAIL\n\r");
			return false;
		}
	}
//	HAL_Delay(100); // timing specs

	// boot procedure, only check for bootcomplete response
	if (!bs_Boot()){
		return false; // boot failed, leave
	}

	// config procedure (set&check)
	while(read_Pin(BS_IRQ_pin)); // do not proceeed to Tx unless DR is low
	if (!bs_SetConfig()){
		return false; // set config failed, leave
	}
	currentTime = HAL_GetTick(); // avoid lockup when initializing
	while(!read_Pin(BS_IRQ_pin)  &&  (HAL_GetTick()-currentTime < 5)); // wait for DR
	if (!bs_CheckConfig()){
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
	noBall();
}

void ballSensor_Reset() {
	//Putty_printf ("BS_RST\n\r");
	ballSensorInitialized = 0;
	noBall();
	set_Pin(BS_RST_pin, 0);
	HAL_Delay(1);
	set_Pin(BS_RST_pin, 1);
}

void ballSensorDeInit() {
	//Putty_printf ("BS_DEINIT\n\r");
	ballSensorInitialized = 0;
	noBall();
	set_Pin(BS_RST_pin, 0);
}

void ballSensor_IRQ_Handler() {
	if (ballSensorInitialized) {
//		while (bs_DMA_INUSE);
//		int currentTime = HAL_GetTick(); // avoid lockup
//		uint8_t next_msg[2] = {0}; // receive 2 bytes
//		while (BS_I2C->State != HAL_I2C_STATE_READY){
//			if (HAL_GetTick()-currentTime > 1000) {
//				noBall();
//				ballSensorInitialized = 0;
//				return;
//			}
//		}
//		if (HAL_OK != (error = HAL_I2C_Master_Receive(BS_I2C, BS_I2C_ADDR, next_msg, 2, 1000))){
//			noBall();
//			ballSensorInitialized = 0;
//			Putty_printf("I2C_Rx_Blocking 2bytes - read failed with error [%d]!\n\r", error);
//			return;
//		} else {
//			next_message_length = next_msg[1]; // determine message length
//			currentTime = HAL_GetTick(); // avoid lockup
//			while (BS_I2C->State != HAL_I2C_STATE_READY){
//				if (HAL_GetTick()-currentTime > 1000) {
//					noBall();
//					ballSensorInitialized = 0;
//					return;
//				}
//			}
//			// receive packet in non-blocking mode, bs_CheckMeasurement() is called by callback
//			I2C_Rx_DMA();
//		}

		// receive packet in blocking mode, first option, but comment out everything else:
		if (I2C_Rx()){ // this I2C_Rx() does not have a timeout! it is meant for init
			bs_CheckMeasurement();
		}

		// or second option using 2bytes read from above, but comment out i2c_rx_dma() call only
//		if (HAL_OK != (error = HAL_I2C_Master_Receive(BS_I2C, BS_I2C_ADDR, data, next_message_length, 1000))){
//			noBall();
//			ballSensorInitialized = 0;
//			Putty_printf("I2C_Rx_Blocking - read failed with error [%d]!\n\r", error);
//			return;
//		} else {
//			bs_CheckMeasurement();
//		}


	}
}

bool ballSensor_isWorking() {
	return ballSensorInitialized;
}

void updatePosition(uint8_t data[]) {
	uint16_t x;
	x = data[12] << 8;
	x |= data[13];
	uint16_t y;
	y = data[14] << 8;
	y |= data[15];

	ballPosition.x = x;
	ballPosition.y = y;
	ballPosition.lastSeen = HAL_GetTick();
	ballPosition.id = data[10];
	ballPosition.canKickBall = (y<250) ? 1 : 0;
	ballPosition.canKickBall &= x > 150 && x < 650;
//	ballPosition.canKickBall &= x > 185 && x < 560;
	ballPosition.canSeeBall = 1;

	//printBallPosition();
}

void noBall() {
	ballPosition.x = ballPosition.y NOBALL;
	ballPosition.lastSeen = 0;
	ballPosition.id = -1;
	ballPosition.canKickBall = 0;
	ballPosition.canSeeBall = 0;
}

int8_t getBallPos() {
	if(ballPosition.x != NOBALL) {
		return ballPosition.x/10;
	}
	return NOBALL;
}

bool I2C_Rx() {
	int currentTime = HAL_GetTick(); // avoid lockup
	while (BS_I2C->State != HAL_I2C_STATE_READY){
		if (HAL_GetTick()-currentTime > 5) {
			noBall();
			ballSensorInitialized = 0;
			return false;
		}
	}
	uint8_t next_msg_length[2];
	if (HAL_OK != (error = HAL_I2C_Master_Receive(BS_I2C, BS_I2C_ADDR, next_msg_length, 2, 5))){
		ballSensorInitialized = 0;
		noBall();
		Putty_printf("I2C_Rx_Blocking 2bytes - read failed with error [%d]!\n\r", error);
		return false;
	} else {
		if (HAL_OK != (error = HAL_I2C_Master_Receive(BS_I2C, BS_I2C_ADDR, data, next_msg_length[1], 5))){
			ballSensorInitialized = 0;
			noBall();
			Putty_printf("I2C_Rx_Blocking fullmsg - read failed with error [%d]!\n\r", error);
			return false;
		} else {
			return true;
		}
	}
}

void I2CTx_IT(uint8_t tosend[], uint8_t length) {
    if(HAL_OK != (error = HAL_I2C_Master_Transmit_IT(BS_I2C, BS_I2C_ADDR, tosend, length))){
    	ballSensorInitialized = 0;
    	noBall();
        Putty_printf("BALLSENSOR - i2c transmit failed with error [%d]!\n\rzForce stopped\n\r", error);
    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	//Putty_printf("ballsensor tx callback\n\r");
}

void I2C_Rx_DMA() {
	if(HAL_OK != (error = HAL_I2C_Master_Receive_DMA(BS_I2C, BS_I2C_ADDR, data, next_message_length))){
		noBall();
		ballSensorInitialized = 0;
		Putty_printf("I2C_Rx_DMA - read failed with error [%d]!\n\r", error);
	}
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	//Putty_printf("ballsensor rx callback\n\r");
	bs_CheckMeasurement();
}

bool bs_Boot() {
	I2C_Rx();
	// certain bytes of the response can change according to datasheet, so compare parts of the byte array
	if(!memcmp( data, bootcomplete_response, 10) && !memcmp( data+11, bootcomplete_response+11, 3) && !memcmp( data+13, bootcomplete_response+13, 3)) {
		Putty_printf("BS_BOOT\n\r");
		return true;
	} else {
		return false;
	}
}

bool bs_SetConfig() {
	if (HAL_OK != HAL_I2C_Master_Transmit(BS_I2C, BS_I2C_ADDR, config_command, sizeof(config_command), 5)){
		ballSensorInitialized = 0;
		noBall();
		Putty_printf("bs_SetConfig - i2c transmit failed with error [%d]!", error);
		return false;
	} else {
		return true;
	}
}

bool bs_CheckConfig() {
	I2C_Rx();
	if (!memcmp(data, config_response, sizeof(config_response))) {
		Putty_printf("BS_CONF\n\r");
		return true;
	} else {
		return false;
	}
}

bool bs_SetFreq() {
	if (HAL_OK != HAL_I2C_Master_Transmit(BS_I2C, BS_I2C_ADDR, freq_command, sizeof(freq_command), 5)){
		ballSensorInitialized = 0;
		noBall();
		Putty_printf("bs_SetFreq - i2c transmit failed with error [%d]!\n\r", error);
		return false;
	} else {
		return true;
	}
}

bool bs_CheckFreq() {
	I2C_Rx();
	if (!memcmp(data, freq_response, sizeof(freq_response))) {
		Putty_printf("BS_FREQ\r\n");
		return true;
	} else {
		return false;
	}
}

bool bs_EnableDevice() {
	if (HAL_OK != HAL_I2C_Master_Transmit(BS_I2C, BS_I2C_ADDR, enable_command, sizeof(enable_command), 5)){
		ballSensorInitialized = 0;
		noBall();
		Putty_printf("bs_EnableDevice - i2c transmit failed with error [%d]!\n\r", error);
		return false;
	} else {
		return true;
	}
}

bool bs_CheckEnable() {
	I2C_Rx();
	if (!memcmp(data, enable_response, sizeof(enable_response))) {
        Putty_printf("BS_INIT COMPLETE\n\r");
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
		if (ball_debug == 1) Putty_printf ("ball: x=%d, y=%d\n\r", ballPosition.x, ballPosition.y);
	} else {
		//ignore any other data
		noBall();
	}
}

void printRawData(uint8_t data[]) {
    Putty_printf("\n\rdata = [");
    for(uint i = 0; i < next_message_length; i++){
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

