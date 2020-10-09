#include "basestation.h"
#include "main.h"
#include "Wireless.h"
#include "TextOut.h"
#include "msg_buff.h"

volatile int I1 = 0;
volatile int cFeedback = 0;
volatile int Iusb = 0;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim1;

extern uint32_t usbLength;
extern uint8_t usbData[64];

volatile bool new_packet = false;

void init(){
    HAL_Delay(1000);
    new_packet = false;
    
    // TextOut("[basestation][init] Initializing SX_TX\n");
    SX_TX = Wireless_Init(COMMAND_CHANNEL, &hspi1, 0);
    
    // TextOut("[basestation][init] Initializing SX_RX\n");
    SX_RX = Wireless_Init(FEEDBACK_CHANNEL, &hspi2, 1);
    SX_RX->SX_settings->syncWords[0] = robot_syncWord[16];
    setSyncWords(SX_RX, SX_RX->SX_settings->syncWords[0], 0x00, 0x00);
    setRX(SX_RX, SX_RX->SX_settings->periodBase, 0xFFFF);

    // TextOut("[basestation][init] Initializing Timer\n");
    HAL_TIM_Base_Start_IT(&htim1);
    
    // TextOut("[basestation][init] Initializion complete\n");
}

void loop(){

    /* Send any new command packets */
    for(int id = 0; id < 16; id++){
        if(msgBuff[id].isNewCommand){
            if(!isTransmitting){
                isTransmitting = true;
                SX_TX->SX_settings->syncWords[0] = robot_syncWord[id];
                setSyncWords(SX_TX, SX_TX->SX_settings->syncWords[0], 0, 0);
                SendPacket(SX_TX, msgBuff[id].command+2, PACKET_SIZE_ROBOT_COMMAND-2);
                msgBuff[id].isNewCommand = false;
            }
        }
    }
    
    /* Send any new feedback packets */
    for(int id = 0; id < 16; id++){
        if(msgBuff[id].isNewFeedback){
            HexOut(msgBuff[id].feedback, PACKET_SIZE_ROBOT_FEEDBACK);
            msgBuff[id].isNewFeedback = false;
        }
    }
}

/**
 * @brief 
 * 
 * @param Buf Pointer to the buffer the packet from the USB is stored in
 * @param Len Length of the packet currently in the buffer
 * @return true if the packed has been handled succesfully
 * @return false if the packet has been handled unsuccessfully, e.g. due to corruption
 */
bool handlePacket(uint8_t* Buf, uint32_t *Len){

  uint8_t packetType = Buf[0];
  uint32_t packetSize = *Len;
  
  if(packetType == PACKET_TYPE_ROBOT_COMMAND){
    // this is the USB callback
    // store the latest data in usbData and usbLength
    Iusb++;
    // if the length is correct, store the data in the buffer

    if (packetSize == 10) {
      // determine the robot id
      uint8_t usbDataRobotId = Buf[1];

      // check if the usb data robot id is legal
      if (usbDataRobotId < 16) {
        // put the message in the buffer
        memcpy(msgBuff[usbDataRobotId].command, Buf, PACKET_SIZE_ROBOT_COMMAND);
        msgBuff[usbDataRobotId].isNewCommand = true;
        return true;
      }
    }
    else {
      toggle_pin(LD_2);
    }

  }

  return false;
}

/* Triggers when a call to HAL_SPI_TransmitReceive_DMA or HAL_SPI_TransmitReceive_IT (both non-blocking) completes */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
    if(hspi->Instance == SX_TX->SPI->Instance) {
		I1++;
        Wireless_DMA_Handler(SX_TX);
	}
	if(hspi->Instance == SX_RX->SPI->Instance) {
        Wireless_DMA_Handler(SX_RX);
        new_packet = true;
	}
}

/* External interrupt. Triggers when one of the SX1280 antennas has information available */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    
	if (GPIO_Pin == SX_TX_IRQ.PIN) {
		Wireless_IRQ_Handler(SX_TX, 0, 0);
        I1++;
	}
	if (GPIO_Pin == SX_RX_IRQ.PIN) {
		Wireless_IRQ_Handler(SX_RX, 0, 0);
        cFeedback++;
		toggle_pin(LD_LED1);
	}
    
}