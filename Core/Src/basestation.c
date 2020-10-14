#include "basestation.h"
#include "main.h"
#include "Wireless.h"
#include "TextOut.h"
#include "msg_buff.h"
#include "FT812Q.h"

#include "BaseTypes.h"
#include "BasestationStatistics.h"
#include "RobotCommand.h"

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

    // display_Init();
    // drawBasestation(true);

    // TextOut("[basestation][init] Initializion complete\n");
}

void loop(){

  /* Send any new command packets */
  for(int id = 0; id < 16; id++){
    if(msgBuff[id].isNewCommand){
      if(!isTransmitting){
        isTransmitting = true;
        msgBuff[id].packetsSent++;
        SX_TX->SX_settings->syncWords[0] = robot_syncWord[id];
        setSyncWords(SX_TX, SX_TX->SX_settings->syncWords[0], 0, 0);
        SendPacket(SX_TX, msgBuff[id].command.payload+2, PACKET_SIZE_ROBOT_COMMAND-2);
        msgBuff[id].isNewCommand = false;
      }
    }
  }
  
  /* Send any new feedback packets */
  for(int id = 0; id < 16; id++){
    if(msgBuff[id].isNewFeedback){
        HexOut(msgBuff[id].feedback.payload, PACKET_SIZE_ROBOT_FEEDBACK);
        msgBuff[id].isNewFeedback = false;
        msgBuff[id].packetsReceived++;
        // drawRobotInfo(14, true);
    }
    if(id == 14){
      toggle_pin(LD_LED2);
    }
  }
}

/**
 * @brief Receives a buffer which is assumed to be holding a RobotCommand packet. 
 * If so, it moves the packet to the buffer, and sets a flag to send the packet to
 * the corresponding robot.
 * 
 * @param Buf Pointer to the buffer that holds the packet
 * @param Len Length of the packet
 * @return true if the packet has been handled succesfully
 * @return false if there was something wrong with the packet
 */
bool handleRobotCommand(uint8_t* Buf, uint32_t Len){
  // Check if the packet has the correct size
  if(Len != PACKET_SIZE_ROBOT_COMMAND)
    return false;

  // Interpret buffer as a RobotCommandPayload
  RobotCommandPayload *rc = (RobotCommandPayload*) Buf;

  // Check if the robotId is valid
  uint8_t robotId = RobotCommand_getId(rc);
  if (16 <= robotId)
    return false;
    
  // Store the message in the buffer. Set flag to be sent to the robot
  memcpy(msgBuff[robotId].command.payload, Buf, PACKET_SIZE_ROBOT_COMMAND);
  msgBuff[robotId].isNewCommand = true;
  
  return true;
}

bool handleStatistics(void){
  basestationStatistics bs = {0};
  bs.header = PACKET_TYPE_BASESTATION_STATISTICS;
  for(int ID = 0; ID < 16; ID++){
    // Send ratio + packets sent (clipped) for each robot
    bs.robot[ID].packetsReceived  = msgBuff[ID].packetsReceived >= 255 ? 255 : msgBuff[ID].packetsReceived;
    bs.robot[ID].packetsSent      = msgBuff[ID].packetsSent     >= 255 ? 255 : msgBuff[ID].packetsSent;
    // Reset statistics
    msgBuff[ID].packetsSent = 0;
    msgBuff[ID].packetsReceived = 0;
  }
  HexOut(bs.payload, PACKET_SIZE_BASESTATION_STATISTICS);
  return true;
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
  
  Iusb++;

  switch(packetType){
    case PACKET_TYPE_ROBOT_COMMAND:
      handleRobotCommand(Buf, packetSize);
      break;
    case PACKET_TYPE_BASESTATION_GET_STATISTICS:
      handleStatistics();
      break;
    default:
      break;
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