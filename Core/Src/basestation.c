#include "basestation.h"
#include "main.h"
#include "Wireless.h"
#include "TextOut.h"

volatile int I1 = 0;
volatile int I2 = 0;
volatile int Iusb = 0;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim1;

extern uint32_t usbLength;
extern uint8_t usbData[64];

void init(){
    HAL_Delay(1000);
    
    TextOut("[basestation][init] Initializing SX_TX\n");
    SX_TX = Wireless_Init(COMMAND_CHANNEL, &hspi1, 0);
    
    TextOut("[basestation][init] Initializing SX_RX\n");
    SX_RX = Wireless_Init(COMMAND_CHANNEL, &hspi2, 1);
    SX_RX->SX_settings->syncWords[0] = robot_syncWord[16];
    setSyncWords(SX_RX, SX_RX->SX_settings->syncWords[0], 0x00, 0x00);
    setRX(SX_RX, SX_RX->SX_settings->periodBase, 0xFFFF);

    TextOut("[basestation][init] Initializing Timer\n");
    HAL_TIM_Base_Start_IT(&htim1);
    
    TextOut("[basestation][init] Initializion complete\n");
}

uint8_t msgToSend[8] = {'h', 'e', 'l', 'l', 'o', 'o', 'o', 'o'};

void loop(){
    TextOut("loop\n");
    char msg[64];
    
    SX_TX->SX_settings->syncWords[0] = robot_syncWord[16];
    setSyncWords(SX_TX, SX_TX->SX_settings->syncWords[0], 0x00, 0x00);

    while(1){
        sprintf(msg, "I1=%d, I2=%d Iusb=%d length=%d\n", I1, I2, Iusb, usbLength);
        TextOut(msg);

        SendPacket(SX_TX, msgToSend, 8);
        TextOut("Packet sent\n");
        HAL_Delay(100);
        getStatus(SX_TX);
        getStatus(SX_RX);
        
        sprintf(msg, "StatusTX : busy:%d, cmd:%d, int:%d\n", SX_TX->SX1280_status.busy, SX_TX->SX1280_status.CommandStatus, SX_TX->SX1280_status.InternalState);
        TextOut(msg);
        sprintf(msg, "StatusRX : busy:%d, cmd:%d, int:%d\n", SX_RX->SX1280_status.busy, SX_RX->SX1280_status.CommandStatus, SX_RX->SX1280_status.InternalState);
        TextOut(msg);

        TextOut("Packet received:");
        HexOut(Bot_to_PC, 8);
        TextOut("\n");
        HAL_Delay(500); 

    }
}

void sendFakePacket(){
    
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
    if(hspi->Instance == SX_TX->SPI->Instance) {
		I1++;
        Wireless_DMA_Handler(SX_TX, PC_to_Bot);
	}
	if(hspi->Instance == SX_RX->SPI->Instance) {
        I2++;
		Wireless_DMA_Handler(SX_RX, PC_to_Bot);
	}
    // TextOut("HAL_SPI_TxRxCpltCallback() handled\n");
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    
	if (GPIO_Pin == SX_TX_IRQ.PIN) {
		Wireless_IRQ_Handler(SX_TX, 0, 0);
        I1++;
	}
	if (GPIO_Pin == SX_RX_IRQ.PIN) {
		Wireless_IRQ_Handler(SX_RX, 0, 0);
        I2++;
		toggle_pin(LD_LED1);
	}
    
}