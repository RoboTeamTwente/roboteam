#include "basestation.h"
#include "Wireless.h"
#include "TextOut.h"

volatile int I1 = 0;
volatile int I2 = 0;

extern SPI_HandleTypeDef hspi3;
extern TIM_HandleTypeDef htim1;

void init(){
    HAL_Delay(2000);
    TextOut("[basestation][init] Initializing SX\n");
    SX = Wireless_Init(20, &hspi3);
    TextOut("[basestation][init] Initializing Timer\n");
    HAL_TIM_Base_Start_IT(&htim1);
    TextOut("[basestation][init] Initializion complete\n");
}

void loop(){
    TextOut("loop\n");
    char msg[64];
    
    while(1){
        sprintf(msg, "Interrupt status : I1=%d, I2=%d\n", I1, I2);
        TextOut(msg);
        HAL_Delay(1000);
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
    TextOut("HAL_SPI_TxRxCpltCallback()\n");
    I1 = 1;

    for(int i = 0; i < 20; i++){
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
        HAL_Delay(50);
    }
    // if(hspi->Instance == SX->SPI->Instance) {
    // 	Wireless_DMA_Handler(SX, PC_to_Bot);
    // }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  TextOut("HAL_SPI_TxRxCpltCallback()\n");
  I2 = 2;
  for(int i = 0; i < 20; i++){
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
    HAL_Delay(50);
  }
	// if (GPIO_Pin == SX_IRQ.PIN) {
	// 	Wireless_IRQ_Handler(SX, 0, 0);
	// }
}