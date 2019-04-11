/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define APB 216
#define Charge_done_Pin GPIO_PIN_2
#define Charge_done_GPIO_Port GPIOE
#define LF_Locked_Pin GPIO_PIN_3
#define LF_Locked_GPIO_Port GPIOE
#define LF_FR_Pin GPIO_PIN_4
#define LF_FR_GPIO_Port GPIOE
#define LF_PWM_Pin GPIO_PIN_5
#define LF_PWM_GPIO_Port GPIOE
#define LB_PWM_Pin GPIO_PIN_6
#define LB_PWM_GPIO_Port GPIOE
#define LB_FR_Pin GPIO_PIN_13
#define LB_FR_GPIO_Port GPIOC
#define LB_Locked_Pin GPIO_PIN_14
#define LB_Locked_GPIO_Port GPIOC
#define PWM_Buzzer_Pin GPIO_PIN_6
#define PWM_Buzzer_GPIO_Port GPIOF
#define LF_CHA_Pin GPIO_PIN_0
#define LF_CHA_GPIO_Port GPIOA
#define LF_CHB_Pin GPIO_PIN_1
#define LF_CHB_GPIO_Port GPIOA
#define SW_Freq_Pin GPIO_PIN_4
#define SW_Freq_GPIO_Port GPIOA
#define OUT_Pin GPIO_PIN_5
#define OUT_GPIO_Port GPIOA
#define LB_CHA_Pin GPIO_PIN_6
#define LB_CHA_GPIO_Port GPIOA
#define LB_CHB_Pin GPIO_PIN_7
#define LB_CHB_GPIO_Port GPIOA
#define ID0_Pin GPIO_PIN_4
#define ID0_GPIO_Port GPIOC
#define ID1_Pin GPIO_PIN_5
#define ID1_GPIO_Port GPIOC
#define ID2_Pin GPIO_PIN_0
#define ID2_GPIO_Port GPIOB
#define ID3_Pin GPIO_PIN_1
#define ID3_GPIO_Port GPIOB
#define LD6_Pin GPIO_PIN_11
#define LD6_GPIO_Port GPIOF
#define LD5_Pin GPIO_PIN_12
#define LD5_GPIO_Port GPIOF
#define LD4_Pin GPIO_PIN_13
#define LD4_GPIO_Port GPIOF
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOF
#define LD2_Pin GPIO_PIN_15
#define LD2_GPIO_Port GPIOF
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOG
#define LD0_Pin GPIO_PIN_1
#define LD0_GPIO_Port GPIOG
#define SPI4_RST_Pin GPIO_PIN_15
#define SPI4_RST_GPIO_Port GPIOE
#define SPI4_BUSY_Pin GPIO_PIN_10
#define SPI4_BUSY_GPIO_Port GPIOB
#define SPI4_IRQ_Pin GPIO_PIN_11
#define SPI4_IRQ_GPIO_Port GPIOB
#define SPI4_IRQ_EXTI_IRQn EXTI15_10_IRQn
#define PC_RX_Pin GPIO_PIN_12
#define PC_RX_GPIO_Port GPIOB
#define PC_TX_Pin GPIO_PIN_13
#define PC_TX_GPIO_Port GPIOB
#define RB_CHA_Pin GPIO_PIN_12
#define RB_CHA_GPIO_Port GPIOD
#define RB_CHB_Pin GPIO_PIN_13
#define RB_CHB_GPIO_Port GPIOD
#define RB_Locked_Pin GPIO_PIN_7
#define RB_Locked_GPIO_Port GPIOG
#define RB_FR_Pin GPIO_PIN_8
#define RB_FR_GPIO_Port GPIOG
#define RB_PWM_Pin GPIO_PIN_6
#define RB_PWM_GPIO_Port GPIOC
#define RF_PWM_Pin GPIO_PIN_7
#define RF_PWM_GPIO_Port GPIOC
#define PWM_Dribbler_Pin GPIO_PIN_8
#define PWM_Dribbler_GPIO_Port GPIOC
#define PWM_Geneva_Pin GPIO_PIN_9
#define PWM_Geneva_GPIO_Port GPIOC
#define RF_CHA_Pin GPIO_PIN_8
#define RF_CHA_GPIO_Port GPIOA
#define RF_CHB_Pin GPIO_PIN_9
#define RF_CHB_GPIO_Port GPIOA
#define RF_FR_Pin GPIO_PIN_10
#define RF_FR_GPIO_Port GPIOA
#define RF_Locked_Pin GPIO_PIN_11
#define RF_Locked_GPIO_Port GPIOA
#define Geneva_CHA_Pin GPIO_PIN_15
#define Geneva_CHA_GPIO_Port GPIOA
#define Battery_empty_Pin GPIO_PIN_10
#define Battery_empty_GPIO_Port GPIOC
#define Geneva_cal_sensor_Pin GPIO_PIN_0
#define Geneva_cal_sensor_GPIO_Port GPIOD
#define Geneva_DIRA_Pin GPIO_PIN_1
#define Geneva_DIRA_GPIO_Port GPIOD
#define Geneva_DIRB_Pin GPIO_PIN_2
#define Geneva_DIRB_GPIO_Port GPIOD
#define XSENS_IRQ_Pin GPIO_PIN_5
#define XSENS_IRQ_GPIO_Port GPIOD
#define XSENS_IRQ_EXTI_IRQn EXTI9_5_IRQn
#define XSENS_RST_Pin GPIO_PIN_6
#define XSENS_RST_GPIO_Port GPIOD
#define QUADSPI_IRQ_Pin GPIO_PIN_14
#define QUADSPI_IRQ_GPIO_Port GPIOG
#define QUADSPI_IRQ_EXTI_IRQn EXTI15_10_IRQn
#define Geneva_CHB_Pin GPIO_PIN_3
#define Geneva_CHB_GPIO_Port GPIOB
#define BS_RST_Pin GPIO_PIN_5
#define BS_RST_GPIO_Port GPIOB
#define BS_IRQ_Pin GPIO_PIN_6
#define BS_IRQ_GPIO_Port GPIOB
#define BS_IRQ_EXTI_IRQn EXTI9_5_IRQn
#define Charge_Pin GPIO_PIN_9
#define Charge_GPIO_Port GPIOB
#define Chip_Pin GPIO_PIN_0
#define Chip_GPIO_Port GPIOE
#define Kick_Pin GPIO_PIN_1
#define Kick_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
