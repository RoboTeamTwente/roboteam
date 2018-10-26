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
  * COPYRIGHT(c) 2018 STMicroelectronics
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define MAX_PWM 2400
#define ENCODER_FILTER 15
#define APB1 48

#define FR_RB_Pin GPIO_PIN_4
#define FR_RB_GPIO_Port GPIOE
#define PWM_RB_Pin GPIO_PIN_5
#define PWM_RB_GPIO_Port GPIOE
#define PWM_RF_Pin GPIO_PIN_6
#define PWM_RF_GPIO_Port GPIOE
#define FR_RF_Pin GPIO_PIN_13
#define FR_RF_GPIO_Port GPIOC
#define XSENS_nRST_Pin GPIO_PIN_4
#define XSENS_nRST_GPIO_Port GPIOA
#define CHB_LB_Pin GPIO_PIN_6
#define CHB_LB_GPIO_Port GPIOA
#define CHA_LB_Pin GPIO_PIN_7
#define CHA_LB_GPIO_Port GPIOA
#define Charge_done_Pin GPIO_PIN_4
#define Charge_done_GPIO_Port GPIOC
#define Charge_Pin GPIO_PIN_5
#define Charge_GPIO_Port GPIOC
#define Chip_Pin GPIO_PIN_0
#define Chip_GPIO_Port GPIOB
#define Kick_Pin GPIO_PIN_1
#define Kick_GPIO_Port GPIOB
#define LD6_Pin GPIO_PIN_2
#define LD6_GPIO_Port GPIOB
#define LD5_Pin GPIO_PIN_7
#define LD5_GPIO_Port GPIOE
#define LD4_Pin GPIO_PIN_8
#define LD4_GPIO_Port GPIOE
#define LD3_Pin GPIO_PIN_9
#define LD3_GPIO_Port GPIOE
#define LD2_Pin GPIO_PIN_10
#define LD2_GPIO_Port GPIOE
#define LD1_Pin GPIO_PIN_11
#define LD1_GPIO_Port GPIOE
#define ID3_Pin GPIO_PIN_12
#define ID3_GPIO_Port GPIOE
#define ID2_Pin GPIO_PIN_13
#define ID2_GPIO_Port GPIOE
#define ID1_Pin GPIO_PIN_14
#define ID1_GPIO_Port GPIOE
#define ID0_Pin GPIO_PIN_15
#define ID0_GPIO_Port GPIOE
#define SPI1_IRQ_Pin GPIO_PIN_11
#define SPI1_IRQ_GPIO_Port GPIOB
#define SPI1_IRQ_EXTI_IRQn EXTI15_10_IRQn
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define FR_LB_Pin GPIO_PIN_13
#define FR_LB_GPIO_Port GPIOB
#define PWM_LB_Pin GPIO_PIN_14
#define PWM_LB_GPIO_Port GPIOB
#define PWM_LF_Pin GPIO_PIN_15
#define PWM_LF_GPIO_Port GPIOB
#define FR_LF_Pin GPIO_PIN_8
#define FR_LF_GPIO_Port GPIOD
#define CHA_LF_Pin GPIO_PIN_12
#define CHA_LF_GPIO_Port GPIOD
#define CHB_LF_Pin GPIO_PIN_13
#define CHB_LF_GPIO_Port GPIOD
#define CHA_RF_Pin GPIO_PIN_6
#define CHA_RF_GPIO_Port GPIOC
#define CHB_RF_Pin GPIO_PIN_7
#define CHB_RF_GPIO_Port GPIOC
#define CHA_RB_Pin GPIO_PIN_8
#define CHA_RB_GPIO_Port GPIOA
#define CHB_RB_Pin GPIO_PIN_9
#define CHB_RB_GPIO_Port GPIOA
#define Geneva_CHA_Pin GPIO_PIN_15
#define Geneva_CHA_GPIO_Port GPIOA
#define PC_RX_Pin GPIO_PIN_10
#define PC_RX_GPIO_Port GPIOC
#define PC_RXC11_Pin GPIO_PIN_11
#define PC_RXC11_GPIO_Port GPIOC
#define Switch_Pin GPIO_PIN_0
#define Switch_GPIO_Port GPIOD
#define SW_freq_Pin GPIO_PIN_2
#define SW_freq_GPIO_Port GPIOD
#define empty_battery_Pin GPIO_PIN_4
#define empty_battery_GPIO_Port GPIOD
#define Geneva_cal_sens_Pin GPIO_PIN_5
#define Geneva_cal_sens_GPIO_Port GPIOD
#define Geneva_dir_A_Pin GPIO_PIN_6
#define Geneva_dir_A_GPIO_Port GPIOD
#define Geneva_dir_B_Pin GPIO_PIN_7
#define Geneva_dir_B_GPIO_Port GPIOD
#define Geneva_CHB_Pin GPIO_PIN_3
#define Geneva_CHB_GPIO_Port GPIOB
#define bs_EXTI_Pin GPIO_PIN_4
#define bs_EXTI_GPIO_Port GPIOB
#define bs_EXTI_EXTI_IRQn EXTI4_IRQn
#define bs_nRST_Pin GPIO_PIN_5
#define bs_nRST_GPIO_Port GPIOB
#define Geneva_PWM_Pin GPIO_PIN_8
#define Geneva_PWM_GPIO_Port GPIOB
#define PWM_Dribbler_Pin GPIO_PIN_9
#define PWM_Dribbler_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
