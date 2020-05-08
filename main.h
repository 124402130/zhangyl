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
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DDS_CS_Pin GPIO_PIN_1
#define DDS_CS_GPIO_Port GPIOA
#define LD1_Pulse_Pin GPIO_PIN_2
#define LD1_Pulse_GPIO_Port GPIOA
#define LD2_Pulse_Pin GPIO_PIN_3
#define LD2_Pulse_GPIO_Port GPIOA
#define ADAU1401A_RESET_1_Pin GPIO_PIN_7
#define ADAU1401A_RESET_1_GPIO_Port GPIOE
#define ADAU1401A_RESET_2_Pin GPIO_PIN_8
#define ADAU1401A_RESET_2_GPIO_Port GPIOE
#define ADAU1401A_RESET_3_Pin GPIO_PIN_9
#define ADAU1401A_RESET_3_GPIO_Port GPIOE
#define ADAU1401A_RESET_4_Pin GPIO_PIN_10
#define ADAU1401A_RESET_4_GPIO_Port GPIOE
#define ADAU1401A_RESET_5_Pin GPIO_PIN_11
#define ADAU1401A_RESET_5_GPIO_Port GPIOE
#define ADAU1401A_RESET_6_Pin GPIO_PIN_12
#define ADAU1401A_RESET_6_GPIO_Port GPIOE
#define DA1_CS_Pin GPIO_PIN_0
#define DA1_CS_GPIO_Port GPIOD
#define DA1_LOAD_Pin GPIO_PIN_1
#define DA1_LOAD_GPIO_Port GPIOD
#define DA2_CS_Pin GPIO_PIN_2
#define DA2_CS_GPIO_Port GPIOD
#define DA2_LOAD_Pin GPIO_PIN_3
#define DA2_LOAD_GPIO_Port GPIOD
#define LD1_On_Pin GPIO_PIN_6
#define LD1_On_GPIO_Port GPIOD
#define LD2_On_Pin GPIO_PIN_7
#define LD2_On_GPIO_Port GPIOD
#define I2C1_SCL_ROM_SCL_Pin GPIO_PIN_6
#define I2C1_SCL_ROM_SCL_GPIO_Port GPIOB
#define I2C1_SDA_ROM_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_ROM_SDA_GPIO_Port GPIOB
#define ROM_WP_Pin GPIO_PIN_0
#define ROM_WP_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */
 
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
