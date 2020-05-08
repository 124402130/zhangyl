/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "AD5541A.h"
#include "AD9837.h"
#include "pid.h"
#include "ADAU1701.h"
#include "Com_Protocol.h"
#include "MR44V064B.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

  
	extern  uint8_t ProgramReg_temp[5120];
	extern  uint8_t ParaReg_temp[4096];
	extern  uint8_t HWConfigReg_temp[24];
	extern  uint8_t CoreReg_temp_1[2];
	extern  uint8_t CoreReg_temp_2[2];
	extern  uint8_t ProgramReg_current[5120];
	extern  uint8_t ParaReg_current[4096];
	extern  uint8_t HWConfigReg_current[24];
	extern  uint8_t CoreReg_current_1[2];
	extern  uint8_t CoreReg_current_2[2];
	extern  uint8_t ProgramReg_board_temp[5120];
	extern  uint8_t ParaReg_board_temp[4096];
	extern  uint8_t HWConfigReg_board_temp[24];
	extern  uint8_t CoreReg_board_temp_1[2];
	extern  uint8_t CoreReg_board_temp_2[2];
  extern  uint8_t Read_Com[][2];
	
	
  extern SAD_Data AD_Data;
	extern SPID_Data PID_Data;
	extern SDA_Data DA_Data;
	extern SCom_Protocol Protocol_Data;
	extern SMR44V064B_Data MR44V064B_Data;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM14_Init();
  MX_I2C3_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_I2C_MspInit(&hi2c1);
	HAL_I2C_MspInit(&hi2c2);
	HAL_I2C_MspInit(&hi2c3);
	HAL_SPI_MspInit(&hspi2);
  HAL_SPI_MspInit(&hspi3);
	MR44V064B_Read(&MR44V064B_Data,0,60);
	Com_Protocol_Init(&Protocol_Data);
	DDS_10K_init();
  DDS_Frequent_set(10000);
	HAL_SPI_DeInit(&hspi2);	
	PID_Init(&PID_Data,&DA_Data);
	DA1_SET(0);
	DA2_SET(0);                //电流源满量程2.88MA
	delay_us(100);
  ADAU1701_Setup(ADDR_ADAU1701_1,&CoreReg_temp_1[0],&ProgramReg_temp[0],&ParaReg_temp[0],&HWConfigReg_temp[0],&CoreReg_temp_2[0]);
  ADAU1701_Setup(ADDR_ADAU1701_2,&CoreReg_temp_1[0],&ProgramReg_temp[0],&ParaReg_temp[0],&HWConfigReg_temp[0],&CoreReg_temp_2[0]);
  ADAU1701_Setup(ADDR_ADAU1701_3,&CoreReg_temp_1[0],&ProgramReg_temp[0],&ParaReg_temp[0],&HWConfigReg_temp[0],&CoreReg_temp_2[0]);
	ADAU1701_Setup(ADDR_ADAU1701_4,&CoreReg_temp_1[0],&ProgramReg_temp[0],&ParaReg_temp[0],&HWConfigReg_temp[0],&CoreReg_temp_2[0]);
	ADAU1701_Setup(ADDR_ADAU1701_5,&CoreReg_current_1[0],&ProgramReg_current[0],&ParaReg_current[0],&HWConfigReg_current[0],&CoreReg_current_2[0]);
	ADAU1701_Setup(ADDR_ADAU1701_6,&CoreReg_board_temp_1[0],&ProgramReg_board_temp[0],&ParaReg_board_temp[0],&HWConfigReg_board_temp[0],&CoreReg_board_temp_2[0]);
	AD_Data.count = 0;
	AD_Data.evrN =1;
	HAL_TIM_Base_MspInit(&htim3);
	HAL_TIM_Base_Start_IT(&htim3); //开启定时器读取数据
	HAL_UART_Receive_IT(&huart3,&Protocol_Data.Receive_BUFF[0],7);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
