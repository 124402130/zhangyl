#include "AD5541A.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"
#define waittime      10
uint8_t Data_DA1_Tx[2]={0x00,0x00};
uint8_t Data_DA2_Tx[2]={0x00,0x00};
SDA_Data DA_Data;
void DA1_SET(double Value)
{		   uint16_t set=(unsigned short)(Value/4.096*65536+0.5);
	     uint8_t  Data_DA1_Tx_L=set&(0xff);
	     uint8_t  Data_DA1_Tx_H=(set&(0xff00))>>8;
       Data_DA1_Tx[0]=Data_DA1_Tx_L;
       Data_DA1_Tx[1]=Data_DA1_Tx_H;
       HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_RESET);
			 HAL_SPI_Transmit(&hspi3,Data_DA1_Tx,1,waittime);
       HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0,GPIO_PIN_SET);
	     HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_RESET);
	     delay_us(500);
	     HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_SET);
}
void DA2_SET(double Value)
{		   uint16_t set=(unsigned short)(Value/4.096*65536+0.5);
	     uint8_t  Data_DA2_Tx_L=set&(0xff);
	     uint8_t  Data_DA2_Tx_H=(set&(0xff00))>>8;
       Data_DA2_Tx[0]=Data_DA2_Tx_L;
       Data_DA2_Tx[1]=Data_DA2_Tx_H;
       HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
			 HAL_SPI_Transmit(&hspi3,Data_DA2_Tx,1,waittime);
       HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
		   HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_RESET);
	     delay_us(500);
	     HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_SET);
}

void PWM1_start(double value,int time_ms)
{  
	DA1_SET(0.5);
	HAL_Delay(10);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
// HAL_Delay(time_ms);
// HAL_TIM_PWM_Stop(&htim5,TIM_CHANNEL_2);
}
void PWM2_start(double value,int time_ms)
{  
 DA2_SET(0.5);
 HAL_Delay(10);
 HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
// HAL_Delay(time_ms);
// HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);
}
