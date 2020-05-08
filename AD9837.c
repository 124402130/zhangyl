#include "AD9837.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"
void DDS_10K_init()
{			uint8_t Data_DDS[2]={0x00,0x21};
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
      delay_us(5);
      HAL_SPI_Transmit(&hspi2,Data_DDS,1,waittime);
      delay_us(5);
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
}

void DDS_Frequent_set(uint32_t Frequent)
{

     uint8_t Control[2]={0x00,0x20};
		 uint8_t FreqL[2];
		 uint8_t FreqH[2];
		 uint32_t Freq={0x00000000};
		 Freq=(uint32_t)(268435456*1.0/16000000*Frequent);
		 FreqL[0]=(uint8_t)(Freq&(0xff));
		 FreqL[1]=(uint8_t)(((Freq&(0x3f00))>>8)|0x40);
     FreqH[0]=(uint8_t)((Freq&(0x3fc000))>>14);
		 FreqH[1]=(uint8_t)(((Freq&(0x00fc0000))>>28)|0x40);
		 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
		 delay_us(5);
		 HAL_SPI_Transmit(&hspi2,Control,1,waittime);
		 delay_us(5);
		 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
		 delay_us(5);
     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
		 delay_us(5);
		 HAL_SPI_Transmit(&hspi2,FreqL,1,waittime);
		 delay_us(5);
		 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
		 delay_us(5);
		 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
		 delay_us(5);
		 HAL_SPI_Transmit(&hspi2,FreqH,1,waittime);
		 delay_us(5);
		 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
		 
}	

