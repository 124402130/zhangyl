#ifndef __AD5541A_H__
#define __AD5541A_H__
/***************************** Include Files *********************************/
#include <stdint.h>
/******************* AD5676 Constants ****************************************/
		 
typedef struct _SDA_Data
{
  uint8_t laser_MAX[4];
	uint8_t laser_MIN[4];
	uint8_t laser_MAX_Real[4];
	uint8_t laser_MIN_Real[4];
	uint8_t laser_mA[4];
	uint16_t laser_uA[4];
	uint8_t laser_mA_rx[4];
	uint16_t laser_uA_rx[4];
  uint8_t laser_DA_start_bit[4];
	uint8_t k_start[4];
} SDA_Data;


		 

/************************ Functions Declarations *****************************/
void DA1_SET(double Value);
void DA2_SET(double Value);		 
void PWM1_start(double value,int time_ms);
void PWM2_start(double value,int time_ms);
#endif // __AD5676_H__
