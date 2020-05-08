#ifndef __MR44V064B_H__
#define __MR44V064B_H__
/***************************** Include Files *********************************/
#include <stdint.h>
/******************* AD5676 Constants ****************************************/
 #define ADDR_MR44V064B 0xAE


typedef struct _SMR44V064B_Data
{
   uint8_t MR44V064B_Buff_RD[100];
	 uint8_t MR44V064B_Buff_WR[100];
} SMR44V064B_Data;
		 

/************************ Functions Declarations *****************************/
void MR44V064B_Read(SMR44V064B_Data *pMR44V064B_Data,int sector,uint16_t Size);
void MR44V064B_Write(SMR44V064B_Data *pMR44V064B_Data,int sector,uint16_t Size);
#endif // __MR44V064B_H__
