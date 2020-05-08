#ifndef __ADAU1701_H__
#define __ADAU1701_H__
/***************************** Include Files *********************************/
#include <stdint.h>
#include "i2c.h"
/******************* ADAU1701 Constants ****************************************/
#define ADDR_ADAU1701_1 0x68 
#define ADDR_ADAU1701_2 0x6A
#define ADDR_ADAU1701_3 0x6C 
#define ADDR_ADAU1701_4 0x6E
#define ADDR_ADAU1701_5 0x70
#define ADDR_ADAU1701_6 0x72
#define ADAU1701_Channel1 1-1 
#define ADAU1701_Channel2 2-1
#define ADAU1701_Channel3 3-1
#define ADAU1701_Channel4 4-1
enum Read_Com_Protocol
{
    ADAU1701_temp1_Com=00,
	  ADAU1701_temp2_Com,
	  ADAU1701_temp3_Com,
	  ADAU1701_temp4_Com,
	  ADAU1701_current1_Com,
    ADAU1701_current2_Com,
		ADAU1701_boardtemp_Com, 
};
typedef struct SAD_Data 
{
 uint32_t count;
 uint32_t evrN;
	

	
 double value[7];
		
	
 double dbl_CH_Av[4];
 double dbl_current[2];
 double dbl_boardtemp[1];
	
 double dbl_SumCH_Av[4];	
 double dbl_EvrCH_Av[4];	
 double dbl_SumCH_current[4];	
 double dbl_EvrCH_current[4];
 double dbl_SumCH_boardtemp[1];	
 double dbl_EvrCH_boardtemp[1];	
	

} SAD_Data;

/************************ Functions Declarations *****************************/
void ADAU1701_Setup(uint8_t ADDR,uint8_t *CoreReg_1,uint8_t *ProgramReg,uint8_t *ParaReg,uint8_t *HWConfigReg,uint8_t *CoreReg_2);
void ADAU1701_ReadData(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,uint8_t Com,SAD_Data *pAD_Data);
void ADAU1701_Res_Set(double value,uint8_t ADAU1701_Channel,uint16_t Safeload_ADDR);




#endif // __ADAU1701_H__
