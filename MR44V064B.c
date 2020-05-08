#include "MR44V064B.h"
#include "i2c.h"
SMR44V064B_Data MR44V064B_Data;
void MR44V064B_Read(SMR44V064B_Data *pMR44V064B_Data,int sector,uint16_t Size)
{
  HAL_I2C_Mem_Read(&hi2c1,ADDR_MR44V064B,sector,I2C_MEMADD_SIZE_16BIT,& MR44V064B_Data.MR44V064B_Buff_RD[sector],Size,100);
}
void MR44V064B_Write(SMR44V064B_Data *pMR44V064B_Data,int sector,uint16_t Size)
{

  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_RESET);	
  HAL_I2C_Mem_Write(&hi2c1,ADDR_MR44V064B,sector,I2C_MEMADD_SIZE_16BIT,&MR44V064B_Data.MR44V064B_Buff_WR[sector],Size,100);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET);
}
