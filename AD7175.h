#ifndef __AD7175_H__
#define __AD7175_H__
/***************************** Include Files *********************************/
#include <stdint.h>
#include "AD7175_regs.h"
/******************* AD7175 Constants ****************************************/
#define AD7175_CRC_POLYNOMIAL 0x07 // x^8 + x^2 + x +1 (MSB first)
#define AD7175_CRC_CHECK_CODE 0xF3
#define PIDFRQ 20
/************************ Functions Declarations *****************************/
void AD7175_Reset(void);
int32_t AD7175_Setup(void);
int32_t AD7175_ReadRegister(st_reg* pReg);
int32_t AD7175_WriteRegister(st_reg reg);
uint8_t AD7175_ComputeCRC(uint8_t* pBuf, uint8_t bufSize);
int32_t AD7175_WaitForReady(uint32_t timeout);
int32_t AD7175_ReadData(int32_t pData);
void Init_AD7175(void);
//typedef struct SAD_Data 
//{uint32_t CH[8];
//	
// double fCH[8];
// double fSumCH[8];	
// double fEvrCH[8];		
// uint32_t count;
// uint32_t evrN;
// double fTempCh[4];
// double fCurrCh[4];
//} SAD_Data;

#endif // __AD7175_H__
