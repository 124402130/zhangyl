#ifndef __Com_Protocol_H__
#define __Com_Protocol_H__
/***************************** Include Files *********************************/
#include <stdint.h>
/******************* Com Protocol Constants ****************************************/
typedef struct _SCom_Protocol
{
  uint8_t  flag_start;
	uint8_t  Receive_BUFF[7];
	uint8_t  Transmit_INT[8];
	uint16_t Transmit_decimal[8];
	uint8_t  Transmit_decimal_H[8];
	uint8_t  Transmit_decimal_L[8];
	uint8_t  Transmit_BUFF[30];
}SCom_Protocol;

	
/************************ Functions Declarations *****************************/
void Com_Protocol_Init(SCom_Protocol *pProtocol_Data);


#endif // __Com_Protocol_H__

