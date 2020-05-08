#include "Com_Protocol.h"
SCom_Protocol Protocol_Data;
void Com_Protocol_Init(SCom_Protocol *pProtocol_Data)
{   
	  int i;
		Protocol_Data.flag_start=0;
	  for(i=0;i<7;i++)
	 {
	  Protocol_Data.Receive_BUFF[i]=0;
	 }
	 	for(i=0;i<8;i++)
	 {
	  Protocol_Data.Transmit_INT[i]=0;
		Protocol_Data.Transmit_decimal[i]=0;
		Protocol_Data.Transmit_decimal_H[i]=0;
	  Protocol_Data.Transmit_decimal_L[i]=0;
		 
	 }
		Protocol_Data.Transmit_BUFF[0]=0x55;
	  Protocol_Data.Transmit_BUFF[1]=0xAA;
		Protocol_Data.Transmit_BUFF[2]=0x1A;
	 	  for(i=3;i<30;i++)
	 {
	  Protocol_Data.Transmit_BUFF[i]=0x00;
	 }

}
