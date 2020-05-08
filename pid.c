#include "pid.h"
#include "usart.h"
#include "math.h"
#include "Com_Protocol.h"
#include "MR44V064B.h"
#include "tim.h"

SPID_Data PID_Data;
extern SAD_Data AD_Data;
extern SCom_Protocol Protocol_Data;
extern SDA_Data DA_Data;
extern SMR44V064B_Data MR44V064B_Data;
extern int list_R10K3CG[];
void PID_Init(SPID_Data *pPID_Data,SDA_Data *pDA_Data)
{
	uint8_t i;
	for(i=0;i<4;i++)
	{
	
		pPID_Data->temp[i].kp=MR44V064B_Data.MR44V064B_Buff_RD[3*i];
		pPID_Data->temp[i].ki=MR44V064B_Data.MR44V064B_Buff_RD[3*i+1];
		pPID_Data->temp[i].kd=MR44V064B_Data.MR44V064B_Buff_RD[3*i+2];

		pPID_Data->temp[i].realtemp=0.0f;
		pPID_Data->temp[i].settemp=30.0f;

		
		pPID_Data->temp[i].flagFirst=0;
		pPID_Data->temp[i].temp_op_bit=0;
		
	  DA_Data.laser_MAX[i]=0;
	  DA_Data.laser_MIN[i]=0;

	}
	pPID_Data->temp[0].R=1500;
	pPID_Data->temp[1].R=1500;
	pPID_Data->temp[2].R=1500;
	pPID_Data->temp[3].R=1500;
		
	pPID_Data->temp[0].settemp_MAX=MR44V064B_Data.MR44V064B_Buff_RD[28];
	pPID_Data->temp[0].settemp_MIN=MR44V064B_Data.MR44V064B_Buff_RD[29];
	pPID_Data->temp[1].settemp_MAX=MR44V064B_Data.MR44V064B_Buff_RD[30];
	pPID_Data->temp[1].settemp_MIN=MR44V064B_Data.MR44V064B_Buff_RD[31];
	pPID_Data->temp[2].settemp_MAX=MR44V064B_Data.MR44V064B_Buff_RD[32];
	pPID_Data->temp[2].settemp_MIN=MR44V064B_Data.MR44V064B_Buff_RD[33];
	pPID_Data->temp[3].settemp_MAX=MR44V064B_Data.MR44V064B_Buff_RD[34];
	pPID_Data->temp[3].settemp_MIN=MR44V064B_Data.MR44V064B_Buff_RD[35];
	
	
	pPID_Data->temp[0].settemp_MAX_Real=85;
	pPID_Data->temp[0].settemp_MIN_Real=10;
	pPID_Data->temp[1].settemp_MAX_Real=85;
	pPID_Data->temp[1].settemp_MIN_Real=10;
	pPID_Data->temp[2].settemp_MAX_Real=85;
	pPID_Data->temp[2].settemp_MIN_Real=10;
	pPID_Data->temp[3].settemp_MAX_Real=85;
	pPID_Data->temp[3].settemp_MIN_Real=10;
	
	
	DA_Data.laser_MAX_Real[0]=5;
  DA_Data.laser_MIN_Real[0]=0;
	DA_Data.laser_MAX_Real[1]=5;
	DA_Data.laser_MIN_Real[1]=0;
	DA_Data.laser_MAX_Real[2]=5;
  DA_Data.laser_MIN_Real[2]=0;
	DA_Data.laser_MAX_Real[3]=5;
	DA_Data.laser_MIN_Real[3]=0;
	
	if(pPID_Data->temp[0].settemp_MAX>=pPID_Data->temp[0].settemp_MAX_Real)
		    pPID_Data->temp[0].settemp_MAX=pPID_Data->temp[0].settemp_MAX_Real;
	if(pPID_Data->temp[1].settemp_MAX>=pPID_Data->temp[1].settemp_MAX_Real)
		    pPID_Data->temp[1].settemp_MAX=pPID_Data->temp[1].settemp_MAX_Real;
	if(pPID_Data->temp[2].settemp_MAX>=pPID_Data->temp[2].settemp_MAX_Real)
		    pPID_Data->temp[2].settemp_MAX=pPID_Data->temp[2].settemp_MAX_Real;
	if(pPID_Data->temp[3].settemp_MAX>=pPID_Data->temp[3].settemp_MAX_Real)
		    pPID_Data->temp[3].settemp_MAX=pPID_Data->temp[3].settemp_MAX_Real;
	
	
	if(pPID_Data->temp[0].settemp_MIN<=pPID_Data->temp[0].settemp_MIN_Real)
		    pPID_Data->temp[0].settemp_MIN=pPID_Data->temp[0].settemp_MIN_Real;
	if(pPID_Data->temp[1].settemp_MIN<=pPID_Data->temp[1].settemp_MIN_Real)
		    pPID_Data->temp[1].settemp_MIN=pPID_Data->temp[1].settemp_MIN_Real;
	if(pPID_Data->temp[2].settemp_MIN<=pPID_Data->temp[2].settemp_MIN_Real)
		    pPID_Data->temp[2].settemp_MIN=pPID_Data->temp[2].settemp_MIN_Real;
	if(pPID_Data->temp[3].settemp_MIN<=pPID_Data->temp[3].settemp_MIN_Real)
		    pPID_Data->temp[3].settemp_MIN=pPID_Data->temp[3].settemp_MIN_Real;
	 
	

	if((MR44V064B_Data.MR44V064B_Buff_RD[48]+(((uint16_t)MR44V064B_Data.MR44V064B_Buff_RD[49]<<8)+MR44V064B_Data.MR44V064B_Buff_RD[50])*1.0f/1000<=pPID_Data->temp[0].settemp_MAX)&&(MR44V064B_Data.MR44V064B_Buff_RD[48]+(((uint16_t)MR44V064B_Data.MR44V064B_Buff_RD[49]<<8)+MR44V064B_Data.MR44V064B_Buff_RD[50])*1.0f/1000>=pPID_Data->temp[0].settemp_MIN))
	{
		pPID_Data->temp[0].settemp=MR44V064B_Data.MR44V064B_Buff_RD[48]+(((uint16_t)MR44V064B_Data.MR44V064B_Buff_RD[49]<<8)+MR44V064B_Data.MR44V064B_Buff_RD[50])*1.0f/1000;
	}
	if((MR44V064B_Data.MR44V064B_Buff_RD[51]+(((uint16_t)MR44V064B_Data.MR44V064B_Buff_RD[52]<<8)+MR44V064B_Data.MR44V064B_Buff_RD[53])*1.0f/1000<=pPID_Data->temp[1].settemp_MAX)&&(MR44V064B_Data.MR44V064B_Buff_RD[51]+(((uint16_t)MR44V064B_Data.MR44V064B_Buff_RD[52]<<8)+MR44V064B_Data.MR44V064B_Buff_RD[53])*1.0f/1000>=pPID_Data->temp[1].settemp_MIN))
	{
	}
		pPID_Data->temp[1].settemp=MR44V064B_Data.MR44V064B_Buff_RD[51]+(((uint16_t)MR44V064B_Data.MR44V064B_Buff_RD[52]<<8)+MR44V064B_Data.MR44V064B_Buff_RD[53])*1.0f/1000;
	if((MR44V064B_Data.MR44V064B_Buff_RD[54]+(((uint16_t)MR44V064B_Data.MR44V064B_Buff_RD[55]<<8)+MR44V064B_Data.MR44V064B_Buff_RD[56])*1.0f/1000<=pPID_Data->temp[2].settemp_MAX)&&(MR44V064B_Data.MR44V064B_Buff_RD[54]+(((uint16_t)MR44V064B_Data.MR44V064B_Buff_RD[55]<<8)+MR44V064B_Data.MR44V064B_Buff_RD[56])*1.0f/1000>=pPID_Data->temp[2].settemp_MIN))
	{
		pPID_Data->temp[2].settemp=MR44V064B_Data.MR44V064B_Buff_RD[54]+(((uint16_t)MR44V064B_Data.MR44V064B_Buff_RD[55]<<8)+MR44V064B_Data.MR44V064B_Buff_RD[56])*1.0f/1000;
	}
	if((MR44V064B_Data.MR44V064B_Buff_RD[57]+(((uint16_t)MR44V064B_Data.MR44V064B_Buff_RD[58]<<8)+MR44V064B_Data.MR44V064B_Buff_RD[59])*1.0f/1000<=pPID_Data->temp[3].settemp_MAX)&&(MR44V064B_Data.MR44V064B_Buff_RD[57]+(((uint16_t)MR44V064B_Data.MR44V064B_Buff_RD[58]<<8)+MR44V064B_Data.MR44V064B_Buff_RD[59])*1.0f/1000>=pPID_Data->temp[3].settemp_MIN))
	{
		pPID_Data->temp[3].settemp=MR44V064B_Data.MR44V064B_Buff_RD[57]+(((uint16_t)MR44V064B_Data.MR44V064B_Buff_RD[58]<<8)+MR44V064B_Data.MR44V064B_Buff_RD[59])*1.0f/1000;
	}
 	for(i=0;i<4;i++)
   { 

		 PID_Data.temp[i].R_hot_X=1.0/8.762e-8*(0.001129-1.0/(PID_Data.temp[i].settemp+273.15));
		 PID_Data.temp[i].R_hot_Y=sqrt(pow(0.0002341/(3*8.762e-8),3)+pow(PID_Data.temp[i].R_hot_X/2.0,2));
		 PID_Data.temp[i].R_hot=exp(pow(PID_Data.temp[i].R_hot_Y-PID_Data.temp[i].R_hot_X/2.0,1.0/3)-pow(PID_Data.temp[i].R_hot_Y+PID_Data.temp[i].R_hot_X/2.0,1.0/3));
	   PID_Data.temp[i].set_Av=(PID_Data.temp[i].R_hot-PID_Data.temp[i].R)/(PID_Data.temp[i].R_hot+PID_Data.temp[i].R)*0.5*(6000.0/120+1.0);
	 }
	
}

double calculate_Temp(double real_Av,double R)
{

	double R_hot_mid,R_hot,temp,R_log;
	R_hot_mid=real_Av/0.5/(6000.0/120.0+1.0);	
	R_hot=(1+R_hot_mid)*R/(1-R_hot_mid);
	R_log=log(R_hot);
  temp=1.0/(8.762e-8*R_log*R_log*R_log+2.341e-4*R_log+1.129e-3)-273.15;
	return temp;
}
double DOB(double realtemp,double u)
{
	realtemp=1;
	return 0;
}
void PID_Control(SPID_Data *pPID_Data,SAD_Data *pAD_Data)
{

	pPID_Data->temp[0].realtemp=calculate_Temp(pPID_Data->temp[0].real_Av,pPID_Data->temp[0].R);
	pPID_Data->temp[1].realtemp=calculate_Temp(pPID_Data->temp[1].real_Av,pPID_Data->temp[1].R);
	pPID_Data->temp[2].realtemp=calculate_Temp(pPID_Data->temp[2].real_Av,pPID_Data->temp[2].R);
	pPID_Data->temp[3].realtemp=calculate_Temp(pPID_Data->temp[3].real_Av,pPID_Data->temp[3].R);	
   	
	/*********************温度保护机制****************************/
	
	if((pPID_Data->temp[0].realtemp>PID_Data.temp[0].settemp_MAX)&&(PID_Data.temp[0].temp_op_bit==0))
	{     
     ADAU1701_Res_Set(0,ADAU1701_Channel1,0x0814);	
     PID_Data.temp[0].temp_op_bit++;
	}

	if((pPID_Data->temp[1].realtemp>PID_Data.temp[1].settemp_MAX)&&(PID_Data.temp[1].temp_op_bit==0))
	{
     ADAU1701_Res_Set(0,ADAU1701_Channel2,0x0814);
     PID_Data.temp[1].temp_op_bit++;
	}
		if((pPID_Data->temp[2].realtemp>PID_Data.temp[2].settemp_MAX)&&(PID_Data.temp[2].temp_op_bit==0))
	{
      ADAU1701_Res_Set(0,ADAU1701_Channel3,0x0814);
      PID_Data.temp[2].temp_op_bit++;
	}
		if((pPID_Data->temp[3].realtemp>PID_Data.temp[3].settemp_MAX)&&(PID_Data.temp[3].temp_op_bit==0))
	{
      ADAU1701_Res_Set(0,ADAU1701_Channel4,0x0814);
      PID_Data.temp[3].temp_op_bit++;
	}
	
	
	if(PID_Data.temp[0].temp_op_bit>=1)
	{	
		PID_Data.temp[0].temp_op_bit++;
				if(PID_Data.temp[0].temp_op_bit==1000)
				{
					ADAU1701_Res_Set(1,ADAU1701_Channel1,0x0814);
					PID_Data.temp[0].temp_op_bit=0;
				}
		
	}
	if(PID_Data.temp[1].temp_op_bit>=1)
	{
		PID_Data.temp[1].temp_op_bit++;
				 if(PID_Data.temp[1].temp_op_bit==1000)
				 {
					ADAU1701_Res_Set(1,ADAU1701_Channel2,0x0814);
					PID_Data.temp[1].temp_op_bit=0;
				 }
	}
				
	if(PID_Data.temp[2].temp_op_bit>=1)
	{

		PID_Data.temp[2].temp_op_bit++;
				if(PID_Data.temp[2].temp_op_bit==1000)
				{
					ADAU1701_Res_Set(1,ADAU1701_Channel3,0x0814);
					PID_Data.temp[2].temp_op_bit=0;
				}
		
	}
	if(PID_Data.temp[3].temp_op_bit>=1)
	{
		PID_Data.temp[3].temp_op_bit++;
				if(PID_Data.temp[3].temp_op_bit==1000)
				{
					ADAU1701_Res_Set(1,ADAU1701_Channel4,0x0814);
					PID_Data.temp[3].temp_op_bit=0;
				}
	}
 
	/*********************协议*****************************/
	Protocol_Data.Transmit_BUFF[27]++;
	for(int i=0;i<3;i++)          //给一路看板上温度
	{
	Protocol_Data.Transmit_INT[i]=(uint8_t)AD_Data.dbl_EvrCH_current[i];
	Protocol_Data.Transmit_decimal[i]=(uint16_t)((AD_Data.dbl_EvrCH_current[i]-Protocol_Data.Transmit_INT[i])/0.02f*1000);
	Protocol_Data.Transmit_decimal_H[i]=(uint8_t)(Protocol_Data.Transmit_decimal[i]>>8);
	Protocol_Data.Transmit_decimal_L[i]=(uint8_t)(Protocol_Data.Transmit_decimal[i]&0xFF);
	}
	for(int i=3;i<4;i++)          //给一路看板上温度
	{
	Protocol_Data.Transmit_INT[i]=(uint8_t)AD_Data.dbl_EvrCH_boardtemp[0];
	Protocol_Data.Transmit_decimal[i]=(uint16_t)((AD_Data.dbl_EvrCH_boardtemp[0]-Protocol_Data.Transmit_INT[i])/0.02f*1000);
	Protocol_Data.Transmit_decimal_H[i]=(uint8_t)(Protocol_Data.Transmit_decimal[i]>>8);
	Protocol_Data.Transmit_decimal_L[i]=(uint8_t)(Protocol_Data.Transmit_decimal[i]&0xFF);
	}
  for(int i=4;i<8;i++)
	{
	Protocol_Data.Transmit_INT[i]=(uint8_t)pPID_Data->temp[i-4].realtemp;
	Protocol_Data.Transmit_decimal[i]=(uint16_t)((pPID_Data->temp[i-4].realtemp-Protocol_Data.Transmit_INT[i])/0.02f*1000);
	Protocol_Data.Transmit_decimal_H[i]=(uint8_t)(Protocol_Data.Transmit_decimal[i]>>8);
	Protocol_Data.Transmit_decimal_L[i]=(uint8_t)(Protocol_Data.Transmit_decimal[i]&0xFF);
	}
	 for(int i=1;i<=8;i++)
	{
		Protocol_Data.Transmit_BUFF[3*i]=Protocol_Data.Transmit_INT[i-1];
	}
	 for(int i=1;i<=8;i++)
	{
		Protocol_Data.Transmit_BUFF[3*i+1]=Protocol_Data.Transmit_decimal_H[i-1];
	}
	 for(int i=1;i<=8;i++)
	{
		Protocol_Data.Transmit_BUFF[3*i+2]=Protocol_Data.Transmit_decimal_L[i-1];
	}

	for(int i=3;i<28;i++)
	{
		Protocol_Data.Transmit_BUFF[28]+=Protocol_Data.Transmit_BUFF[i];
	}
	
   HAL_UART_Transmit_DMA(&huart3,&Protocol_Data.Transmit_BUFF[0],29);
	 Protocol_Data.Transmit_BUFF[28]=0;
 }

 
 
 



