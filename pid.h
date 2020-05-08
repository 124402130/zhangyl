#ifndef __pid_H__
#define __pid_H__
#include <stdint.h>
#include "AD5541A.h"
#include "ADAU1701.h"
/******************* pid Constants ****************************************/

typedef struct 
{
	unsigned int flagFirst;
  unsigned int temp_op_bit;
	
	double	e1;       	
	double	e2;     			
	double	e3;        		
	
	
	double	kp;      		  
	double	ki;       		
	double	kd;   
	
	double p_value;
	double i_value;
	double d_value;
	
	
	uint16_t R;
	double R_hot_X;
	double R_hot_Y;
	double R_hot;
	double realtemp;
	double settemp;
	double settemp_rx;
	
	double uout;
	double dt;
	double set_Av;
	double real_Av;
	
	double i,p,d;	
	double realcurrent;

	
	uint8_t settemp_MAX;
	uint8_t settemp_MIN;
	
	uint8_t settemp_MAX_Real;
	uint8_t settemp_MIN_Real;
	
	double kpMax;
	double kpMin;
	double kiMax;
	double kiMin;
	double kdMax;
	double kdMin;
}Temp_pid;
typedef struct _SPID_Data
{
		Temp_pid temp[4];
} SPID_Data;




void PID_Init(SPID_Data *pPID_data,SDA_Data *pDA_Data);
void PID_Control(SPID_Data *pPID_data,SAD_Data *pAD_Data);
double pid(double realtemp , double settemp ,Temp_pid *temp,double dt);
double DOB(double realtemp,double u);
/************************ Functions Declarations *****************************/

#endif // __pid_H__
