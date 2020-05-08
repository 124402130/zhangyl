/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "Com_protocol.h"
#include "pid.h"
#include "AD5541A.h"
#include "MR44V064B.h"
#include "R10K3CG.h"
#include "math.h"
extern   SCom_Protocol Protocol_Data;
extern   SDA_Data DA_Data;
extern   SPID_Data PID_Data;
extern SMR44V064B_Data MR44V064B_Data;

/* USER CODE END 0 */

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();
  
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**USART3 GPIO Configuration    
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USART3 DMA Init */
    /* USART3_TX Init */
    hdma_usart3_tx.Instance = DMA1_Stream3;
    hdma_usart3_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_tx.Init.Mode = DMA_NORMAL;
    hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart3_tx);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();
  
    /**USART3 GPIO Configuration    
    PD8     ------> USART3_TX
    PD9     ------> USART3_RX 
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8|GPIO_PIN_9);

    /* USART3 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
if(huart->Instance==USART3)
	{
    if((Protocol_Data.Receive_BUFF[0]!=0x55)&&(Protocol_Data.flag_start==0))
			 HAL_UART_Receive_IT(&huart3,&Protocol_Data.Receive_BUFF[0],7);
    else
			Protocol_Data.flag_start=1;
    if(Protocol_Data.flag_start==1)
		 {
      if((Protocol_Data.Receive_BUFF[1]==0xAA)&&(Protocol_Data.Receive_BUFF[6]==0x00))
		   {
				 switch(Protocol_Data.Receive_BUFF[2])
				     {
							 case 0xC1:
						           DA_Data.laser_mA_rx[0]=Protocol_Data.Receive_BUFF[3];
				               DA_Data.laser_uA_rx[0]=((uint16_t)Protocol_Data.Receive_BUFF[4]<<8)+Protocol_Data.Receive_BUFF[5];
				               {
												 if((DA_Data.laser_mA_rx[0]!=0)||(DA_Data.laser_uA_rx[0]!=0))
								          {
									             if((DA_Data.laser_mA_rx[0]+DA_Data.laser_uA_rx[0]*1.0f/1000)<=DA_Data.laser_MAX[0])
									                {								
																	 DA_Data.laser_mA[0]=DA_Data.laser_mA_rx[0];
																	 DA_Data.laser_uA[0]=DA_Data.laser_uA_rx[0];
									                 HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_SET);
									                 DA_Data.laser_DA_start_bit[0]=1;																			
                                   MR44V064B_Data.MR44V064B_Buff_WR[36]=Protocol_Data.Receive_BUFF[3];
				                           MR44V064B_Data.MR44V064B_Buff_WR[37]=Protocol_Data.Receive_BUFF[4];	
				                           MR44V064B_Data.MR44V064B_Buff_WR[38]=Protocol_Data.Receive_BUFF[5];
													         MR44V064B_Write(&MR44V064B_Data,36,3);	
								                  }	
								          }	
													else	
															 DA1_SET(0);
												  break;
				       case 0xC2:
						           DA_Data.laser_mA_rx[1]=Protocol_Data.Receive_BUFF[3];
				               DA_Data.laser_uA_rx[1]=((uint16_t)Protocol_Data.Receive_BUFF[4]<<8)+Protocol_Data.Receive_BUFF[5];
				               if((DA_Data.laser_mA_rx[1]!=0)||(DA_Data.laser_uA_rx[1]!=0))
				                {
									          if((DA_Data.laser_mA_rx[1]+DA_Data.laser_uA_rx[1]*1.0f/1000)<=DA_Data.laser_MAX[1])
	                               {
																	DA_Data.laser_mA[1]=DA_Data.laser_mA_rx[1];
																	DA_Data.laser_uA[1]=DA_Data.laser_uA_rx[1];
																	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_SET);
									                DA_Data.laser_DA_start_bit[1]=1;	
								                  MR44V064B_Data.MR44V064B_Buff_WR[39]=Protocol_Data.Receive_BUFF[3];
				                          MR44V064B_Data.MR44V064B_Buff_WR[40]=Protocol_Data.Receive_BUFF[4];	
				                          MR44V064B_Data.MR44V064B_Buff_WR[41]=Protocol_Data.Receive_BUFF[5];
                                  MR44V064B_Write(&MR44V064B_Data,39,3);
								                 }	
							          }
											 	else	
															 DA2_SET(0);
					              break;								
				        case 0xC5:
					                 PID_Data.temp[0].settemp_rx=Protocol_Data.Receive_BUFF[3]+(((uint16_t)Protocol_Data.Receive_BUFF[4]<<8)+Protocol_Data.Receive_BUFF[5])*1.0f/1000;
						               if((PID_Data.temp[0].settemp_rx<=PID_Data.temp[0].settemp_MAX)&&(PID_Data.temp[0].settemp_rx>=PID_Data.temp[0].settemp_MIN))
						                {
								             PID_Data.temp[0].settemp=PID_Data.temp[0].settemp_rx;
                             PID_Data.temp[0].R_hot_X=1.0/8.762e-8*(0.001129-1.0/(PID_Data.temp[0].settemp+273.15));
		                         PID_Data.temp[0].R_hot_Y=sqrt(pow(0.0002341/(3*8.762e-8),3)+pow(PID_Data.temp[0].R_hot_X/2.0,2));
		                         PID_Data.temp[0].R_hot=exp(pow(PID_Data.temp[0].R_hot_Y-PID_Data.temp[0].R_hot_X/2.0,1.0/3)-pow(PID_Data.temp[0].R_hot_Y+PID_Data.temp[0].R_hot_X/2.0,1.0/3));
	                           PID_Data.temp[0].set_Av=(PID_Data.temp[0].R_hot-PID_Data.temp[0].R)/(PID_Data.temp[0].R_hot+PID_Data.temp[0].R)*0.5*(6000.0/120+1.0);
														 ADAU1701_Res_Set(PID_Data.temp[0].set_Av,ADAU1701_Channel1,0x0810);
								             MR44V064B_Data.MR44V064B_Buff_WR[48]=Protocol_Data.Receive_BUFF[3];
								             MR44V064B_Data.MR44V064B_Buff_WR[49]=Protocol_Data.Receive_BUFF[4];
								             MR44V064B_Data.MR44V064B_Buff_WR[50]=Protocol_Data.Receive_BUFF[5];
							               MR44V064B_Write(&MR44V064B_Data,48,3);
														
							            	}				
					               break;
				         case 0xC6:
					                 PID_Data.temp[1].settemp_rx=Protocol_Data.Receive_BUFF[3]+(((uint16_t)Protocol_Data.Receive_BUFF[4]<<8)+Protocol_Data.Receive_BUFF[5])*1.0f/1000;
						               if((PID_Data.temp[1].settemp_rx<=PID_Data.temp[1].settemp_MAX)&&(PID_Data.temp[1].settemp_rx>=PID_Data.temp[1].settemp_MIN))
						               {
								              PID_Data.temp[1].settemp=PID_Data.temp[1].settemp_rx;
														  PID_Data.temp[1].R_hot_X=1.0/8.762e-8*(0.001129-1.0/(PID_Data.temp[1].settemp+273.15));
		                          PID_Data.temp[1].R_hot_Y=sqrt(pow(0.0002341/(3*8.762e-8),3)+pow(PID_Data.temp[1].R_hot_X/2.0,2));
		                          PID_Data.temp[1].R_hot=exp(pow(PID_Data.temp[1].R_hot_Y-PID_Data.temp[1].R_hot_X/2.0,1.0/3)-pow(PID_Data.temp[1].R_hot_Y+PID_Data.temp[1].R_hot_X/2.0,1.0/3));
	                            PID_Data.temp[1].set_Av=(PID_Data.temp[1].R_hot-PID_Data.temp[1].R)/(PID_Data.temp[1].R_hot+PID_Data.temp[1].R)*0.5*(6000.0/120+1.0);
															ADAU1701_Res_Set(PID_Data.temp[1].set_Av,ADAU1701_Channel2,0x0810);
								              MR44V064B_Data.MR44V064B_Buff_WR[51]=Protocol_Data.Receive_BUFF[3];
								              MR44V064B_Data.MR44V064B_Buff_WR[52]=Protocol_Data.Receive_BUFF[4];
								              MR44V064B_Data.MR44V064B_Buff_WR[53]=Protocol_Data.Receive_BUFF[5];
                              MR44V064B_Write(&MR44V064B_Data,51,3);
								            }				
					                break;	
				             case 0xC7:
					                 PID_Data.temp[2].settemp_rx=Protocol_Data.Receive_BUFF[3]+(((uint16_t)Protocol_Data.Receive_BUFF[4]<<8)+Protocol_Data.Receive_BUFF[5])*1.0f/1000;
						               if((PID_Data.temp[2].settemp_rx<=PID_Data.temp[2].settemp_MAX)&&(PID_Data.temp[2].settemp_rx>=PID_Data.temp[2].settemp_MIN))
						                {
								              PID_Data.temp[2].settemp=PID_Data.temp[2].settemp_rx;
						                  PID_Data.temp[2].R_hot_X=1.0/8.762e-8*(0.001129-1.0/(PID_Data.temp[2].settemp+273.15));
		                          PID_Data.temp[2].R_hot_Y=sqrt(pow(0.0002341/(3*8.762e-8),3)+pow(PID_Data.temp[2].R_hot_X/2.0,2));
		                          PID_Data.temp[2].R_hot=exp(pow(PID_Data.temp[2].R_hot_Y-PID_Data.temp[2].R_hot_X/2.0,1.0/3)-pow(PID_Data.temp[2].R_hot_Y+PID_Data.temp[2].R_hot_X/2.0,1.0/3));
	                            PID_Data.temp[2].set_Av=(PID_Data.temp[2].R_hot-PID_Data.temp[2].R)/(PID_Data.temp[2].R_hot+PID_Data.temp[2].R)*0.5*(6000.0/120+1.0);
															ADAU1701_Res_Set(PID_Data.temp[2].set_Av,ADAU1701_Channel3,0x0810);
								              MR44V064B_Data.MR44V064B_Buff_WR[54]=Protocol_Data.Receive_BUFF[3];
								              MR44V064B_Data.MR44V064B_Buff_WR[55]=Protocol_Data.Receive_BUFF[4];
								              MR44V064B_Data.MR44V064B_Buff_WR[56]=Protocol_Data.Receive_BUFF[5];
                              MR44V064B_Write(&MR44V064B_Data,54,3);
								             }				
					                 break;
				              case 0xC8:
					                 PID_Data.temp[3].settemp_rx=Protocol_Data.Receive_BUFF[3]+(((uint16_t)Protocol_Data.Receive_BUFF[4]<<8)+Protocol_Data.Receive_BUFF[5])*1.0f/1000;
						               if((PID_Data.temp[3].settemp_rx<=PID_Data.temp[3].settemp_MAX)&&(PID_Data.temp[3].settemp_rx>=PID_Data.temp[3].settemp_MIN))
						                  {
								                PID_Data.temp[3].settemp=PID_Data.temp[3].settemp_rx;
															  PID_Data.temp[3].R_hot_X=1.0/8.762e-8*(0.001129-1.0/(PID_Data.temp[3].settemp+273.15));
		                            PID_Data.temp[3].R_hot_Y=sqrt(pow(0.0002341/(3*8.762e-8),3)+pow(PID_Data.temp[3].R_hot_X/2.0,2));
		                            PID_Data.temp[3].R_hot=exp(pow(PID_Data.temp[3].R_hot_Y-PID_Data.temp[3].R_hot_X/2.0,1.0/3)-pow(PID_Data.temp[3].R_hot_Y+PID_Data.temp[3].R_hot_X/2.0,1.0/3));
	                              PID_Data.temp[3].set_Av=(PID_Data.temp[3].R_hot-PID_Data.temp[3].R)/(PID_Data.temp[3].R_hot+PID_Data.temp[3].R)*0.5*(6000.0/120+1.0);
																ADAU1701_Res_Set(PID_Data.temp[3].set_Av,ADAU1701_Channel4,0x0810);
								                MR44V064B_Data.MR44V064B_Buff_WR[57]=Protocol_Data.Receive_BUFF[3];
                                MR44V064B_Data.MR44V064B_Buff_WR[58]=Protocol_Data.Receive_BUFF[4];
								                MR44V064B_Data.MR44V064B_Buff_WR[59]=Protocol_Data.Receive_BUFF[5];
                                MR44V064B_Write(&MR44V064B_Data,57,3);
                              }				
					                  break;										
					             case 0xD1:
						                DA_Data.laser_MAX[0]=Protocol_Data.Receive_BUFF[3];
											 	    if(DA_Data.laser_MAX[0]>=DA_Data.laser_MAX_Real[0])
														   DA_Data.laser_MAX[0]=DA_Data.laser_MAX_Real[0];
				                    MR44V064B_Data.MR44V064B_Buff_WR[24]=DA_Data.laser_MAX[0];
                            MR44V064B_Write(&MR44V064B_Data,24,1);		
					                  break;
				               case 0xD2:
						                DA_Data.laser_MAX[1]=Protocol_Data.Receive_BUFF[3];
											 			if(DA_Data.laser_MAX[1]>=DA_Data.laser_MAX_Real[1])
														   DA_Data.laser_MAX[1]=DA_Data.laser_MAX_Real[1];
				                    MR44V064B_Data.MR44V064B_Buff_WR[25]=DA_Data.laser_MAX[1];
                            MR44V064B_Write(&MR44V064B_Data,25,1);	      				
					                  break;

				               case 0xD5:
						                PID_Data.temp[0].settemp_MAX=Protocol_Data.Receive_BUFF[3];	
                            PID_Data.temp[0].settemp_MIN=Protocol_Data.Receive_BUFF[4];
											 			if(PID_Data.temp[0].settemp_MAX>=PID_Data.temp[0].settemp_MAX_Real)
												         {PID_Data.temp[0].settemp_MAX=PID_Data.temp[0].settemp_MAX_Real;}
												    if(PID_Data.temp[0].settemp_MIN<=PID_Data.temp[0].settemp_MIN_Real)
												         {PID_Data.temp[0].settemp_MIN=PID_Data.temp[0].settemp_MIN_Real;}
				                    MR44V064B_Data.MR44V064B_Buff_WR[28]=PID_Data.temp[0].settemp_MAX;
				                    MR44V064B_Data.MR44V064B_Buff_WR[29]=PID_Data.temp[0].settemp_MIN;
                            MR44V064B_Write(&MR44V064B_Data,28,2);					          
					                  break;
				               case 0xD6:
						                PID_Data.temp[1].settemp_MAX=Protocol_Data.Receive_BUFF[3];
                            PID_Data.temp[1].settemp_MIN=Protocol_Data.Receive_BUFF[4];
											 			if(PID_Data.temp[1].settemp_MAX>=PID_Data.temp[1].settemp_MAX_Real)
												         {PID_Data.temp[1].settemp_MAX=PID_Data.temp[1].settemp_MAX_Real;}
												    if(PID_Data.temp[1].settemp_MIN<=PID_Data.temp[1].settemp_MIN_Real)
												         {PID_Data.temp[1].settemp_MIN=PID_Data.temp[1].settemp_MIN_Real;}
				                    MR44V064B_Data.MR44V064B_Buff_WR[30]=PID_Data.temp[1].settemp_MAX;
				                    MR44V064B_Data.MR44V064B_Buff_WR[31]=PID_Data.temp[1].settemp_MIN;
                            MR44V064B_Write(&MR44V064B_Data,30,2);			
					                  break;
			                 case 0xD7:
						                PID_Data.temp[2].settemp_MAX=Protocol_Data.Receive_BUFF[3];
			                      PID_Data.temp[2].settemp_MIN=Protocol_Data.Receive_BUFF[4];
											 			if(PID_Data.temp[2].settemp_MAX>=PID_Data.temp[2].settemp_MAX_Real)
												         {PID_Data.temp[2].settemp_MAX=PID_Data.temp[2].settemp_MAX_Real;}
												    if(PID_Data.temp[2].settemp_MIN<=PID_Data.temp[2].settemp_MIN_Real)
												         {PID_Data.temp[2].settemp_MIN=PID_Data.temp[2].settemp_MIN_Real;}
								            MR44V064B_Data.MR44V064B_Buff_WR[32]=PID_Data.temp[2].settemp_MAX;
				                    MR44V064B_Data.MR44V064B_Buff_WR[33]=PID_Data.temp[2].settemp_MIN;	
                            MR44V064B_Write(&MR44V064B_Data,32,2);		
 					                  break;
				               case 0xD8:					      
						                PID_Data.temp[3].settemp_MAX=Protocol_Data.Receive_BUFF[3];
                            PID_Data.temp[3].settemp_MIN=Protocol_Data.Receive_BUFF[4];
											 			if(PID_Data.temp[3].settemp_MAX>=PID_Data.temp[3].settemp_MAX_Real)
												         {PID_Data.temp[3].settemp_MAX=PID_Data.temp[3].settemp_MAX_Real;}
												    if(PID_Data.temp[3].settemp_MIN<=PID_Data.temp[3].settemp_MIN_Real)
												         {PID_Data.temp[3].settemp_MIN=PID_Data.temp[3].settemp_MIN_Real;}
				                    MR44V064B_Data.MR44V064B_Buff_WR[34]=PID_Data.temp[3].settemp_MAX;
				                    MR44V064B_Data.MR44V064B_Buff_WR[35]=PID_Data.temp[3].settemp_MIN;
	                          MR44V064B_Write(&MR44V064B_Data,34,2);			
					                  break;				
				                case 0xE1:
						                PID_Data.temp[0].kp=Protocol_Data.Receive_BUFF[3];
				                    PID_Data.temp[0].ki=Protocol_Data.Receive_BUFF[4];
				                    PID_Data.temp[0].kd=Protocol_Data.Receive_BUFF[5];
												    ADAU1701_Res_Set(PID_Data.temp[0].kp/10.0,ADAU1701_Channel1,0x0811);
												    ADAU1701_Res_Set(PID_Data.temp[0].ki/50.0,ADAU1701_Channel1,0x0812);
												    ADAU1701_Res_Set(PID_Data.temp[0].kd/100.0,ADAU1701_Channel1,0x0813);
				                    MR44V064B_Data.MR44V064B_Buff_WR[3*0]=PID_Data.temp[0].kp;
				                    MR44V064B_Data.MR44V064B_Buff_WR[3*0+1]=PID_Data.temp[0].ki;
				                    MR44V064B_Data.MR44V064B_Buff_WR[3*0+2]=PID_Data.temp[0].kd; 
                            MR44V064B_Write(&MR44V064B_Data,3*0,3);		
					                  break;
				                case 0xE2:
								            PID_Data.temp[1].kp=Protocol_Data.Receive_BUFF[3];
				                    PID_Data.temp[1].ki=Protocol_Data.Receive_BUFF[4];
								            PID_Data.temp[1].kd=Protocol_Data.Receive_BUFF[5]; 
												    ADAU1701_Res_Set(PID_Data.temp[1].kp/10.0,ADAU1701_Channel2,0x0811);
												    ADAU1701_Res_Set(PID_Data.temp[1].ki/50.0,ADAU1701_Channel2,0x0812);
												    ADAU1701_Res_Set(PID_Data.temp[1].kd/100.0,ADAU1701_Channel2,0x0813);												
							              MR44V064B_Data.MR44V064B_Buff_WR[3*1]=PID_Data.temp[1].kp;
				                    MR44V064B_Data.MR44V064B_Buff_WR[3*1+1]=PID_Data.temp[1].ki;
				                    MR44V064B_Data.MR44V064B_Buff_WR[3*1+2]=PID_Data.temp[1].kd; 
                            MR44V064B_Write(&MR44V064B_Data,3*1,3);
					                  break;
				                 case 0xE3:
								            PID_Data.temp[2].kp=Protocol_Data.Receive_BUFF[3];
				                    PID_Data.temp[2].ki=Protocol_Data.Receive_BUFF[4];
								            PID_Data.temp[2].kd=Protocol_Data.Receive_BUFF[5];
												 		ADAU1701_Res_Set(PID_Data.temp[2].kp/10.0,ADAU1701_Channel3,0x0811);
												    ADAU1701_Res_Set(PID_Data.temp[2].ki/50.0,ADAU1701_Channel3,0x0812);
												    ADAU1701_Res_Set(PID_Data.temp[2].kd/100.0,ADAU1701_Channel3,0x0813);
				                    MR44V064B_Data.MR44V064B_Buff_WR[3*2]=PID_Data.temp[2].kp;
				                    MR44V064B_Data.MR44V064B_Buff_WR[3*2+1]=PID_Data.temp[2].ki;
				                    MR44V064B_Data.MR44V064B_Buff_WR[3*2+2]=PID_Data.temp[2].kd; 
                            MR44V064B_Write(&MR44V064B_Data,3*2,3);
                            break;				
                         case 0xE4:
								            PID_Data.temp[3].kp=Protocol_Data.Receive_BUFF[3];
				                    PID_Data.temp[3].ki=Protocol_Data.Receive_BUFF[4];
								            PID_Data.temp[3].kd=Protocol_Data.Receive_BUFF[5];	
												 		ADAU1701_Res_Set(PID_Data.temp[3].kp/10.0,ADAU1701_Channel4,0x0811);
												    ADAU1701_Res_Set(PID_Data.temp[3].ki/50.0,ADAU1701_Channel4,0x0812);
												    ADAU1701_Res_Set(PID_Data.temp[3].kd/100.0,ADAU1701_Channel4,0x0813);
				                    MR44V064B_Data.MR44V064B_Buff_WR[3*3]=PID_Data.temp[3].kp;
				                    MR44V064B_Data.MR44V064B_Buff_WR[3*3+1]=PID_Data.temp[3].ki;
				                    MR44V064B_Data.MR44V064B_Buff_WR[3*3+2]=PID_Data.temp[3].kd; 
                            MR44V064B_Write(&MR44V064B_Data,3*3,3);		
					                  break;
				                 default:
					                 break;
					
				                  }
		         }
			 for(int i=0;i<7;i++)
		   {  
				  Protocol_Data.Receive_BUFF[i]=0;
			 }
  
      HAL_UART_Receive_IT(&huart3,&Protocol_Data.Receive_BUFF[0],7);	
      Protocol_Data.flag_start=0;				 
		  }
	  }	
	
	
	}
	
	
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
