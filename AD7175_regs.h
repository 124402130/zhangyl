/**************************************************************************//**
*   @file   AD7175_regs.h
*   @brief  AD7175 Registers Definitions.
*   @author acozma (andrei.cozma@analog.com)
*
*******************************************************************************
* Copyright 2011(c) Analog Devices, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*  - Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  - Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
*  - Neither the name of Analog Devices, Inc. nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*  - The use of this software may or may not infringe the patent rights
*    of one or more patent holders.  This license does not release you
*    from the requirement that you obtain separate licenses from these
*    patent holders to use this software.
*  - Use of the software either in source or binary form, must be run
*    on or directly connected to an Analog Devices Inc. component.
*
* THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, MERCHANTABILITY
* AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*******************************************************************************
*   SVN Revision: 0
******************************************************************************/
#ifndef __AD7175_REGS_H__
#define __AD7175_REGS_H__
/*! AD7175 register info */
typedef struct _st_reg
{
    unsigned long addr;
    unsigned long value;
    unsigned long size;
} st_reg;
/*! AD7175 registers list*/
enum AD7175_registers
{
    Status_Register = 0x00, 
    ADC_Mode_Register,
    Interface_Mode_Register,
    REGCHECK,
    Data_Register,
    GPIOCON,
    ID_st_reg,
    CH_Map_0,
    CH_Map_1,
    CH_Map_2,
	  CH_Map_3,
	  CH_Map_4,
	  CH_Map_5,
	  CH_Map_6,
	  CH_Map_7,
	  CH_Map_8,
	  CH_Map_9,
	  CH_Map_10,
	  CH_Map_11,
	  CH_Map_12,
	  CH_Map_13,
	  CH_Map_14,
	  CH_Map_15,
    Setup_Config_0,
    Setup_Config_1,
    Setup_Config_2,
    Setup_Config_3,
		Setup_Config_4,
		Setup_Config_5,
		Setup_Config_6,
		Setup_Config_7,
    Filter_Config_0,
    Filter_Config_1,
    Filter_Config_2,
    Filter_Config_3,
		Filter_Config_4,
		Filter_Config_5,
		Filter_Config_6,
		Filter_Config_7,
    Offset_0,
    Offset_1,
    Offset_2,
    Offset_3,
		Offset_4,
    Offset_5,
    Offset_6,
    Offset_7,
    Gain_0,
    Gain_1,
    Gain_2,
    Gain_3,
		Gain_4,
    Gain_5,
    Gain_6,
    Gain_7,
    Communications_Register,
    AD7175_REG_NO
};
#ifdef AD7175_INIT
/*! Array holding the info for the AD7175 registers - address, initial value, size */
//============================================================================
//ADC_Mode_Register  bit 15     使能内部基准电压源2.5V。                 0 Disable 1 Enable
//                   bit 14     是否隐藏延时时间，选sin5c+sin1c滤波器。  0 Enable  1 Disable
//                   bit 13     单个通道用一个滤波器的时候用             0 Disable 1 Enable
//                   bit 12-11   RESERVED                               00 
//                   bit 10-8    通道切换时为了电路稳定下来            000 0us
//                                                                     001 4us
//                                                                     010 16us
//                                                                     011 40us
//                                                                     100 100us
//                                                                     101 200us
//                                                                     110 500us
//                                                                     111 1ms
//                   bit 7      RESERVED                                 0
//                   bit 6-4    工作模式                               000 连续转换
//                                                                     001 单次转换
//                                                                     010 待机模式
//                                                                     011 掉电模式
//                                                                     100 内部失调校准
//                                                                     110 系统失调校准
//                                                                     111 系统增益校准
//                   bit 3-2     时钟源                                 00 内部时钟输入
//                                                                      01 内部时钟输出
//                                                                      10 外部时钟输入有源
//                                                                      11 外部时钟输入无源
//                   bit 1-0     RESERVED                               00
//============================================================================
//Interface_Mode_Register  bit 15~13    RESERVED                       000
//                         bit 12       交替帧同步                       0 Disable 1 Enable
//                         bit 11       DOUT/RDY驱动强度                 0 Disable 1 Enable
//                         bit 10~9     RESERVED                        00 
//                         bit 8        DOUT/RDY延长RDY时间              0 Disable 1 Enable
//                         bit 7        连续读模式                       0 Disable 1 Enable
//                         bit 6        输出结果是否附加状态寄存器       0 Disable 1 Enable
//                         bit 5        是否启用检查寄存器               0 Disable 1 Enable
//                         bit 4        RESERVED                         0
//                         bit 3~2      是否启用CRC校验                 00 Disable
//                                                                      01 读XOR，写CRC
//                                                                      10 读CRC，写CRC
//                         bit 1        RESERVED                         0  
//                         bit 0        ADC数据寄存器位数                0 24bit   1 16bit
//============================================================================
//GPIOCON_Register         bit 15       RESERVED                         0
//                         bit 14~12                                   000
//                         bit 11       帧同步使能                       0 Disable 1 Enable
//                         bit 10~9     寄存器错误使能                  00 Disable
//                                                                      01 错误输入
//                                                                      10 错误开漏输出
//                                                                      11 错误通用输出
//                         bit 8        寄存器错误逻辑电平               0
//                         bit 7        写GPO3                           0
//                         bit 6        写GPO2                           0
//                         bit 5        GPIO1输入                        0 Disable 1 Enable
//                         bit 4        GPIO0输入                        0 Disable 1 Enable
//                         bit 3        GPIO1输出                        0 Disable 1 Enable
//                         bit 2        GPIO0输出                        0 Disable 1 Enable
//                         bit 1        GPIO1读写的数据                  0 
//                         bit 0        GPIO0读写的数据                  0 
//============================================================================
//CHx_Register             bit 15       通道使能                         0 Disable 1 Enable
//                         bit 14~12    安装到那种滤波增益配置         000 setup0
//                                                                     001 setup1
//                                                                     010 setup2
//                                                                     011 setup3
//                                                                     100 setup4
//                                                                     101 setup5
//                                                                     110 setup6
//                                                                     111 setup7
//                         bit 11~10    RESERVED                        00
//                         bit 9~5      ADC 正端输入选择             00000 AIN0
//                                                                   00001 AIN1
//                                                                   00010 AIN2
//                                                                   00011 AIN3
//                                                                   00100 AIN4
//                                                                   00101 AIN5
//                                                                   00110 AIN6
//                                                                   00111 AIN7
//                                                                   01000 AIN8
//                                                                   01001 AIN9
//                                                                   01010 AIN10
//                                                                   01011 AIN11
//                                                                   01100 AIN12
//                                                                   01101 AIN13
//                                                                   01110 AIN14
//                                                                   01111 AIN15
//                                                                   10000 AIN16
//                                                                   10001 Temp Sensor+
//                                                                   10010 Temp Sensor-
//                                                                   10011 ((AVDD1 - AVSS)/5)+    (analog input buffers must be enabled) 
//                                                                   10100 ((AVDD1 - AVSS)/5)-    (analog input buffers must be enabled)
//                                                                   10101 REF+
//                                                                   10110 REF-
//                         bit 4~0      ADC负端输入选择              00000 AIN0
//                                                                   00001 AIN1
//                                                                   00010 AIN2
//                                                                   00011 AIN3
//                                                                   00100 AIN4
//                                                                   00101 AIN5
//                                                                   00110 AIN6
//                                                                   00111 AIN7
//                                                                   01000 AIN8
//                                                                   01001 AIN9
//                                                                   01010 AIN10
//                                                                   01011 AIN11
//                                                                   01100 AIN12
//                                                                   01101 AIN13
//                                                                   01110 AIN14
//                                                                   01111 AIN15
//                                                                   10000 AIN16
//                                                                   10001 Temp Sensor+
//                                                                   10010 Temp Sensor-
//                                                                   10011 ((AVDD1 - AVSS)/5)+    (analog input buffers must be enabled) 
//                                                                   10100 ((AVDD1 - AVSS)/5)-    (analog input buffers must be enabled)
//                                                                   10101 REF+
//                                                                   10110 REF-
//                                                                   8020      AIN1_AIN0
//                                                                   9040      AIN2_AIN0
//                                                                   A060      AIN3_AIN0
//                                                                   B080      AIN4_AIN0
//                                                                   C0A0      AIN5_AIN0
//                                                                   D0C0      AIN6_AIN0
//                                                                   E0E0      AIN7_AIN0
//                                                                   F100      AIN8_AIN0
//============================================================================
//SetupConx_Register             bit 15~13       RESERVED              000 
//                               bit 12          双极性还是单极性        0 单极性 1 双极性
//                               bit 11          REF+ BUFF使能           0 Disable 1 Enable
//                               bit 10          REF- BUFF使能           0 Disable 1 Enable
//                               bit 9           AIN+ BUFF使能           0 Disable 1 Enable
//                               bit 8           AIN- BUFF使能           0 Disable 1 Enable
//                               bit 7           诊断开路                0
//                               bit 6           RESERVED                0 
//                               bit 5~4         基准选择               00 外部REF 
//                                                                      01 外部REF2 
//                                                                      10 内部2.5V
//                                                                      11 AVDD1 - AVSS 可用于诊断，验证基准值
//                               bit 3~0         RESERVED             0000
//                                                                    0500 单极性有地的输入buff-的缓冲器，正端不用缓冲器
//                                                                    0700 单极性板上温度传感器跟随进来用缓冲器
//============================================================================
//FILTCONx_Register             bit 15           SIN3C滤波器使能         0 Disable 1 Enable
//                              bit 14~12        RESERVED              000
//                              bit 11           50/60HZ抑制             0 Disable 1 Enable
//                              bit 10~8         50/60HZ输出速率       010 27HZ、47dB、36.7ms
//                                                                     011 25HZ、62dB、40ms
//                                                                     101 20HZ、86dB、50ms
//                                                                     110 16.67HZ、92dB、60ms
//                              bit 7            RESERVED                0
//                              bit 6~5          滤波器阶数             00 sinc5+sinc1
//                                                                      11 sinc3
//                              bit 4~0          sinc5 + sinc 1速率  00000 250000
//                                                                   00001 125000
//                                                                   00010  62500
//                                                                   00011  50000
//                                                                   00100  31250
//                                                                   00101  25000
//                                                                   00110  15625
//                                                                   00111  10000
//                                                                   01000   5000
//                                                                   01001   2500
//                                                                   01010   1000
//                                                                   01011    500
//                                                                   01100    397.5
//                                                                   01101    200
//                                                                   01110    100
//                                                                   01111     59.92
//                                                                   10000     49.96
//                                                                   10001     20
//                                                                   10010     16.67
//                                                                   10011     10
//                                                                   10100      5
//选择 0D10 sinc5+sinc1 输出49.96Hz 增强50/60Hz输出20Hz


st_reg AD7175_regs[] =
{
    {0x00, 0x80,   1},  //Status_Register
    {0x01, 0x2310, 2},  //ADC_Mode_Register
    {0x02, 0x0040, 2},  //Interface_Mode_Register
    {0X03, 0x000000, 3},  //REGCHECK
    {0x04, 0x00000000, 4},  //Data_Register
    {0x06, 0x0000, 2},  //GPIOCON
    {0x07, 0x0000, 2},  //ID_st_reg
    {0x10, 0x8020, 2},  //CH_Map_0
    {0x11, 0x9040, 2},  //CH_Map_1
    {0x12, 0xA060, 2},  //CH_Map_2
    {0x13, 0xB080, 2},  //CH_Map_3
		{0x14, 0xC0A0, 2},  //CH_Map_4
		{0x15, 0xD0C0, 2},  //CH_Map_5
		{0x16, 0xE0E0, 2},  //CH_Map_6
		{0x17, 0xF100, 2},  //CH_Map_7
		{0x18, 0x0001, 2},  //CH_Map_8
		{0x19, 0x0001, 2},  //CH_Map_9
		{0x1A, 0x0001, 2},  //CH_Map_10
		{0x1B, 0x0001, 2},  //CH_Map_11
		{0x1C, 0x0001, 2},  //CH_Map_12
		{0x1D, 0x0001, 2},  //CH_Map_13
		{0x1E, 0x0001, 2},  //CH_Map_14
		{0x1F, 0x0001, 2},  //CH_Map_15
    {0x20, 0x0700, 2},  //Setup_Config_0
    {0x21, 0x0700, 2},  //Setup_Config_1
    {0x22, 0x0700, 2},  //Setup_Config_2
    {0x23, 0x0700, 2},  //Setup_Config_3
		{0x24, 0x0700, 2},  //Setup_Config_4
		{0x25, 0x0700, 2},  //Setup_Config_5
		{0x26, 0x0700, 2},  //Setup_Config_6
		{0x27, 0x0700, 2},  //Setup_Config_7
    {0x28, 0x000A, 2},  //Filter_Config_0
    {0x29, 0x000A, 2},  //Filter_Config_1
    {0x2A, 0x000A, 2},  //Filter_Config_2
    {0x2B, 0x000A, 2},  //Filter_Config_3
		{0x2C, 0x000A, 2},  //Filter_Config_4
		{0x2D, 0x000A, 2},  //Filter_Config_5
		{0x2E, 0x000A, 2},  //Filter_Config_6
		{0x2F, 0x000A, 2},  //Filter_Config_7
    {0x30, 0x800000, 3},    //Offset_0
    {0x31, 0x800000, 3},    //Offset_1
    {0x32, 0x800000, 3},    //Offset_2
    {0x33, 0x800000, 3},    //Offset_3
		{0x34, 0x800000, 3},    //Offset_4
    {0x35, 0x800000, 3},    //Offset_5
    {0x36, 0x800000, 3},    //Offset_6
    {0x37, 0x800000, 3},    //Offset_7
    {0x38, 0x500000, 3},    //Gain_0
    {0x39, 0x500000, 3},    //Gain_1
    {0x3A, 0x500000, 3},    //Gain_2
    {0x3B, 0x500000, 3},    //Gain_3
		{0x3C, 0x500000, 3},    //Gain_4
    {0x3D, 0x500000, 3},    //Gain_5
    {0x3E, 0x500000, 3},    //Gain_6
    {0x3F, 0x500000, 3},    //Gain_7
    {0xFF, 0x00, 1}     //Communications_Register
};
#else
extern st_reg AD7175_regs[AD7175_REG_NO];
#endif
#define AD7175_SLAVE_ID    1
/* Communication Register bits */
#define COMM_REG_WEN    (0 << 7)
#define COMM_REG_WR     (0 << 6)
#define COMM_REG_RD     (1 << 6)
/* Status Register bits */
#define STATUS_REG_RDY      (1 << 7)
#define STATUS_REG_ADC_ERR  (1 << 6)
#define STATUS_REG_CRC_ERR  (1 << 5)
#define STATUS_REG_REG_ERR  (1 << 4)
#define STATUS_REG_CH(x)    ((x) & 0x3f)
/* ADC Mode Register */
#define ADC_MODE_REG_REF_EN         (1 << 15)
#define ADC_MODE_REG_DELAY(x)       (((x) & 0x7) << 8)
#define ADC_MODE_REG_MODE(x)        (((x) & 0x7) << 4)
#define ADC_MODE_REG_CLKSEL(x))     (((x) & 0x3) << 2)
/* Interface Mode Register bits */
#define INTF_MODE_REG_DOUT_RESET    (1 << 8)
#define INTF_MODE_REG_CONT_READ     (1 << 7)
#define INTF_MODE_REG_DATA_STAT     (1 << 6)
#define INTF_MODE_REG_CRC_EN        (0x02 << 2)
#define INTF_MODE_REG_CRC_STAT(x)   (((x) & INTF_MODE_REG_CRC_EN) == INTF_MODE_REG_CRC_EN)
/* GPIO Configuration Register */
#define GPIO_CONF_REG_MUX_IO        (1 << 12)
#define GPIO_CONF_REG_SYNC_EN       (1 << 11)
#define GPIO_CONF_REG_ERR_EN(x)     (((x) & 0x3) << 9)
#define GPIO_CONF_REG_ERR_DAT       (1 << 8)
#define GPIO_CONF_REG_GP_DATA3      (1 << 7)
#define GPIO_CONF_REG_GP_DATA2      (1 << 6)
#define GPIO_CONF_REG_IP_EN1        (1 << 5)
#define GPIO_CONF_REG_IP_EN0        (1 << 4)
#define GPIO_CONF_REG_OP_EN1        (1 << 3)
#define GPIO_CONF_REG_OP_EN0        (1 << 2)
#define GPIO_CONF_REG_DATA1         (1 << 1)
#define GPIO_CONF_REG_DATA0         (1 << 0)            //没懂
/* ID Register */
#define ID_REG_PRODUCT_ID(x)        (((x) & 0xFF) << 8) //没懂
/* Channel Map Register 1-4 */
#define CH_MAP_REG_CHEN         (1 << 15)
#define CH_MAP_REG_SETUP(x)     (((x) & 0x7) << 12)
#define CH_MAP_REG_AINPOS(x)    (((x) & 0x1F) << 5)
#define CH_MAP_REG_AINNEG(x)    (((x) & 0x1F) << 0)
/* Setup Configuration Register 1-8 */
#define SETUP_CONF_REG_CHOP_MD(x)       (((x) & 0x7) << 13)
#define SETUP_CONF_REG_BI_UNIPOLAR      (1 << 12)
#define SETUP_CONF_REG_REF_BUF_P        (1 << 11)
#define SETUP_CONF_REG_REF_BUF_N        (1 << 10)
#define SETUP_CONF_REG_AIN_BUF_P        (1 << 9)
#define SETUP_CONF_REG_AIN_BUF_N        (1 << 8)
#define SETUP_CONF_REG_BRNOUT_EN        (1 << 7)
#define SETUP_CONF_REG_REF_SEL(x)       (((x) & 0x3) << 4)
/* Filter Configuration Register 1-8 */
#define FILT_CONF_REG_EXTCLKFREQ(x)     (((x) & 0x7) << 12) //没懂要录什么
#define FILT_CONF_REG_ENHFILTEN         (1 << 11)
#define FILT_CONF_REG_ENHFILTSEL(x)     (((x) & 0x7) << 8)
#define FILT_CONF_REG_ORDER(x)          (((x) & 0x7) << 5)
#define FILT_CONF_REG_ODR(x)            (((x) & 0x1F) << 0)
#endif //__AD7175_REGS_H__
