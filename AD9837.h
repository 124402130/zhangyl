#ifndef __AD9837_H__
#define __AD9837_H__
/***************************** Include Files *********************************/
#include <stdint.h>
/******************* AD9837 Constants ****************************************/
#define DDS_SPI3_Size 1
#define waittime      1
/************************ Functions Declarations *****************************/
void DDS_10K_init(void);
//void DDS_13_5K_init(void);
void DDS_Frequent_set(uint32_t Frequent);
//void DDS2_Frequent_set(uint32_t Frequent);

#endif // __AD9837_H__
