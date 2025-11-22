#ifndef _flash_h_
#define _flash_h_


#include "stm32f10x.h"

typedef struct
{
    uint16_t sensor;
    uint8_t pid;
//    uint16_t acc_offset;
//    uint8_t acc_scale;
//    uint8_t mag_offset;
//	uint8_t gyro_offset;
}_FLASH_flag;


#define START_ADDRESS  0x08000000 + 1024*60   
#define SENSOR_CAL_ADDRESS      36
#define PID_Group1_ADDRESS    SENSOR_CAL_ADDRESS + 36
#define PID_Group2_ADDRESS    PID_Group1_ADDRESS + 36
#define PID_Group3_ADDRESS    PID_Group2_ADDRESS + 36
#define PID_FLAG_ADDRESS      PID_Group3_ADDRESS + 4  


void FlashWriteNineFloat(uint32_t WriteAddress, float WriteData1,float WriteData2,float WriteData3,
                                                float WriteData4,float WriteData5,float WriteData6,
                                                float WriteData7,float WriteData8,float WriteData9);
                                            
uint16_t FlashReadNineFloat(uint32_t ReadAddress,float *ReadData1,float *ReadData2,float *ReadData3,
                                                 float *ReadData4,float *ReadData5,float *ReadData6,
                                                 float *ReadData7,float *ReadData8,float *ReadData9);

void FlashWriteWord(uint32_t WriteAddress,uint32_t WriteData);
int FlashReadBytes(uint32_t ReadAddress, uint8_t *ReadBuf, int32_t ReadLen);      
void FlashWriteByte(uint32_t WriteAddress,uint8_t WriteData);
                                            
extern _FLASH_flag flash_flag;


#endif


