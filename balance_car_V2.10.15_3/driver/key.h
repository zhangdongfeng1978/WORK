#ifndef _key_h_
#define _key_h_


#include "stm32f10x.h"


typedef struct
{
    int value;
    int r;
    uint8_t flag;
}_KEY;


#define key_right_rcc    RCC_APB2Periph_GPIOB  
#define key_right_port   GPIOB
#define key_right_pin    GPIO_Pin_5


#define key_right  GPIO_ReadInputDataBit(key_right_port,key_right_pin)

extern _KEY key;

void key_init(void);
uint8_t key_scan(uint8_t mode);
void key_info(void);


#endif


