#ifndef _systick_h_
#define _systick_h_

#include "stm32f10x.h"

void systick_init(void);

void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);
void softTimesCountdown(void);

extern uint8_t getstatus;
extern uint32_t softTimes[5];
#endif

