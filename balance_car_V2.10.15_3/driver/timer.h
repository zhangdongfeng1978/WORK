#ifndef _timer_h_
#define _timer_h_

#include "stm32f10x.h"
 
void timer_init(void);

extern int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
extern int Moto1,Moto2;
extern uint32_t running_tim_cnt;


#endif

