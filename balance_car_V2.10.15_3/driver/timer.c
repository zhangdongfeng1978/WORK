/**********************************************************************
版权所有：源动力科技
淘    宝：https://1024tech.taobao.com/
文 件 名：timer.c
版    本：V21.01.30
摘    要：
***********************************************************************/
#include "timer.h"
#include "led.h"
#include "systick.h"
#include "iic.h"
#include "mpu6050.h"
#include "imu.h"
#include "pwm.h"
#include "controller.h"
#include "usart1.h"
#include "imath.h"
#include "key.h"
#include "ps2.h"
#include "ultrasonic.h"
//定时器初始化 5ms
void timer_init(void)
{ 
	TIM_TimeBaseInitTypeDef TIM_timeBaseStucture;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	
    // 72 * 50000 / 72000000 = 5/100 = 1/20s = 50ms = 50000us  
	TIM_timeBaseStucture.TIM_ClockDivision = TIM_CKD_DIV1;      	            //分频因子，输出给定时器的ETRP数字滤波器提供时钟
	TIM_timeBaseStucture.TIM_Prescaler = 72-1;                		            //预分频，给TIMx_CNT驱动的时钟，注意：实际的预分频值是0+1
	TIM_timeBaseStucture.TIM_Period = 50000-1;	
	TIM_timeBaseStucture.TIM_CounterMode = TIM_CounterMode_Up;		            //向上计数
    TIM_timeBaseStucture.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1,&TIM_timeBaseStucture);
	TIM_Cmd(TIM1,ENABLE);
	TIM_ClearFlag(TIM1,TIM_FLAG_Update);							            //先清除定时器更新标志位，防止一开启中断就进入中断处理函数中
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
}


void TIM1_UP_IRQHandler(void)
{      
   	if(TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
	{        

	}				   
	TIM_ClearITPendingBit(TIM1,TIM_IT_Update);  
}


