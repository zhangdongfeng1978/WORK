/**********************************************************************
版权所有：源动力科技
淘    宝：https://1024tech.taobao.com/
文 件 名：ultrasonic.c
版    本：V21.01.30
摘    要：
***********************************************************************/
/* Includes ------------------------------------------------------------------*/
/* Hardware and starter kit includes. */
/* Kernel includes. */
#include "stm32f10x.h"
#include "systick.h"
/* application includes */
#include "ultrasonic.h"
#include "timer.h"
#include "usart1.h"
#include "bluetooth.h"
#include "imath.h"
/* Private macro -------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//TRIG
#define HCSR04_TRIG_CLK     RCC_APB2Periph_GPIOA
#define HCSR04_TRIG_PORT    GPIOA
#define HCSR04_TRIG_PIN     GPIO_Pin_3
//ECHO
#define HCSR04_ECHO_CLK     RCC_APB2Periph_GPIOC
#define HCSR04_ECHO_PORT    GPIOC
#define HCSR04_ECHO_PIN     GPIO_Pin_13

/* Private type --------------------------------------------------------------*/

/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
int HCSR04_Init(void)
{
    int err;
    NVIC_InitTypeDef NVIC_InitStructure;  
	EXTI_InitTypeDef EXTI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;  
    RCC_APB2PeriphClockCmd(HCSR04_TRIG_CLK,ENABLE);  
    RCC_APB2PeriphClockCmd(HCSR04_ECHO_CLK,ENABLE); 

    // IO初始化  
    GPIO_InitStructure.GPIO_Pin = HCSR04_TRIG_PIN;          //发送电平引脚  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;        //推挽输出  
    GPIO_Init(HCSR04_TRIG_PORT,&GPIO_InitStructure);  

    GPIO_InitStructure.GPIO_Pin = HCSR04_ECHO_PIN;          //返回电平引脚  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;           //输入  
    GPIO_Init(HCSR04_ECHO_PORT,&GPIO_InitStructure);    

	//GPIOC.7	  中断线以及中断初始化配置
 	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource13);

    EXTI_InitStructure.EXTI_Line=EXTI_Line13;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);		                    //根据EXTI_InitStruct中指定的参数初始化外设EXTI寄存器
		
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;		
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2 ;	
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;			
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				
  	NVIC_Init(&NVIC_InitStructure);  	
    return err;
}

float getDistance;
uint16_t EchoFlag = 0;
uint16_t disCounter;
void EXTI15_10_IRQHandler(void)
{
    disCounter = 0;
	tdelay_us(3);		                      //延时10us
    if(EXTI_GetITStatus(EXTI_Line13) != RESET)
	{
        EchoFlag = 1;
        TIM_SetCounter(TIM1,0);
        TIM_Cmd(TIM1, ENABLE);  

        while(GPIO_ReadInputDataBit(HCSR04_ECHO_PORT,HCSR04_ECHO_PIN))              //等待低电平
        {
            disCounter = TIM_GetCounter(TIM1);
            if(disCounter>5000)                                                     //最远测量控制在5000计数值，换为距离为85cm
                break;
        }
        
        TIM_Cmd(TIM1, DISABLE);

        getDistance = TIM_GetCounter(TIM1)/1000000.0f * 340.0f / 2.0f * 100.0f;	    //us->s   声速340m/s  最终转为厘米cm	
        //计算公式为：距离=（高电平时间*340m/s）/2

//        printf("%.3fcm   %d\r\n",getDistance,TIM_GetCounter(TIM1));

        EXTI_ClearITPendingBit(EXTI_Line13);  //清除EXTI7线路挂起位
    }
}

/*
 * 函数名：UltrasonicWave_StartMeasure
 * 描述  ：开始测距，发送一个>10us的脉冲，然后测量返回的高电平时间
 * 输入  ：无
 * 输出  ：无	
 */
void UltrasonicWave_StartMeasure(void)
{
    GPIO_SetBits(HCSR04_TRIG_PORT,HCSR04_TRIG_PIN); 		    //送>10US的高电平
    tdelay_us(20);		                                        //延时20US
    GPIO_ResetBits(HCSR04_TRIG_PORT,HCSR04_TRIG_PIN);
}
//超声波自检是否正常标志，0模块正常，1模块未插上
uint8_t UltraFlag = 0;
uint8_t UltrasonicCheck(void)
{
    static uint16_t UltraTime = 0;
	if(getDistance==0.0f)
    {
        UltraTime++;
		if(UltraTime>200)
        {
            UltraTime = 0;
            UltraFlag = 1;    
//            EXTI->IMR &= ~(EXTI_Line13) ;
        }
	}
    else
    {
        UltraFlag = 0;
//        EXTI->IMR |= ~(EXTI_Line13) ;
    }    

    return 0;
}
/*
 * 函数名：UltraSuccess
 * 描述  ：根据超声波是否在位来判断模式
 * 输入  ：蓝牙接收输入的模式
 * 输出  ：最终模式	
 */
uint8_t UltraSuccess(uint8_t rcMode)
{
    uint8_t UltraMode = 0;
    if( UltraFlag == 0 )
    {
        UltraMode = rcMode ;
    }
    else 
    {
        UltraMode = 0;
    }
	return UltraMode;
}

uint8_t IsUltrasonicOK(void)
{
//    printf("ultra：%d\r\n",UltraFlag);
	return UltraFlag;
}
/****************** (C) COPYRIGHT Mr.Lin @ 源动力科技 *********END OF FILE****/
