/**********************************************************************
版权所有：源动力科技
淘    宝：https://1024tech.taobao.com/
文 件 名：Infrared.c
版    本：V21.01.30
摘    要：
***********************************************************************/

/* Includes ------------------------------------------------------------------*/
/* Hardware and starter kit includes. */
/* Kernel includes. */

#include "systick.h"
/* application includes */
#include "Infrared.h"
#include "ultrasonic.h"
#include "timer.h"
#include "usart1.h"
#include "bluetooth.h"
#include "imath.h"

/* Private define ------------------------------------------------------------*/
#define Infrared1_PORT      GPIOB
#define Infrared2_PORT      GPIOB
#define Infrared3_PORT      GPIOB
#define Infrared4_PORT      GPIOA

#define Infrared1_PIN       GPIO_Pin_9
#define Infrared2_PIN       GPIO_Pin_8
#define Infrared3_PIN       GPIO_Pin_5
#define Infrared4_PIN       GPIO_Pin_15

/* Private macro -------------------------------------------------------------*/
#define Inf1        GPIO_ReadInputDataBit(Infrared1_PORT, Infrared1_PIN)
#define Inf2        GPIO_ReadInputDataBit(Infrared2_PORT, Infrared2_PIN)
#define Inf3        GPIO_ReadInputDataBit(Infrared3_PORT, Infrared3_PIN)
#define Inf4        GPIO_ReadInputDataBit(Infrared4_PORT, Infrared4_PIN)

/* Private type --------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void Infrared_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE); 

    //1/2/3
    GPIO_InitStructure.GPIO_Pin = Infrared1_PIN | Infrared2_PIN | Infrared3_PIN;             
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;           
    GPIO_Init(GPIOB,&GPIO_InitStructure); 
    //4
    GPIO_InitStructure.GPIO_Pin = Infrared4_PIN ;             
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;           
    GPIO_Init(GPIOA,&GPIO_InitStructure);      
}

/* 读取每个红外通道状态数据 */
uint8_t getInfaredStatus(void)
{
    uint8_t getstatus = 0;
    if(!Inf1)
    {
        getstatus |= InfraredChannel1;        
//printf("Inf1 %d  %2X\r\n",getstatus,getstatus);
    }
    else if(!Inf2)
    {
        getstatus |= InfraredChannel2;        
    }
    else if(!Inf3)
    {
        getstatus |= InfraredChannel3;   
    }
    else if(!Inf4)
    {
        getstatus |= InfraredChannel4;     
    }

    return getstatus;
}

/****************** (C) COPYRIGHT Mr.Lin @ 源动力科技 *********END OF FILE****/
