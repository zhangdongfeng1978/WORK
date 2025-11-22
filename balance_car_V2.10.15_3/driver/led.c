/**********************************************************************
版权所有：源动力科技
淘    宝：https://1024tech.taobao.com/
文 件 名：led.c
版    本：V21.01.30
摘    要：
***********************************************************************/
#include "led.h"
#include "imu.h"
#include "mpu6050.h"
#include "imath.h"

/*
 * 函数名：led_on_all
 * 描述  ：打开LED灯显示
 * 输入  ： 
 * 返回  ： 
 */
void led_on_all(void)
{
    led_on(1);
    led_on(2);  
}
/*
 * 函数名：led_off_all
 * 描述  ：关闭LED灯
 * 输入  ： 
 * 返回  ： 
 */
void led_off_all(void)
{
    led_off(1);
    led_off(2);    
}

/*
 * 函数名：led_init
 * 描述  ：LED指示灯初始化
 * 输入  ： 
 * 返回  ： 
 */
void led_init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(led1_rcc | led2_rcc , ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = led1_pin;
    GPIO_Init(led1_port, &GPIO_InitStructure);  

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = led2_pin;
    GPIO_Init(led2_port, &GPIO_InitStructure);  
    
    led_on_all();
}

/*
 * 函数名：led_on
 * 描述  ：点亮LED
 * 输入  ：led_num指示灯号数
 * 返回  ： 
 */
void led_on(uint8_t led_num)
{
    if(led_num==1)
        led1_port->BRR  = led1_pin;
    if(led_num==2)
        led2_port->BRR  = led2_pin;
}

/*
 * 函数名：led_off
 * 描述  ：关闭LED
 * 输入  ：led_num指示灯号数
 * 返回  ： 
 */
void led_off(uint8_t led_num)
{
    if(led_num==1)
        led1_port->BSRR  = led1_pin;
    if(led_num==2)
        led2_port->BSRR  = led2_pin;
}

/*
 * 函数名：led_toggle
 * 描述  ：翻转LED状态
 * 输入  ：led_num指示灯号数
 * 返回  ： 
 */
void led_toggle(uint8_t led_num)
{
    if(led_num==1)
        led1_port->ODR ^= led1_pin;    
    if(led_num==2)
        led2_port->ODR ^= led2_pin;
}

_LED_STATES _led ={0};


//无任何操作时状态灯
void _no_operation(void)
{
    static uint16_t l_cnt = 0;
    
    if(++l_cnt>100)
    {
        l_cnt = 0;
        led_toggle(1);         
    }
}
//校准陀螺仪时的指示灯状态
void _cal_operation(void)
{
    static uint16_t l_cnt = 0;
    
    if(++l_cnt>10)
    {
        l_cnt = 0;
        led_toggle(1);         
    }
}
//_CAR_STATES _car = {0};


void led_blink(void )
{
    switch(_led.sta)
    {
        case 1:
            _no_operation(); break;
        case 2:
            _cal_operation();break;
        default:break;
    }

}

