/**********************************************************************
版权所有：源动力科技
淘    宝：https://1024tech.taobao.com/
文 件 名：key.c
版    本：V21.01.30
摘    要：
***********************************************************************/
#include "key.h"
#include "systick.h"
#include "imath.h"

_KEY key = {0};

//初始化IO
void key_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
        
    RCC_APB2PeriphClockCmd(key_right_rcc, ENABLE);

    GPIO_InitStructure.GPIO_Pin = key_right_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU  ;          
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(key_right_port, &GPIO_InitStructure); 
}

//按键扫描
//0：不支持连续按
//1：支持连续按
uint8_t key_scan(uint8_t mode)
{
    static uint8_t key_up = 1;
    
    if(mode)  
        key_up = 1;
    if(key_up&&(key_right==0))
    {
        tdelay_ms(5);
        key_up = 0;
    
        if(key_right==0)    return 1;
    }
    else if(key_right==1)
    {
        key_up = 1;
    }
    return 0;
}


//按键处理任务
void key_info(void)
{
    key.value = key_scan(0);                //获取键值
    if(key.value==1)    key.flag = 1; 
}



