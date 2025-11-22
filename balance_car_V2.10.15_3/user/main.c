/**********************************************************************
版权所有：源动力科技
淘    宝：https://1024tech.taobao.com/
文 件 名：mian.c
版    本：V21.01.30
摘    要：
***********************************************************************/
#include "stm32f10x.h"
#include "systick.h"
#include "led.h"
#include "iic.h"
#include "mpu6050.h"
#include "timer.h"
#include "nvic.h"
#include "imu.h"
#include "pwm.h"
#include "flash.h"
#include "pid.h"
#include "usart1.h"
#include "bluetooth.h"
#include "key.h"
#include "ps2.h"
#include "oled.h"
#include "adc.h"
#include "ultrasonic.h"
#include "imath.h"
#include "Infrared.h"

/* 
*	使能SWD， 失能JTAG
*	PB3,PB4,PA15作为普通IO使用 
*   MCU复位后，PA13/14/15 & PB3/4默认配置为JTAG功能
*/
static void _SWJ_Config(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);	
}

int main(void)
{    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);         //优先级组别2
    _SWJ_Config();                                          //使能SWD，失能JTAG  
    usart1_init(115200);                                    //串口1配置初始化                              
    printf("usart1 is ok\r\n");
    BluetoothUsart_init(9600);                              //蓝牙串口初始化
    pwm_init();                                             //pwm初始
    driver_pin_init();                                      //电机驱动配置初始化
    Encoder_A_init();                                       //编码器A初始化     
    Encoder_B_init();                                       //编码器B初始化  
    tdelay_ms(100);                                      
    IIC_Init();                                             //IIC端口配置
    mpu6050_init();                                         //mpu6050初始化             
    while(1)                                                //mpu6050在位检测    
    {
        uint8_t mpuId;
        mpuId = get_mpu_id();                               //读取mpu6050的id
        if(mpuId==0x68||mpuId==0x98)                        //判断mpu6050的ID是否正确
            break;               
        tdelay_ms(50);                   
    }
    get_iir_factor(&Mpu.att_acc_factor,0.005f,25);   	    //姿态解算时加速度低通系数 
    OLED_Init();                                            //OLED显示屏端口初始化
    adc_init();                                             //ADC配置初始化
    ReadCalData();                                          //读取校准后的陀螺仪零偏数据
    tdelay_ms(100);                                      
    timer_init();                                           //定0时器初始化   
    HCSR04_Init();                                          //超声波端口初始化
    systick_init();                                         //滴答定时器初始化   
    NVIC_config();                                          //中断配置初始化

    while(1)
    {
        if(softTimes[0] == 0)                               //100ms
        {   
            softTimes[0] = 20;  
        }
        if(softTimes[1] == 0)                               //20ms
        {
            softTimes[1] = 4;  
            UltrasonicWave_StartMeasure();                  //超声波发出起始高电平   
            ParseBluetoothMessage(USARTxBlueTooth_RX_DATA, &BluetoothParseMsg);     //蓝牙数据包解析   
        }
        if(softTimes[2] == 0)                               //100ms
        {
            softTimes[2] = 20;  
            voltage_detection();                            //低电压检测
            OledShowDatas();            
        } 
    }
}










/*
Code:    存在ROM(FLASH)里面的指令，这是在程序运行过程中不变的量，是指令
RO_data：只读数据(Read only data)，是指令的操作数
RW_data: 已经初始化的可读写变量的大小，这个存在两个地方，初始化量存在ROM(FLASH)，
         由于还要对其进行"写"操作，所以RAM中也要占有相应空间
ZI_data: 程序中未初始化的变量大小(Zero Initialize)，存在RAM中

    ROM(FLASH) SIZE:  Code + RO-data + RW-data 
    FLASH SIZE = 25230 + 1130 + 208 = 26.568KB
    
    FLASH起始地址：0x08000000，所以在设置FLASH进行保存的时候，要大于本代码所占用的扇区地址
    代码扇区大小计算：0x08000000~0x08003fff  ：16*16*16*4=16KB
    所以在设置FLASH地址的时候要在代码flash之外
*/





