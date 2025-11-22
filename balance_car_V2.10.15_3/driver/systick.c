/**********************************************************************
版权所有：源动力科技
淘    宝：https://1024tech.taobao.com/
文 件 名：systick.c
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
#include "Infrared.h"

/* SystemFrequency / 100     10ms中断一次
*  SystemFrequency / 1000    1ms中断一次
*  SystemFrequency / 100000	 10us中断一次
*  SystemFrequency / 1000000 1us中断一次
*/
uint32_t tick_count;
/*
 * 函数名：systick_init
 * 描述  ：系统滴答定时器配置初始化
 * 输入  ： 
 * 返回  ： 
 */
void systick_init(void)
{
	SystemCoreClockUpdate();
    //时钟频率：72Mhz ， 每秒可以计数72000000次，一次的时间则为：1/72000（ms），10ms需要的次数：10/(1/72000) = 720000 -> 5ms需要的次数则为：360000
	if (SysTick_Config(SystemCoreClock / 200))	        //1000 -> 1ms
	{ 
		/* Capture error */ 
		while (1);
	}
        
	SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;          //使能
}


uint32_t softTimes[5] = {0,0,0,0,0};                        //软定时时间存储
/*
 * 函数名：softTimesCountdown
 * 描述  ：软定时器倒计时
 * 输入  ： 
 * 返回  ： 
 */
void softTimesCountdown(void)
{
	uint8_t i;
	for(i = 0;  i < 5; i++)
    {
		if(softTimes[i] > 0)
            softTimes[i]--;
	} 
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    softTimesCountdown();                                                   //软件定时倒计时
    UltrasonicCheck();                                                      //超声波在位检测
    get_gyro_raw();                                                         //陀螺仪数据
    get_deg_s(&gyro_raw_f,&Mpu.deg_s);                                      //陀螺仪原始数据转为度为单位的速率    
    get_rad_s(&gyro_raw_f,&Mpu.rad_s);                                      //陀螺仪原始数据转为弧度为单位的速率
    get_acc_raw();                                                          //加速度数据
    acc_iir_lpf(&acc_raw_f,&acc_att_lpf,Mpu.att_acc_factor);                //姿态解算时加速度低通滤波 
    get_acc_g(&acc_att_lpf,&Mpu.acc_g);  
                                                                            //姿态解算
    mahony_update(Mpu.rad_s.x,Mpu.rad_s.y,Mpu.rad_s.z,Mpu.acc_g.x,Mpu.acc_g.y,Mpu.acc_g.z); 
    Matrix_ready();                                                         //姿态解算相关矩阵更新

    encoder_val_a = read_encoder(ENCODER_A);                                //A编码器值读取
    encoder_manage(&encoder_val_a);                                         //编码器值处理
    encoder_val_b = read_encoder(ENCODER_B);                                //B编码器值读取    
    encoder_manage(&encoder_val_b);                                         //编码器值处理

    ctr.pwm[0] = ctr_bal(att.rol,Mpu.deg_s.y);                              //角度直立平衡控制器
    ctr.pwm[1] = ctr_vel(-encoder_val_a,encoder_val_b);                     //速度控制器
    ctr.pwm[2] = ctr_turn(-encoder_val_a,encoder_val_b,Mpu.deg_s.z);        //转向控制器
   
    ctr.out[0] = ctr.pwm[0] + ctr.pwm[1] + ctr.pwm[2];                      //电机1匹配输出           
    ctr.out[1] = ctr.pwm[0] + ctr.pwm[1] - ctr.pwm[2];                      //电机2匹配输出

    i_limit(&(ctr.out[0]),AMPLITUDE);                                       //输出限幅
    i_limit(&(ctr.out[1]),AMPLITUDE);                                       //输出限幅
     
    dir_config(&(ctr.out[0]),&(ctr.out[1]));                                //根据正负设置方向

    _ctr_out();                                                             //控制器输出
    
    ANO_DMA_SEND_DATA();                                                    //地面站波形显示
    gyro_cal(&gyro_raw_cal);                                                //陀螺仪零偏校准
}


