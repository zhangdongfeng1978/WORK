/**********************************************************************
版权所有：源动力科技
淘    宝：https://1024tech.taobao.com/
文 件 名：usart1.c
版    本：V21.01.30
摘    要：
1、usart1既用作与匿名上位机通信的串口，也用于printf打印数据
2、usart1的RX（PA10）当用于与上位机通信作为接收上位机数据时，需要关闭红外寻迹模块功能（与OUT5引脚共用），方可接收上位机数据
***********************************************************************/
#include "usart1.h"
#include "mpu6050.h"
#include "imu.h"
#include "imath.h"
#include "systick.h"
#include "key.h"
#include "pwm.h"
#include "pid.h"
#include "controller.h"
#include "timer.h"
#include "flash.h"
#include "led.h"
#include "bluetooth.h"

uint8_t data_to_send[100];                  //发送数据缓存
uint8_t USART_RX_DATA[USART_RX_LEN];        //接收数据缓存
dt_flag_t f;					            //需要发送数据的标志 

#pragma import(__use_no_semihosting)
 
struct __FILE
{
	int a;
};
 
FILE __stdout;
 
void _sys_exit(int x)
{
	
}
int fputc(int ch,FILE *f)
{
    USART1->SR; 
    USART_SendData(USART1, (unsigned char) ch);
    while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);
    return(ch);
} 
/* 串口1初始化设置 */
/* 入口参数：波特率 */
void usart1_init(u32 bound)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;

    //USART1 Tx(PA.09) 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    //USART1 Rx(PA.10) 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = bound; 
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; 
    USART_InitStructure.USART_StopBits = USART_StopBits_1; 
    USART_InitStructure.USART_Parity = USART_Parity_No; 
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; 
    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);                                    //rx interrupt is enable

    USART_Cmd(USART1, ENABLE);   
}
/* 串口DMA通道配置                */
/* DMA_CHx:         DMA传输通道x     */
/* peripheral_addr: 外设地址         */
/* memory_addr:     内存地址         */
/* data_length:     传输的数据长度   */  
void USARTx_DMA_TX_Config(DMA_Channel_TypeDef* DMA_CHx,u32 peripheral_addr,u32 memory_addr,u16 data_length)
{
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	                        //时钟使能                                                                              
    DMA_DeInit(DMA_CHx);                                                        //复位
                                                                                
    DMA_InitStructure.DMA_PeripheralBaseAddr = peripheral_addr;                 //外设地址     
    DMA_InitStructure.DMA_MemoryBaseAddr =memory_addr;                          //内存地址  
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;                          //外设作为传输的目的地
    DMA_InitStructure.DMA_BufferSize = data_length;                             //数据缓存大小                       
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;            //外设地址不自增
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                     //内存地址自增   
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;     //外设数据宽度8位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;             //内存数据宽度8位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                               //正常模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                     //高优先级
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                //无内存到内存传输                    
    DMA_Init(DMA_CHx, &DMA_InitStructure);                                 
}

/* 串口DMA数据发送 */
void USARTx_DMA_SEND_DATA(u32 SendBuff,u16 len) 
{
	USARTx_DMA_TX_Config(DMA1_Channel4,(u32)&USART1->DR,(u32)SendBuff,len);
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);                              //使能串口DMA发送
    DMA_Cmd(DMA1_Channel4, ENABLE);                                             //使能DMA传输
}

/* 将大于一个字节的数据拆分成多个字节发送 */
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)    ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

/* 向匿名上位机发送姿态角，锁定状态 */
void ANO_DT_Send_Status(void)
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2;
	u8 sum = 0;
	u8 i;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;

	_temp = (int)(att.rol*100);                     //横滚角
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
    
	_temp = (int)(att.pit*100);                     //俯仰角    
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
    
	_temp = (int)(att.yaw*100);                     //偏航角
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	_temp2 = (int32_t)(100*99.99);       		    //高度 
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);

    data_to_send[_cnt++]=0x01;  					//飞行模式    01：姿态  02：定高  03：定点
    data_to_send[_cnt++]=0x01;                      //锁定状态

	data_to_send[3] = _cnt-4;
	sum = 0;
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
    
    USARTx_DMA_SEND_DATA((u32)(data_to_send),_cnt); //发送           
}

 void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
{
    u8 _cnt=0;
    vs16 _temp;
    u8 sum = 0;
    u8 i=0;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x02;
    data_to_send[_cnt++]=0;

    _temp = a_x;    
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = a_y;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = a_z;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    _temp = g_x;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = g_y;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = g_z;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    _temp = m_x;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = m_y;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = m_z;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    data_to_send[3] = _cnt-4;

    sum = 0;
    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
    
    USARTx_DMA_SEND_DATA((u32)(data_to_send),_cnt);
}

/* 遥控器通道数据 */
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
    u8 _cnt=0;
    u8 i=0;
    u8 sum = 0;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x03;
    data_to_send[_cnt++]=0;
    
    data_to_send[_cnt++]=BYTE1(thr);
    data_to_send[_cnt++]=BYTE0(thr);
    data_to_send[_cnt++]=BYTE1(yaw);
    data_to_send[_cnt++]=BYTE0(yaw);
    data_to_send[_cnt++]=BYTE1(rol);
    data_to_send[_cnt++]=BYTE0(rol);
    data_to_send[_cnt++]=BYTE1(pit);
    data_to_send[_cnt++]=BYTE0(pit);
    
    data_to_send[_cnt++]=BYTE1(aux1);
    data_to_send[_cnt++]=BYTE0(aux1);
    data_to_send[_cnt++]=BYTE1(aux2);
    data_to_send[_cnt++]=BYTE0(aux2);
    data_to_send[_cnt++]=BYTE1(aux3);
    data_to_send[_cnt++]=BYTE0(aux3);
    data_to_send[_cnt++]=BYTE1(aux4);
    data_to_send[_cnt++]=BYTE0(aux4);
    data_to_send[_cnt++]=BYTE1(aux5);
    data_to_send[_cnt++]=BYTE0(aux5);
    data_to_send[_cnt++]=BYTE1(aux6);
    data_to_send[_cnt++]=BYTE0(aux6);

    data_to_send[3] = _cnt-4;

    sum = 0;
    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];
    data_to_send[_cnt++]=sum;
    
    USARTx_DMA_SEND_DATA((u32)(data_to_send),_cnt);
}

void ANO_DT_Send_Power(float votage, float current)
{
	u8 _cnt=0;
	u16 temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0;
	
	temp = (uint16_t)100*votage;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	temp = (uint16_t)100*current;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	USARTx_DMA_SEND_DATA((u32)(data_to_send), _cnt);
}
//用户自定义数据发送：1-5：int16t类型数据     6-10：float类型数据
 void ANO_DT_Send_User(s16 user1,s16 user2,s16 user3,s16 user4,s16 user5,
                        float user6, float user7, float user8, float user9, float user10,
                        float user11,float user12,float user13,float user14,float user15)
{
    u8 _cnt=0;
    vs16 _temp;
    float _temp_f;
    
    u8 sum = 0;
    u8 i=0;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xF1;
    data_to_send[_cnt++]=0;
    
    //1-5  int16t类型数据
    _temp = user1;    
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = user2;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = user3;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = user4;    
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);
    _temp = user5;
    data_to_send[_cnt++]=BYTE1(_temp);
    data_to_send[_cnt++]=BYTE0(_temp);

    //6-10 ：float类型数据
    _temp_f = user6;
	data_to_send[_cnt++]=BYTE3(_temp_f);
	data_to_send[_cnt++]=BYTE2(_temp_f);
	data_to_send[_cnt++]=BYTE1(_temp_f);
	data_to_send[_cnt++]=BYTE0(_temp_f);
    _temp_f = user7;
	data_to_send[_cnt++]=BYTE3(_temp_f);
	data_to_send[_cnt++]=BYTE2(_temp_f);
	data_to_send[_cnt++]=BYTE1(_temp_f);
	data_to_send[_cnt++]=BYTE0(_temp_f);
    _temp_f = user8;
	data_to_send[_cnt++]=BYTE3(_temp_f);
	data_to_send[_cnt++]=BYTE2(_temp_f);
	data_to_send[_cnt++]=BYTE1(_temp_f);
	data_to_send[_cnt++]=BYTE0(_temp_f);
    _temp_f = user9;
	data_to_send[_cnt++]=BYTE3(_temp_f);
	data_to_send[_cnt++]=BYTE2(_temp_f);
	data_to_send[_cnt++]=BYTE1(_temp_f);
	data_to_send[_cnt++]=BYTE0(_temp_f);
    _temp_f = user10;
	data_to_send[_cnt++]=BYTE3(_temp_f);
	data_to_send[_cnt++]=BYTE2(_temp_f);
	data_to_send[_cnt++]=BYTE1(_temp_f);
	data_to_send[_cnt++]=BYTE0(_temp_f);

    _temp_f = user11;
	data_to_send[_cnt++]=BYTE3(_temp_f);
	data_to_send[_cnt++]=BYTE2(_temp_f);
	data_to_send[_cnt++]=BYTE1(_temp_f);
	data_to_send[_cnt++]=BYTE0(_temp_f);
    _temp_f = user12;
	data_to_send[_cnt++]=BYTE3(_temp_f);
	data_to_send[_cnt++]=BYTE2(_temp_f);
	data_to_send[_cnt++]=BYTE1(_temp_f);
	data_to_send[_cnt++]=BYTE0(_temp_f);
    _temp_f = user13;
	data_to_send[_cnt++]=BYTE3(_temp_f);
	data_to_send[_cnt++]=BYTE2(_temp_f);
	data_to_send[_cnt++]=BYTE1(_temp_f);
	data_to_send[_cnt++]=BYTE0(_temp_f);
    _temp_f = user14;
	data_to_send[_cnt++]=BYTE3(_temp_f);
	data_to_send[_cnt++]=BYTE2(_temp_f);
	data_to_send[_cnt++]=BYTE1(_temp_f);
	data_to_send[_cnt++]=BYTE0(_temp_f);
    _temp_f = user15;
	data_to_send[_cnt++]=BYTE3(_temp_f);
	data_to_send[_cnt++]=BYTE2(_temp_f);
	data_to_send[_cnt++]=BYTE1(_temp_f);
	data_to_send[_cnt++]=BYTE0(_temp_f);

    data_to_send[3] = _cnt-4;

    sum = 0;
    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;
    
    USARTx_DMA_SEND_DATA((u32)(data_to_send),_cnt);
}

void ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;

	_temp = p1_p * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_d  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;

	USARTx_DMA_SEND_DATA((u32)(data_to_send),_cnt);
}
/*
在上位机对下位机进行操作时，下位机做出对应的反馈信息
 MSG_ID:                 MSG_DATA:
01:加速度               01:校准成功
02:陀螺仪               E1:校准失败
03:罗盘                 31:设置成功
30:无线定位模块         32:设置成功2
40:匿名数传             A1:恢复默认成功
 */
static void ANO_DT_Send_MSG(u8 MSG_ID, u8 MSG_DATA)
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEE;
	data_to_send[3]=2;                      //长度
	data_to_send[4]=MSG_ID;                 //功能字
	data_to_send[5]=MSG_DATA;
	
	u8 sum = 0;
	for(u8 i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	USARTx_DMA_SEND_DATA((u32)(data_to_send),7);
}

void ANO_DMA_SEND_DATA(void)
{
    static uint8_t ANO_debug_cnt = 0;
    ANO_debug_cnt++;
    if(ANO_debug_cnt==1)
    {
        ANO_DT_Send_Status();
    }
    else if(ANO_debug_cnt==2)
    { 
        ANO_DT_Send_Senser((int16_t)acc_raw.x,(int16_t)acc_raw.y,(int16_t)acc_raw.z, 
                           (int16_t)gyro_raw.x,(int16_t)gyro_raw.y,(int16_t)gyro_raw.z,
                           (int16_t)0,(int16_t)0,(int16_t)0);
    }
    else if(ANO_debug_cnt==3)
    {
        ANO_DT_Send_RCData(1100,1200,1300,1400,1500,1600,1700,1800,1900,1100);
    }
    else if(ANO_debug_cnt==4)
    {
        ANO_DT_Send_Power(13.52,57.63);
    }
    else if(ANO_debug_cnt==5)
    {
        ANO_DT_Send_User(0,0,0,0,0,
                        encoder_val_a, encoder_val_b,ctr.pwm[0],ctr.pwm[1],acc_raw_f.z,
                        0,0,0,0,0);
    }
    else if(ANO_debug_cnt==6)
    {
        if(f.send_pid1)
        {
            f.send_pid1 = 0;
            //向上位机发送内环速度环PID参数值
            ANO_DT_Send_PID(1,  vel.kp*0.1f,
                                vel.ki,
                                vel.kd,
                                1.5f,
                                2.5f,
                                1.5f,
                                2.5f,
                                1.5f,
                                2.5f);        
        }
        if(f.send_pid2)
        {
            f.send_pid2 = 0;
            //向上位机发送外环角度环PID参数值
            ANO_DT_Send_PID(2,  bal.kp*0.1f,
                                bal.ki,
                                bal.kd,
                                2.6f,
                                1.5f,
                                2.4f,
                                1.1f,
                                2.2f,
                                1.1f);   
        }
        if(CalGyro.success==1)                          //返回校准成功信息给上位机
        {
            CalGyro.success = 0;
            ANO_DT_Send_MSG(SENSOR_GYRO,CAL_SUCCESS);
        }
        else if(CalGyro.success==2)                     //反馈校准失败信息给上位机
        {
            CalGyro.success = 0;
            ANO_DT_Send_MSG(SENSOR_GYRO,CAL_SUCCESS);
        }
    }
    else if(ANO_debug_cnt==7)
    {
        ANO_debug_cnt = 0;
    }
}
static void ANO_DT_Send_Check(u8 head, u8 check_sum)
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;                  //长度
	data_to_send[4]=head;               //功能字
	data_to_send[5]=check_sum;
	
	u8 sum = 0;
	for(u8 i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;

	USARTx_DMA_SEND_DATA((u32)(data_to_send),7);
}

//Data_Receive_Prepare函数是协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，格式正确的话再进行数据解析
//移植时，此函数应由用户根据自身使用的通信方式自行调用，比如串口每收到一字节数据，则调用此函数一次
//此函数解析出符合格式的数据帧后，会自行调用数据解析函数
void ANO_DT_Data_Receive_Prepare(u8 data)
{
	static u8 RxBuffer[50];
	static u8 _data_len = 0,_data_cnt = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAF)
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XF1)
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<50)
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;
		ANO_DT_Data_Receive_Anl(RxBuffer,_data_cnt+5);
	}
	else
		state = 0;
}
 
//Data_Receive_Anl函数是协议数据解析函数，函数参数是符合协议格式的一个数据帧，该函数会首先对协议数据进行校验
//校验通过后对数据进行解析，实现相应功能
//此函数可以不用用户自行调用，由函数Data_Receive_Prepare自动调用
void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num)
{  
	u8 sum = 0;
	for(u8 i=0;i<(num-1);i++)
    {
		sum += *(data_buf+i);        
    }
	if(!(sum==*(data_buf+num-1)))	                //判断sum	
        return;		
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))	//判断帧头	
        return;		
	
	if(*(data_buf+2)==0X01)                         //功能字01
	{
//        pu8printf("buf:",data_buf,num,1);
		if(*(data_buf+4)==0X01)                     //ACC校准
        {
        }            
//			mpu6050.Acc_CALIBRATE = 1;
		if(*(data_buf+4)==0X02)                     //GYRO校准
        {
            CalGyro.flag = 1;                       //接收到校准指令后置为1    
            _led.sta = 2;                           //默认无操作
        }
		if(*(data_buf+4)==0X03)
		{
//			mpu6050.Acc_CALIBRATE = 1;		
//			mpu6050.Gyro_CALIBRATE = 1;			
		}
	}
	if(*(data_buf+2)==0X02)                         //功能字02
	{
		if(*(data_buf+4)==0X01)                     //上位机读取pid请求
		{
			f.send_pid1 = 1;
			f.send_pid2 = 1;
			f.send_pid3 = 1;
			f.send_pid4 = 1;
			f.send_pid5 = 1;
			f.send_pid6 = 1;
//            ANO_DT_Send_Check(*(data_buf+2),sum);
		}
		if(*(data_buf+4)==0X02)     //读取飞行模式设置请求
		{
		}
		if(*(data_buf+4)==0XA0)		//读取版本信息
		{
			f.send_version = 1;
		}
		if(*(data_buf+4)==0XA1)		//恢复默认参数
		{
//            PidToFactorySetup();    //参数赋值
            ANO_DT_Send_Check(*(data_buf+2),sum);
            /* 将出厂内环PID参数写入flash */
            FlashWriteNineFloat(PID_Group1_ADDRESS, WriteFlashVel.kp,WriteFlashVel.ki,WriteFlashVel.kd,
                                                    WriteFlashBal.kp,WriteFlashBal.ki,WriteFlashBal.kd,
                                                    WriteFlashTur.kp,WriteFlashTur.ki,WriteFlashTur.kd);
            /* 将出厂外环PID参数写入flash */
//            FlashWriteNineFloat(PID_Group2_ADDRESS, all.rol_angle.kp,all.rol_angle.ki,all.rol_angle.kd,
//                                                    all.pit_angle.kp,all.pit_angle.ki,all.pit_angle.kd,
//                                                    all.yaw_angle.kp,all.yaw_angle.ki,all.yaw_angle.kd);
            flash_flag.pid = 0;
            FlashWriteByte(PID_FLAG_ADDRESS,flash_flag.pid);
		}
	}
	if(*(data_buf+2)==0X10)         //设置PID组1
    {
        /* PID内环速度环参数赋值 */
//        all.rol_gyro.kp = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
//        all.rol_gyro.ki = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
//        all.rol_gyro.kd = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
//        all.pit_gyro.kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
//        all.pit_gyro.ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
//        all.pit_gyro.kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
//        all.yaw_gyro.kp	= 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
//        all.yaw_gyro.ki = 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
//        all.yaw_gyro.kd = 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        ANO_DT_Send_Check(*(data_buf+2),sum);
        /* 将参数写入flash中存储 */
//        FlashWriteNineFloat(PID_Group1_ADDRESS, all.rol_gyro.kp,all.rol_gyro.ki,all.rol_gyro.kd,
//                                                all.pit_gyro.kp,all.pit_gyro.ki,all.pit_gyro.kd,
//                                                all.yaw_gyro.kp,all.yaw_gyro.ki,all.yaw_gyro.kd);
        flash_flag.pid = 1;
        /* 将标志位写入内存 */
        FlashWriteByte(PID_FLAG_ADDRESS,flash_flag.pid);
    }
    if(*(data_buf+2)==0X11)         //设置PID组2
    {
        /* PID外环角度环参数赋值 */
//        all.rol_angle.kp = 0.001*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
//        all.rol_angle.ki = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
//        all.rol_angle.kd = 0.001*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
//        all.pit_angle.kp = 0.001*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
//        all.pit_angle.ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
//        all.pit_angle.kd = 0.001*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
//        all.yaw_angle.kp = 0.001*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
//        all.yaw_angle.ki = 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
//        all.yaw_angle.kd = 0.001*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        ANO_DT_Send_Check(*(data_buf+2),sum);
        /* 将参数写入flash中存储 */
//        FlashWriteNineFloat(PID_Group2_ADDRESS, all.rol_angle.kp,all.rol_angle.ki,all.rol_angle.kd,
//                                                all.pit_angle.kp,all.pit_angle.ki,all.pit_angle.kd,
//                                                all.yaw_angle.kp,all.yaw_angle.ki,all.yaw_angle.kd);
        flash_flag.pid = 1;
        /* 将标志位写入内存 */
        FlashWriteByte(PID_FLAG_ADDRESS,flash_flag.pid);
    }
    if(*(data_buf+2)==0X12)         //设置PID组3
    {	
        ANO_DT_Send_Check(*(data_buf+2),sum);
    }
	if(*(data_buf+2)==0X13)         //设置PID组4
	{
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X14)         //设置PID组5
	{
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X15)         //设置PID组6
	{
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
}

uint16_t bufIndex = 0;
//调试串口接收中断     
void USART1_IRQHandler(void)                                 
{      
    u8 rcANO_DT;
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  
    {  
        if(bufIndex>USART_RX_LEN)
            bufIndex = 0;
		USART_RX_DATA[bufIndex++] = USART_ReceiveData(USART1);
        rcANO_DT = USART_ReceiveData(USART1);
        ANO_DT_Data_Receive_Prepare(rcANO_DT);
        
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);                             //清除空闲中断标志         
    }   
}   

