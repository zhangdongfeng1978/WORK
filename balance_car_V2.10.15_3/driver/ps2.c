/**********************************************************************
版权所有：源动力科技
淘    宝：https://1024tech.taobao.com/
文 件 名：ps2.c
版    本：V21.01.30
摘    要：
***********************************************************************/
#include "ps2.h"
#include "imath.h"
#include "bluetooth.h"
#include "usart1.h"
#define DELAY_TIME  tdelay_us(5); 

//临时存储按键值数据
u16 Handkey;	    

//开始命令，请求数据
u8 Comd[2] = {0x01,0x42};	  

//数据存储数组
u8 Data[9] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};  

u16 MASK[] =
{
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1 ,
    PSB_GREEN,
    PSB_RED,
    PSB_BLUE,
    PSB_PINK
};	    

//手柄接口初始化
void PS2_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin    = DO_pin | CS_pin | CLK_pin;
    GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_Out_PP;                   
    GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin    = DI_pin;
    GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_IPD;                     
    GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_50MHz;
    GPIO_Init(DI_port, &GPIO_InitStructure);    
}

//向手柄发送命令
void PS2_Cmd(u8 CMD)
{
	volatile u16 ref=0x01;
	Data[1] = 0;                                            
    
	for(ref=0x01; ref<0x0100; ref<<=1)
	{
		if(ref&CMD)
		{
			DO_H;                       //输出有效              
		}
		else DO_L;

		CLK_H;                          
		DELAY_TIME;
		CLK_L;                          //拉低时钟
		DELAY_TIME;
		CLK_H;
        
		if(DI)                          //读取数据     
			Data[1] = ref | Data[1];    //Data[1]存储返回ID
	}
	tdelay_us(16);
}

//判断是否为红色模式
//0x41:模拟绿灯，0x73：模拟红灯
u8 PS2_RedLight(void)
{
	CS_L;
	PS2_Cmd(Comd[0]);                   //开始命令  
	PS2_Cmd(Comd[1]);                   //请求数据
	CS_H;
    
	if( Data[1] == 0X73)               
        return 1 ;                      //红灯模式
    else if( Data[1] == 0X41)           //绿灯模式
        return 2;
	else 
        return 0;
}

//读取手柄数据
void PS2_ReadData(void)
{
	volatile u8 byte = 0;
	volatile u16 ref = 0x01;
    
	CS_L;
	PS2_Cmd(Comd[0]);                   //开始命令
	PS2_Cmd(Comd[1]);                   //请求数据
    
	for(byte=2; byte<9; byte++)         //开始接受数据           
	{
		for(ref=0x01; ref<0x100; ref<<=1)
		{
			CLK_H;
			DELAY_TIME;
			CLK_L;
			DELAY_TIME;
			CLK_H;
            
		    if(DI)
		      Data[byte] = ref | Data[byte];  //存储手柄数据
		}
        tdelay_us(16);
	}
	CS_H;
}

//清除数据缓冲区
void PS2_ClearData(void)
{
	u8 a;
	for(a=0; a<9; a++)
		Data[a] = 0x00;
}

//对PS2按键部分数据进行处理
u8 PS2_DataKey(void)
{
	u8 index;

	PS2_ClearData();                                //清除数组数据
	PS2_ReadData();                                 //接收PS2遥控器数据

	Handkey = (Data[4]<<8) | Data[3];               //存储16位按键数据，总共16颗按键，按下为0，未按下为1
	for(index=0; index<16; index++)
	{	    
		if((Handkey&(1<<(MASK[index]-1))) == 0)     //1<<(MASK[index]-1)即让0000 0000 0000 0001中唯一的1左移这个位数X
            return index+1;             
	}
	return 0;                                       //无按键按下 
}

//得到一个摇杆的模拟量：0~256
u8 PS2_AnologData(u8 button)
{
	return Data[button];
}

//手柄震动函数
//motor1：右侧小震动电机 0x00：电机关  其他开
//motor2：左侧大震动电机 0x40~0xff 电机开，值越大，震动越大
void PS2_Vibration(u8 motor1, u8 motor2)
{
	CS_L;
	tdelay_us(16);
    PS2_Cmd(0x01);                                  //开始命令
	PS2_Cmd(0x42);                                  //请求数据
	PS2_Cmd(0X00);
	PS2_Cmd(motor1);
	PS2_Cmd(motor2);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	tdelay_us(16);  
}

//short poll
void PS2_ShortPoll(void)
{
	CS_L;
	tdelay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x42);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x00);
	CS_H;
	tdelay_us(16);	
}

//进入配置
void PS2_EnterConfing(void)
{
    CS_L;
	tdelay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01);
	PS2_Cmd(0x00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	tdelay_us(16);
}

//发送模式配置
void PS2_TurnOnAnalogMode(void)
{
	CS_L;
	PS2_Cmd(0x01);  
	PS2_Cmd(0x44);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01);                          //analog=0x01;digital=0x00   软件设置发送模式
	PS2_Cmd(0x03);                          //Ox03锁存设置，即不可通过按键"MODE"设置模式   
                                            //0xEE不锁存软件设置，可通过按键"MODE"设置模式   
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	tdelay_us(16);
}

//震动模式
void PS2_VibrationMode(void)
{
	CS_L;
	tdelay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x4D);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0X01);
	CS_H;
	tdelay_us(16);	
}

//完成并保存配置
void PS2_ExitConfing(void)
{
    CS_L;
	tdelay_us(16);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	CS_H;
	tdelay_us(16);
}

//手柄初始化设置
void PS2_SetInit(void)
{
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_EnterConfing();	                //进入配置模式	
	PS2_TurnOnAnalogMode();	            //红绿灯配置模式，并选择是否保存
	PS2_VibrationMode();	            //开启震动模式
	PS2_ExitConfing();		            //完成并保存设置
}

_PS2_DATA PS2 = {0};

//PS2遥控器数据读取
void ps2_data_sample(void)
{
    
//    PS2.MODE = PS2_RedLight();                  //根据判断模式从而来判断是否连接正常
//    if(PS2.MODE==0)                             //连接断开，清除数据    
//    {
//        PS2.KEY = 0;
//        PS2.RX =  0;
//        PS2.RY =  0;
//        PS2.LX =  0;
//        PS2.LY =  0;
//        PS2.OpenOrClose = 1;  
//    }
//    else                                        //正常连接时则读取数据
    {
//        PS2.KEY = PS2_DataKey();                //读取按键数据
        PS2.KEY = protectedData(PS2_DataKey());
        
        PS2.RX = PS2_AnologData(PSS_RX);        //右摇杆数据读取
        PS2.RY = PS2_AnologData(PSS_RY);        //右摇杆数据读取
        PS2.LX = PS2_AnologData(PSS_LX);        //左摇杆数据读取
        PS2.LY = PS2_AnologData(PSS_LY);        //左摇杆数据读取
        if(PS2.KEY==PSB_L2)
            PS2.OpenOrClose = 0;
        else if(PS2.KEY==PSB_R2)
            PS2.OpenOrClose = 1;        
    }

//    printf("%d %d %d %d %d\r\n",PS2.KEY,PS2.RX,PS2.RY,PS2.LX,PS2.LY);
}


