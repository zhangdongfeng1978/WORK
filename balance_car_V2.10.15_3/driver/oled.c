/**********************************************************************
版权所有：源动力科技
淘    宝：https://1024tech.taobao.com/
文 件 名：oled.c
版    本：V21.01.30
摘    要：
***********************************************************************/      
#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"  	
#include "systick.h"
#include "mpu6050.h"
#include "imu.h"
#include "pwm.h"
#include "adc.h"
#include "controller.h"
#include <string.h>
#include "stdio.h"
#include "imath.h"
#include "ultrasonic.h"
//OLED显存存放格式
//[0]0 1 2 3 ... 127	
//[1]0 1 2 3 ... 127	
//[2]0 1 2 3 ... 127	
//[3]0 1 2 3 ... 127	
//[4]0 1 2 3 ... 127	
//[5]0 1 2 3 ... 127	
//[6]0 1 2 3 ... 127	
//[7]0 1 2 3 ... 127 			   
void IICO_Start()
{
	OLED_SCLK_Set() ;
	OLED_SDIN_Set();
	OLED_SDIN_Clr();
	OLED_SCLK_Clr();    
}
void IICO_Stop()
{
OLED_SCLK_Set() ;
//	OLED_SCLK_Clr();
	OLED_SDIN_Clr();
	OLED_SDIN_Set();	
}
void IICO_Wait_Ack()
{
	//GPIOB->CRH &= 0XFFF0FFFF;	
	//GPIOB->CRH |= 0x00080000;
//	OLED_SDA = 1;
//	delay_us(1);
	//OLED_SCL = 1;
	//delay_us(50000);
/*	while(1)
	{
		if(!OLED_SDA)				
		{
			//GPIOB->CRH &= 0XFFF0FFFF;	
			//GPIOB->CRH |= 0x00030000;
			return;
		}
	}
*/
	OLED_SCLK_Set() ;
	OLED_SCLK_Clr();
}
void Write_IIC_Byte(unsigned char IIC_Byte)
{
	unsigned char i;
	unsigned char m,da;
	da=IIC_Byte;
	OLED_SCLK_Clr();
	for(i=0;i<8;i++)		
	{
        m=da;
		//	OLED_SCLK_Clr();
		m=m&0x80;
		if(m==0x80)
		{
            OLED_SDIN_Set();
        }
        else 
                OLED_SDIN_Clr();
        da=da<<1;
        OLED_SCLK_Set();
        OLED_SCLK_Clr();
    }
}
void Write_IIC_Command(unsigned char IIC_Command)
{
    IICO_Start();
    Write_IIC_Byte(0x78);            //Slave address,SA0=0
    IICO_Wait_Ack();	
    Write_IIC_Byte(0x00);			//write command
    IICO_Wait_Ack();	
    Write_IIC_Byte(IIC_Command); 
    IICO_Wait_Ack();	
    IICO_Stop();
}
void Write_IIC_Data(unsigned char IIC_Data)
{
    IICO_Start();
    Write_IIC_Byte(0x78);			//D/C#=0; R/W#=0
    IICO_Wait_Ack();	
    Write_IIC_Byte(0x40);			//write data
    IICO_Wait_Ack();	
    Write_IIC_Byte(IIC_Data);
    IICO_Wait_Ack();	
    IICO_Stop();
}
void OLED_WR_Byte(unsigned dat,unsigned cmd)
{
	if(cmd)Write_IIC_Data(dat);	
	else Write_IIC_Command(dat);
}
//坐标设置
void OLED_Set_Pos(unsigned char x, unsigned char y) 
{ 
	OLED_WR_Byte(0xb0+y,OLED_CMD);
	OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
	OLED_WR_Byte((x&0x0f),OLED_CMD); 
}   	  
//开启OLED显示  
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDC命令
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//关闭OLED显示   
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDCÃüÁî
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}		   		
void OLED_Clear(void)  
{  
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    
		OLED_WR_Byte (0x00,OLED_CMD);      
		OLED_WR_Byte (0x10,OLED_CMD);         
		for(n=0;n<128;n++)OLED_WR_Byte(0,OLED_DATA); 
	} 
}
void OLED_On(void)  
{  
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //设置页地址0-7
		OLED_WR_Byte (0x00,OLED_CMD);      //设置显示位置-列高地址
		OLED_WR_Byte (0x10,OLED_CMD);      //设置显示位置-列低地址
		for(n=0;n<128;n++)OLED_WR_Byte(1,OLED_DATA); 
    }   
}
//在指定位置显示一个字符
//x:0~127  y:0~63
//mode:0,反白显示;1,正常显示	 
//size:选择字体 16/12 
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 Char_Size)
{      	
    unsigned char c=0,i=0;	
    c=chr-' ';                              //得到偏移后的地址			
    if(x>Max_Column-1)
    {
        x=0;
        y=y+2;
    }
    if(Char_Size ==16)
    {
        OLED_Set_Pos(x,y);	
        for(i=0;i<8;i++)
            OLED_WR_Byte(F8X16[c*16+i],OLED_DATA);
        OLED_Set_Pos(x,y+1);
        for(i=0;i<8;i++)
            OLED_WR_Byte(F8X16[c*16+i+8],OLED_DATA);
    }
    else
    {	
        OLED_Set_Pos(x,y);
        for(i=0;i<6;i++)
            OLED_WR_Byte(F6x8[c][i],OLED_DATA);			
    }
}
u32 oled_pow(u8 m,u8 n)//m^n函数
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}				  
//显示两个数字
//x,y :起点坐标
//len :数字的位数
//size:字体大小
//mode:模式0,填充模式;1,叠加模式
//num:数值(0~4294967295);	 		  
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size2)
{         	
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size2/2)*t,y,' ',size2);
				continue;
			}
			else enshow=1;  	 
		}
	 	OLED_ShowChar(x+(size2/2)*t,y,temp+'0',size2); 
	}
} 
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 Char_Size)    //显示一个字符号串
{
	unsigned char j=0;
	while (chr[j]!='\0')
	{
		OLED_ShowChar(x,y,chr[j],Char_Size);
		x+=8;
		if(x>120){x=0;y+=2;}j++;
	}
}
void OLED_ShowCHinese(u8 x,u8 y,u8 no)                  //显示汉字
{      			    
	u8 t,adder=0;
	OLED_Set_Pos(x,y);	
    for(t=0;t<16;t++)
    {
        OLED_WR_Byte(Hzk[2*no][t],OLED_DATA);
        adder+=1;
    }	
    OLED_Set_Pos(x,y+1);	
    for(t=0;t<16;t++)
    {	
        OLED_WR_Byte(Hzk[2*no+1][t],OLED_DATA);
        adder+=1;
    }					
}	    
void OLED_Init(void)                                                //初始化SSD1306		
{ 	 
 	GPIO_InitTypeDef  GPIO_InitStructure;
 	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;	 
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		    
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   
 	GPIO_Init(GPIOB, &GPIO_InitStructure);	    
 	GPIO_SetBits(GPIOB,GPIO_Pin_0|GPIO_Pin_1);	

	tdelay_ms(800);
	OLED_WR_Byte(0xAE,OLED_CMD);//--display off
	OLED_WR_Byte(0x00,OLED_CMD);//---set low column address
	OLED_WR_Byte(0x10,OLED_CMD);//---set high column address
	OLED_WR_Byte(0x40,OLED_CMD);//--set start line address  
	OLED_WR_Byte(0xB0,OLED_CMD);//--set page address
	OLED_WR_Byte(0x81,OLED_CMD); // contract control
	OLED_WR_Byte(0xFF,OLED_CMD);//--128   
	OLED_WR_Byte(0xA1,OLED_CMD);//set segment remap 
	OLED_WR_Byte(0xA6,OLED_CMD);//--normal / reverse
	OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
	OLED_WR_Byte(0x3F,OLED_CMD);//--1/32 duty
	OLED_WR_Byte(0xC8,OLED_CMD);//Com scan direction
	OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset
	OLED_WR_Byte(0x00,OLED_CMD);//	
	OLED_WR_Byte(0xD5,OLED_CMD);//set osc division
	OLED_WR_Byte(0x80,OLED_CMD);//	
	OLED_WR_Byte(0xD8,OLED_CMD);//set area color mode off
	OLED_WR_Byte(0x05,OLED_CMD);//	
	OLED_WR_Byte(0xD9,OLED_CMD);//Set Pre-Charge Period
	OLED_WR_Byte(0xF1,OLED_CMD);//	
	OLED_WR_Byte(0xDA,OLED_CMD);//set com pin configuartion
	OLED_WR_Byte(0x12,OLED_CMD);//	
	OLED_WR_Byte(0xDB,OLED_CMD);//set Vcomh
	OLED_WR_Byte(0x30,OLED_CMD);//	
	OLED_WR_Byte(0x8D,OLED_CMD);//set charge pump enable
	OLED_WR_Byte(0x14,OLED_CMD);//	
	OLED_WR_Byte(0xAF,OLED_CMD);//--turn on oled panel
    
    OLED_Clear() ;
}  

/* 
函数功能：OLED显示小车的角度
*/
static int OledShowAngle(void)
{
    int err;
    OLED_ShowString(0,0,"angle:",12);      
    if(att.rol>=0)
    {
        OLED_ShowString(82,0,"+",12);
        OLED_ShowNum(90,0,(u32)(att.rol),3,12);
    }
    else          
    {
        OLED_ShowString(82,0,"-",12);
        OLED_ShowNum(90,0,(u32)((-att.rol)),3,12);
    }    
    
    return err;
}
/* 
函数功能：OLED显示小车的角速度
*/
static int OledShowDegs(void)
{
    int err;
    //Mpu.deg_s.y
    OLED_ShowString(0,1,"vel:",12);     
    if(Mpu.deg_s.y>=0)
    {
        OLED_ShowString(82,1,"+",12);
        OLED_ShowNum(90,1,(u32)(Mpu.deg_s.y),3,12);
    }
    else          
    {
        OLED_ShowString(82,1,"-",12);
        OLED_ShowNum(90,1,(u32)(-Mpu.deg_s.y),3,12);
    }    
    return err;
}
/* 
函数功能：OLED显示小车的电源电压
*/
static int OledShowVbat(void)
{
    int err;
    //vbat
    OLED_ShowString(0,2,"vbat:",12);   
    OLED_ShowNum(82,2,(u32)(bat.voltage),1,12);
    OLED_ShowString(89,2,".",12);    
    OLED_ShowNum(95,2,(u32)((u32)(bat.voltage*10)%10),1,12);
    OLED_ShowString(105,2,"v",12);
//    if(bat.voltage>=7.0f)
//        OLED_ShowNum(100,2,(u32)((u32)((bat.voltage/8.4f)*100)),3,12);
//    else 
//        OLED_ShowNum(100,2,(u32)0,3,12);
//    OLED_ShowString(120,2,"%",12); 

    return err;    
}
/* 
函数功能：OLED显示小车的轮子测速编码值
*/
static int OledShowEncoder(void)
{
    int err;
    //encoder
    //L
    OLED_ShowString(0,3,"ValLeft:",12);
    if(encoder_val_a>=0)
    {
        OLED_ShowString(82,3,"+",12);
        OLED_ShowNum(90,3,encoder_val_a,3,12);
    }
    else
    {
        OLED_ShowString(82,3,"-",12);
        OLED_ShowNum(90,3,-(encoder_val_a),3,12);
    }
    //R
    OLED_ShowString(0,4,"ValRight:",12);
    if(encoder_val_b>=0)
    {
        OLED_ShowString(82,4,"+",12);
        OLED_ShowNum(90,4,encoder_val_b,3,12);
    }
    else
    {
        OLED_ShowString(82,4,"-",12);
        OLED_ShowNum(90,4,-(encoder_val_b),3,12);
    }    
    return err;
}
/*
函数功能：OLED显示小车的前后方向增加/减少的速度变量幅度
*/
static int OledShowStepForwardOrBackward(void)
{
    int err;
    //fb
    OLED_ShowString(0,4,"fb:",12);
    if(CarStepForwardOrBackward>=0)
    {
        OLED_ShowString(25,4,"+",12);
        OLED_ShowNum(30,4,CarStepForwardOrBackward,3,12);
    }
    else
    {
        OLED_ShowString(25,4,"-",12);
        OLED_ShowNum(30,4,-(CarStepForwardOrBackward),3,12);
    }  
    //lr
    OLED_ShowString(60,4,"lr:",12);
    if(CarStepLeftOrRight>=0)
    {
        OLED_ShowString(85,4,"+",12);
        OLED_ShowNum(95,4,CarStepLeftOrRight,4,12);
    }
    else
    {
        OLED_ShowString(85,4,"-",12);
        OLED_ShowNum(95,4,-(CarStepLeftOrRight),4,12);
    }  
    return err;
}

static int OledShowDistance(void)
{
    int err;
    unsigned char buff[32]={0};

    snprintf((char*)buff, 21,"distance: %.1fcm", getDistance);
    OLED_ShowString(0,5,buff,12);
    
    return err;
} 

void OledShowDatas(void)
{	
    OledShowAngle();
    OledShowDegs();
    OledShowVbat();
    OledShowEncoder();
    OledShowEncoder();
//    OledShowStepForwardOrBackward();
    OledShowDistance(); 
    OLED_ShowString(0,7," ----working----",12); 
}
//填充函数
void fill_picture(unsigned char fill_Data)      
{
	unsigned char m,n;
	for(m=0;m<8;m++)
	{
		OLED_WR_Byte(0xb0+m,0);	    //´0-7页
		OLED_WR_Byte(0x00,0);		//low column start address
		OLED_WR_Byte(0x10,0);		//high column start address
		for(n=0;n<128;n++)OLED_WR_Byte(fill_Data,1);	
	}
}



