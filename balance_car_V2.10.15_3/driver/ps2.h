#ifndef _ps2_h_
#define _ps2_h_

#include "stm32f10x.h"
#include "systick.h"

#define PS2_REMOTE

#define DIR_ANGLE 10.0f

typedef struct
{
    uint8_t OpenOrClose;    
    
    uint8_t KEY;
    uint8_t RX;
    uint8_t RY;
    uint8_t LX;
    uint8_t LY;
    uint8_t MODE;
    
    float increment;
    int8_t dir_flag;
    float exp_dir;
    
}_PS2_DATA;

//DI 输入
#define DI_port  GPIOB
#define DI_pin   GPIO_Pin_12
#define DI       GPIO_ReadInputDataBit(DI_port,DI_pin)           

//DO 输出
#define DO_port  GPIOB
#define DO_pin   GPIO_Pin_13
#define DO_H DO_port->BSRR = DO_pin         //命令位高
#define DO_L DO_port->BRR  = DO_pin         //命令位低

//CS 片选
#define CS_port  GPIOB
#define CS_pin   GPIO_Pin_14    
#define CS_H CS_port->BSRR = CS_pin         //CS拉高
#define CS_L CS_port->BRR  = CS_pin         //CS拉低

//CLK 时钟
#define CLK_port GPIOB
#define CLK_pin  GPIO_Pin_15   
#define CLK_H CLK_port->BSRR = CLK_pin      //时钟高
#define CLK_L CLK_port->BRR  = CLK_pin      //时钟低

//These are our button constants
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2          9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_GREEN       13
#define PSB_RED         14
#define PSB_BLUE        15
#define PSB_PINK        16

#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15
#define PSB_SQUARE      16

//These are stick values
#define PSS_RX 5                //右摇杆X轴
#define PSS_RY 6                //右摇杆Y轴
#define PSS_LX 7                //左摇杆X轴
#define PSS_LY 8                //左摇杆Y轴

void PS2_Init(void);
void PS2_SetInit(void);
void PS2_Cmd(u8 CMD);
u8 PS2_RedLight(void);
void PS2_ReadData(void);
void PS2_ClearData(void);
u8 PS2_DataKey(void);
u8 PS2_AnologData(u8 button);
void PS2_Vibration(u8 motor1, u8 motor2);
void PS2_ShortPoll(void);
void PS2_EnterConfing(void);
void PS2_TurnOnAnalogMode(void);
void PS2_VibrationMode(void);
void PS2_ExitConfing(void);

void ps2_data_sample(void);

extern _PS2_DATA PS2;


#endif

