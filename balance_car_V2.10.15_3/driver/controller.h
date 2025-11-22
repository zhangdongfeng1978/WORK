#ifndef _controller_h_
#define _controller_h_

#include "stm32f10x.h"

#define AMPLITUDE 7000    
#define MODE_PS2            1
#define MODE_BLUETEETH      2
#define MODE_USER           3

#define TheCarMode          2

#define remoteMode          0       //蓝牙遥控
#define ultraFollow         1       //超声波跟随
#define infraredTrace       2       //红外寻迹
#define ultraAvoid          3       //超声波避障

#define UltraTurnDist       25      //超声波避障模式时的指定距离进行避障

//#define USE_BLUETOOTHopenOrclose    //该宏定义为是否使用蓝牙遥控去控制小车的开启或停止，若屏蔽则不使用蓝牙进行开启或停止

typedef struct 
{
    float ForwardOrBackward;
    float LeftOrRight;
    int LeftOrRightTurOut;
    float Speed;
    
    uint8_t ContorlMode;
    
}_REMOTE_Data;

typedef struct 
{
    int out;
}_OUT_Motor;


typedef struct
{
    int balance;
    int velocity;
    int turn;
    
    float bais[3];
    float exp[3];
    
    int pwm[3];
    int out[3];
    int motor[2];
    
}_OUT_CONTROL;

typedef struct
{
    float kp;
    float ki;
    float kd;
}_PID_CONTROL;

void angle_controller(void);
void gyro_controller(void);
void _controller_output(void);
void _controller_perform(void);
uint8_t shut_down(float angle,float Target);    

int ctr_bal(float angle,float gyro);
int ctr_vel(int encoder_left,int encoder_right);
void _ctr_out(void);

void dir_config(int *motor1,int *motor2);
int ctr_turn(int encoder_left,int encoder_right, float gyro_z);

int dirTest(uint8_t AIN1,uint8_t AIN2,uint8_t BIN1,uint8_t BIN2);


extern _OUT_Motor Motor1;
extern _OUT_CONTROL ctr;
extern _PID_CONTROL vel;
extern _PID_CONTROL bal;
extern _PID_CONTROL tur;

extern _PID_CONTROL ReadFlashVel;
extern _PID_CONTROL ReadFlashBal;
extern _PID_CONTROL ReadFlashTur;
extern _PID_CONTROL WriteFlashVel;
extern _PID_CONTROL WriteFlashBal;
extern _PID_CONTROL WriteFlashTur;

extern _REMOTE_Data remote;

extern int CarStepForwardOrBackward;
extern int CarStepLeftOrRight;

#endif

