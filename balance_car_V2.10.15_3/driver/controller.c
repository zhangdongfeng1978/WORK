/**********************************************************************
版权所有：源动力科技
淘    宝：https://1024tech.taobao.com/
文 件 名：controller.c
版    本：V21.01.30
摘    要：
***********************************************************************/
#include "controller.h"
#include "pid.h"
#include "imu.h"
#include "pwm.h"
#include "mpu6050.h"
#include "key.h"
#include "imath.h"
#include "ps2.h"
#include "bluetooth.h"
#include "ultrasonic.h"
#include "Infrared.h"
_OUT_Motor Motor1 = {0};
_OUT_Motor Motor2 = {0};

/* PID参数保存 */
#if 1 /* product motor */

_PID_CONTROL vel = {135.0f, 0.5f, 0.0f};       //{130.0f, 0.65f, 0.0f};
_PID_CONTROL bal = {200.0f, 0.0f , 15.0f};      //{150.0f, 0.0f , 20.0f};
_PID_CONTROL tur = {5.0f,  0.0f,  10.0f};      //{10.0f,  0.0f,  15.5f};
#else  /*self-maker-motor*/

_PID_CONTROL vel = {115.0f, 0.65f, 15.0f};       //{130.0f, 0.65f, 0.0f};
_PID_CONTROL bal = {230.0f, 0.0f , 15.0f};      //{150.0f, 0.0f , 20.0f};
_PID_CONTROL tur = {10.0f,  0.0f,  15.5f};      //{10.0f,  0.0f,  15.5f};
#endif
_PID_CONTROL ReadFlashVel = {0};
_PID_CONTROL ReadFlashBal = {0};
_PID_CONTROL ReadFlashTur = {0};

_PID_CONTROL WriteFlashVel = {0};
_PID_CONTROL WriteFlashBal = {0};
_PID_CONTROL WriteFlashTur = {0};

_OUT_CONTROL ctr = {0};
_REMOTE_Data remote = {0};

/*
 * 函数名：CarSpeedCtrlForwardOrBackward
 * 描述  ：遥控时前进或者后退的速度更改
 * 输入  ：ChangeStep增加或者减少的阶梯步数(改变前后运行速度的快慢程度)
 * 返回  ：处理及限幅之后的前后方向速度更改阶梯步数  
 */
int CarSpeedCtrlForwardOrBackward(int ChangeStep)
{
    static int carUpOrDownOut = 40;                                 //开机上电默认前进后退速度
    if(PS2.KEY==PSB_GREEN)
    {
        carUpOrDownOut += ChangeStep;
    }
    if(PS2.KEY==PSB_BLUE)
    {
        carUpOrDownOut -= ChangeStep;
    }
    
    ForceIntLimit(&carUpOrDownOut,10,70);
    return carUpOrDownOut;                                          //限幅
}

/*
 * 函数名：CarSpeedCtrlLeftOrRight
 * 描述  ：更改遥控时左右转向/旋转的速度快慢
 * 输入  ：ChangeStep增加或者减少的阶梯步数(改变前后运行速度的快慢程度)
 * 返回  ：处理及限幅之后的左右/旋转方向速度更改阶梯步数  
 */
int CarSpeedCtrlLeftOrRight(int ChangeStep)
{
    static int carLeftOrRightOut = 2500;                            //开机上电转向的初始控制量输出
    if(PS2.KEY==PSB_RED)
    {
        carLeftOrRightOut += ChangeStep;
    }
    if(PS2.KEY==PSB_PINK)
    {
        carLeftOrRightOut -= ChangeStep;
    }
    
    ForceIntLimit(&carLeftOrRightOut,1000,4000);
    return carLeftOrRightOut;                                       //限幅
}


/*
 * 函数名：remoteCarForwardOrBackward
 * 描述  ：遥控控制前进和后退
 * 输入  ：Speed控制前进和后退的速度，RemoteModeSelect遥控模式选择，UltrasonicMode功能模式选择
 * 返回  ：当前速度运行 
 */
float remoteCarForwardOrBackward(float Speed , uint8_t RemoteModeSelect , uint8_t UltrasonicMode)
{
    float TargetVel;
  
    if( UltrasonicMode == ultraFollow)                                  //超声波跟随模式
    {
        if(( getDistance <= 10) && ( getDistance > 0 ))                 //距离小于10CM小车后退
        {
            TargetVel = -20;                                            //后退速度
        }
        else if(( getDistance > 20) && ( getDistance <= 30 ))           //距离大于25且小于30CM小车前进
        {
            TargetVel = 20;                                             //前进速度
        }
        else
        {
            TargetVel = 0;
        }
    }
    else if( UltrasonicMode == ultraAvoid)                              //超声波避障模式
    {
        if(( getDistance <= UltraTurnDist ) && ( getDistance > 0 ))    //距离小于指定距离表示遇到障碍则停止前进
        {
            TargetVel = 0;                                              //停止前进
        }
        else                                                            //无障碍则前进      
        {
            TargetVel = 10;                                             //前进速度默认最低
        }
    }   
    else                                                                //遥控模式
    {
        if( RemoteModeSelect == MODE_PS2 )                                                  
        {
        }
        else if( RemoteModeSelect == MODE_BLUETEETH )
        {
            if(BluetoothParseMsg.Yrocker == 0)                          //前进      
                TargetVel = Speed;                                      //速度期望
            else if(BluetoothParseMsg.Yrocker == 5)                     //后退      
                TargetVel = -Speed;                                     //速度期望
            else
                TargetVel = 0;              
        }
        else if( RemoteModeSelect == MODE_USER )
        {
        }        
    }

    return TargetVel;
}

/*
 * 函数名：shut_down
 * 描述  ：关闭电机检测
 * 输入  ：angle小车当前角度，Target达到设置的目标角度关闭电机停止
 * 返回  ：关闭输出标志位 
 */
uint8_t shut_down(float angle,float Target)
{
    uint8_t shut_flag = 0;
    if(f_abs(angle)>Target)                                             
    {
        shut_flag = 1;                                                  //关闭输出标志置位                 
    }                                                            
    return shut_flag;                                            
}       

static uint8_t theCarStopFlag = 0;
/*
 * 函数名：PickUpTheCar
 * 描述  ：小车被拿起
 * 输入  ：angle当前角度，leftEncoder左轮编码器值，rightEncoder右轮编码器值
 * 返回  ：是否成功拿起
 */
static uint8_t PickUpisOK = 0 ;
uint8_t PickUpTheCar(float carAngle , int leftEncoder , int rightEncoder)
{
    static uint16_t pickUpTime = 0 ;
    /* 小车被拿起后双轮高速转动  */
    if( leftEncoder >=40  || leftEncoder <=-40  ||  rightEncoder >=40  || rightEncoder <=-40  )
    {
        pickUpTime++;
        if(pickUpTime>300)                                   //转动时长：300 * 5ms 
        {
            pickUpTime = 0;
            PickUpisOK = 1;
        }
    }
    
    return PickUpisOK ; 
}
/*
 * 函数名：PutDownTheCar
 * 描述  ：小车被放下
 * 输入  ：angle当前角度，leftEncoder左轮编码器值，rightEncoder右轮编码器值
 * 返回  ：是否成功放下
 */
uint8_t PutDownTheCar(float carAngle , int leftEncoder , int rightEncoder)
{
    static uint16_t isOK = 0 ; 
    static uint16_t PutDownTime = 0;
    
    if(theCarStopFlag==0)                                   //放置误检
        return 0;
    /* 小车两轮静止，并且放置的角度在平衡位置10度附近 */
    if( int_abs(leftEncoder) ==0 &&  int_abs(rightEncoder) == 0 && (f_abs(carAngle) <= 5.0f) )
    {
        PutDownTime++;
        if(PutDownTime>400)                                 //放下持续2S后平衡
        {
            PutDownTime = 0;
            isOK = 1;
        }
    }
    else 
    {
        PutDownTime = 0;
        isOK = 0;
    }
    
    return isOK;
}
/*   
ctr.bais[0]：角度控制器偏差存储 
ctr.bais[1]：速度控制器偏差累计存储 
ctr.bais[2]：转向控制器偏差存储 

ctr.exp[0]：角度控制器期望角度
ctr.exp[1]：速度控制器期望速度
ctr.exp[2]：转向控制器期望角度
*/

/*
 * 函数名：ctr_bal
 * 描述  ：角度PD控制器
 * 输入  ：angle当前角度，gyro当前角速度
 * 返回  ：PID控制器输出
 */
int ctr_bal(float angle,float gyro)
{       
    ctr.exp[0] = 0;                                                     //期望角度    
    ctr.bais[0] = (float)(angle - ctr.exp[0]);                          //角度偏差
    ctr.balance = bal.kp * ctr.bais[0] + gyro * bal.kd;                 //角度平衡控制
    
    return ctr.balance;
}
int CarStepForwardOrBackward;
/*
 * 函数名：ctr_vel
 * 描述  ：速度PI控制器
 * 输入  ：encoder_left编码器值A，encoder_right编码器值B
 * 返回  ：PID控制器输出
 */
int ctr_vel(int encoder_left,int encoder_right)
{  
    static float encoder_cur,encoder_last;

    CarStepForwardOrBackward = CarSpeedCtrlForwardOrBackward(1);        //遥控器控制的前后方向速度    
    remote.ForwardOrBackward = remoteCarForwardOrBackward( (float)CarStepForwardOrBackward , MODE_BLUETEETH , UltraSuccess(BluetoothParseMsg.ModeRocker) ); //遥控器控制前后方向运动
    
    encoder_last = ((encoder_left) + (encoder_right)) - 0;              //速度误差
    encoder_cur = encoder_cur * 0.8 + encoder_last * 0.2;               //对偏差进行低通滤波
    ctr.bais[1] += encoder_cur;                                         //偏差累加和   

    ctr.bais[1] = ctr.bais[1] - remote.ForwardOrBackward;               //遥控控制前后方向
    
    if(ctr.bais[1] > 10000)     ctr.bais[1] = 10000;                    //限幅
    if(ctr.bais[1] <-10000)	    ctr.bais[1] =-10000; 
    ctr.velocity = encoder_cur * vel.kp + ctr.bais[1] * vel.ki;         //速度控制     

    return ctr.velocity;
}

/*
 * 函数名：remoteCarLeftOrRight
 * 描述  ：遥控控制左转和右转
 * 输入  ：Speed转动速度，RemoteModeSelect遥控模式选择，UltrasonicMode功能模式选择
 * 返回  ：经过处理及限幅后的转向速度
 */
int remoteCarLeftOrRight(int16_t Speed,uint8_t RemoteModeSelect , uint8_t UltrasonicMode)
{
    static uint8_t turnFlag = 0;
    static uint8_t turnTime = 0;
    
    if( UltrasonicMode == ultraAvoid )                                              //超声波避障模式
    {
        if((( getDistance <= UltraTurnDist ) && ( getDistance > 0 )) )             //距离小于指定距离时表示遇到障碍，便进行转向标志置位
        {
            turnFlag = 1;
        }
        if( turnFlag == 1 )                                                         //进行转向准备
        {
            tur.kd = 0;                                                             
            remote.LeftOrRight += 15;                                               //固定向左转向
            turnTime++;
            if(turnTime >= 50)                                                      //转向计数达到50时便停止转向
            {
                turnTime = 0;     
                turnFlag = 0;
            }
        }
        else
        {
            remote.LeftOrRight = 0;                                                 //无障碍则不进行转向
        }
    }      
    else if( UltrasonicMode == ultraFollow )                                        //超声波跟随模式
    {
        if(BluetoothParseMsg.Xrocker == 0 || BluetoothParseMsg.Xrocker == 5)        //有转向控制  
        { 
            tur.kd = 0;   
        }
        else                                                                        //无转向控制
        {
            tur.kd = 15.5f;
        }      
        if(BluetoothParseMsg.Xrocker == 0)                                          //左转向                      
            remote.LeftOrRight += 15;                       
        else if(BluetoothParseMsg.Xrocker ==5)                                      //右转向
            remote.LeftOrRight -= 15;
        else 
            remote.LeftOrRight = 0;  
    }
    else                                                                            //遥控模式
    {
        if(RemoteModeSelect==MODE_PS2)                                              
        {
        }
        else if(RemoteModeSelect==MODE_BLUETEETH)                                   //蓝牙遥控模式
        {
            if(BluetoothParseMsg.Xrocker == 0 || BluetoothParseMsg.Xrocker == 5)    //有转向控制  
            { 
                tur.kd = 0;   
            }
            else                                                                    //无转向控制
            {
                tur.kd = 15.5f;
            }      
            if(BluetoothParseMsg.Xrocker == 0)                                      //左转向                                                        
                remote.LeftOrRight += 15;
            else if(BluetoothParseMsg.Xrocker ==5)                                  //右转向
                remote.LeftOrRight -= 15;
            else 
                remote.LeftOrRight = 0;                
        }
        else if(RemoteModeSelect==MODE_USER)
        {
        }          
    }

    remote.LeftOrRightTurOut = (int)(-remote.LeftOrRight * tur.kp);                 //向左向右控制输出
    i_limit(&remote.LeftOrRightTurOut,Speed);                                       //遥控左右转旋转输出速度范围限制，不能太快（最大2500）   
    
    return remote.LeftOrRightTurOut;
}

//转向控制器 / 
int CarStepLeftOrRight;
/*
 * 函数名：ctr_turn
 * 描述  ：转向控制器
 * 输入  ：encoder_left编码器A的值，encoder_right编码器B的值，gyro_z转向的Z轴角速度
 * 返回  ：控制器输出
 */
int ctr_turn(int encoder_left,int encoder_right, float gyro_z) 
{
    static int remoteTurOut;
 
    CarStepLeftOrRight = CarSpeedCtrlLeftOrRight(1);                                //转向控制速度加减
    remoteTurOut = remoteCarLeftOrRight( CarStepLeftOrRight , MODE_BLUETEETH , UltraSuccess(BluetoothParseMsg.ModeRocker) );    //左转/右转控制输出
    ctr.turn = remoteTurOut + gyro_z * tur.kd;
    
    return ctr.turn;
}

/*
 * 函数名：_ctr_out
 * 描述  ：控制输出
 * 输入  ：    
 * 返回  ：    
 */
void _ctr_out(void)
{   

    if(PickUpTheCar(att.rol,encoder_val_a,-encoder_val_b) == 1)                //小车被拿起检测
    {
        theCarStopFlag = 1;
    }
    if(PutDownTheCar(att.rol,encoder_val_a,-encoder_val_b))                    //小车被放下检测
    {
        theCarStopFlag = 0;
    }
  
#ifdef USE_BLUETOOTHopenOrclose        
    /* 检测是否倒下和是否人为遥控控制小车停止，倒下或者停止后，积分项清零(不清除会干扰下次平衡)，并且电机停转 */
    if( shut_down(att.rol,70.0f) || (BluetoothParseMsg.OpenCloseStatus == 0) || theCarStopFlag == 1)                                                 
    {      
#else
    /* 检测是否倒下小车停止，倒下或者停止后，积分项清零(不清除会干扰下次平衡)，并且电机停转 */
    if( shut_down(att.rol,70.0f) || theCarStopFlag == 1)                                                 
    {      
#endif        
        PickUpisOK = 0;
        for(uint8_t i = 0 ; i < 3 ; i++)                                       //清除所有偏差
        {
            ctr.out[i] = 0;
            ctr.bais[i] = 0;
        }
        ctr.motor[0] = 0;
        ctr.motor[1] = 0;
    }                                                                          
    else 
    {
        ctr.motor[0] = ctr.out[0];
        ctr.motor[1] = ctr.out[1];
    }
    if( BluetoothParseMsg.OpenCloseStatus == 1 || theCarStopFlag == 0)          //蓝牙遥控开启小车动力输出或者不使用蓝牙开启关闭
        motor_pwm_out(ctr.motor[0],ctr.motor[1]);                               //正常输出
    else
        motor_pwm_out(0,0);                                                     //关闭电机
}

/*
 * 函数名：dir_config
 * 描述  ：电机转向控制
 * 输入  ：motor1电机1的pwm值首地址，motor2电机2的pwm值得首地址    
 * 返回  ：     
 */
void dir_config(int *motor1,int *motor2)
{
    if(*motor1<0) AIN1_HIGH, AIN2_LOW;
    else          AIN1_LOW, AIN2_HIGH;
    *motor1 = int_abs(*motor1);
    
    if(*motor2<0) BIN1_HIGH, BIN2_LOW;
    else          BIN1_LOW, BIN2_HIGH;
    *motor2 = int_abs(*motor2);	
}
/*
 * 函数名：dirTest
 * 描述  ：测试使用
 * 输入  ：        
 * 返回  ：     
 */
int dirTest(uint8_t AIN1,uint8_t AIN2,uint8_t BIN1,uint8_t BIN2)
{
    if(AIN2==0 && AIN1==1) AIN1_HIGH, AIN2_LOW;
    else if(AIN2==1 && AIN1==0)         AIN1_LOW, AIN2_HIGH;
    else if(BIN2==0 && BIN1==1) BIN1_HIGH, BIN2_LOW;
    else if(BIN2==1 && BIN1==0)         BIN1_LOW, BIN2_HIGH;
    else 
    {
        AIN1_LOW,AIN2_LOW,BIN1_LOW, BIN2_LOW;
    }
    return 0;
}
