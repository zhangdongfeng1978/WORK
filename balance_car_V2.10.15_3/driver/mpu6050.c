/**********************************************************************
版权所有：源动力科技
淘    宝：https://1024tech.taobao.com/
文 件 名：mpu6050.c
版    本：V21.01.30
摘    要：
***********************************************************************/
#include "mpu6050.h"
#include "iic.h"
#include "systick.h"
#include "filter.h"
#include "flash.h"
#include <string.h>
#include "imath.h"
#include "led.h"

S16_XYZ  acc_raw = {0};                  //加速度计原始数据存储
S16_XYZ  gyro_raw = {0};                 //陀螺仪原始数据存储
SI_F_XYZ  gyro_raw_cal = {0};                 //陀螺仪用于校准的原始数据存储
SI_F_XYZ acc_raw_f = {0};
SI_F_XYZ gyro_raw_f = {0};
SI_F_XYZ acc_att_lpf = {0};
SI_F_XYZ gyro_lpf = {0};
SI_F_XYZ gyro_offset = {0,0,0} ;         //陀螺仪零偏数据存储
_Mpu6050_data Mpu = {0};

_GYRO_CAL CalGyro = {0};                 //陀螺仪校准相关变量存储
/*
 * 函数名：mpu6050_init
 * 描述  ：初始化MOU6050配置
 * 输入  ：    
 * 返回  ： 
 */
void mpu6050_init(void)
{
	IIC_Write_One_Byte(0xD0,PWR_MGMT_1, 0x80);		
	tdelay_ms(100);													
	IIC_Write_One_Byte(0xD0,PWR_MGMT_1, 0x00);              //唤醒mpu		
 
    /* when DLPF is disabled( DLPF_CFG=0 or 7),陀螺仪输出频率 = 8kHz; 
       when DLPFis enabled,陀螺仪输出频率 = 1KHz 
       fs(采样频率) = 陀螺仪输出频率 / (1 + SMPLRT_DIV)*/	
	IIC_Write_One_Byte(0xD0,SMPLRT_DIV, 0x00);		        //sample rate.  Fsample= 1Khz/(<this value>+1) = 1000Hz	
	IIC_Write_One_Byte(0xD0,MPU_CONFIG, 0x03);              //内部低通  acc:44hz	gyro:42hz
	IIC_Write_One_Byte(0xD0,GYRO_CONFIG, 0x18);			    // gyro scale  ：+-2000°/s
	IIC_Write_One_Byte(0xD0,ACCEL_CONFIG, 0x10);			// Accel scale ：+-8g (65536/16=4096 LSB/g)    			   			
}
/*
 * 函数名：get_data
 * 描述  ：两字节数据读取并合成一个16位数据
 * 输入  ：REG_Address寄存器地址    
 * 返回  ：合成的16位数据 
 */
static int get_data(unsigned char REG_Address)
{
	unsigned char H,L;
	H = IIC_Read_One_Byte(0xD0,REG_Address);
	L = IIC_Read_One_Byte(0xD0,REG_Address+1);
	return ((H<<8)+L);   
}
/*
 * 函数名：get_mpu_id
 * 描述  ：读取mpu6050的ID
 * 输入  ：     
 * 返回  ：芯片ID 
 */
uint8_t get_mpu_id(void)
{
    u8 mpu_id;
    mpu_id = IIC_Read_One_Byte(0xD0,WHO_AM_I);
    return mpu_id;
}
/*
 * 函数名：get_acc_raw
 * 描述  ：读取加速度计三轴原始数据
 * 输入  ：     
 * 返回  ：     
 */
void get_acc_raw(void)
{
    acc_raw.x = get_data(ACCEL_XOUT_H);
    acc_raw.y = get_data(ACCEL_YOUT_H);
    acc_raw.z = get_data(ACCEL_ZOUT_H); 

	acc_raw_f.x = (float)acc_raw.x;
	acc_raw_f.y = (float)acc_raw.y;
	acc_raw_f.z = (float)acc_raw.z;
}
/*
 * 变量名：gyro_30hz_parameter
 * 描述  ：巴特沃斯低通滤波器参数
 * 输入  ：     
 * 返回  ：     
 */
_Butterworth_parameter gyro_30hz_parameter =
{
    //200hz---30hz
    1,  -0.7477891782585,    0.272214937925,
    0.1311064399166,   0.2622128798333,   0.1311064399166 
}; 

_Butterworth_data   gyro_butter_data[3];
/*
 * 函数名：get_gyro_raw
 * 描述  ：读取陀螺仪三轴原始数据 & 进行校准 & 低通滤波
 * 输入  ：     
 * 返回  ：     
 */
void get_gyro_raw(void)
{
    gyro_raw.x = get_data(GYRO_XOUT_H) - gyro_offset.x;
    gyro_raw.y = get_data(GYRO_YOUT_H) - gyro_offset.y;
    gyro_raw.z = get_data(GYRO_ZOUT_H) - gyro_offset.z;        
    
    gyro_raw_f.x = (float)butterworth_lpf(((float)gyro_raw.x),&gyro_butter_data[0],&gyro_30hz_parameter);
    gyro_raw_f.y = (float)butterworth_lpf(((float)gyro_raw.y),&gyro_butter_data[1],&gyro_30hz_parameter);
    gyro_raw_f.z = (float)butterworth_lpf(((float)gyro_raw.z),&gyro_butter_data[2],&gyro_30hz_parameter);
    
    gyro_raw_cal.x = (float)get_data(GYRO_XOUT_H);
    gyro_raw_cal.y = (float)get_data(GYRO_YOUT_H);
    gyro_raw_cal.z = (float)get_data(GYRO_ZOUT_H);
}
/*
 * 函数名：get_iir_factor
 * 描述  ：求取IIR滤波器的滤波因子
 * 输入  ：out_factor滤波因子首地址，Time任务执行周期，Cut_Off滤波截止频率     
 * 返回  ：     
 */
void get_iir_factor(float *out_factor,float Time, float Cut_Off)
{
	*out_factor = Time /( Time + 1/(2.0f * PI * Cut_Off) );
}
//IIR低通滤波器(加速度)
void acc_iir_lpf(SI_F_XYZ *acc_in,SI_F_XYZ *acc_out,float lpf_factor)
{
	acc_out->x = acc_out->x + lpf_factor*(acc_in->x - acc_out->x); 
	acc_out->y = acc_out->y + lpf_factor*(acc_in->y - acc_out->y); 
	acc_out->z = acc_out->z + lpf_factor*(acc_in->z - acc_out->z); 
}
//加速度计滤波参数  
_Butterworth_parameter acc_5hz_parameter =
{
  //200hz---1hz
//  1,   -1.955578240315,   0.9565436765112,
//  0.000241359049042, 0.000482718098084, 0.000241359049042
  //200hz---2hz
//  1,   -1.911197067426,   0.9149758348014,
//  0.0009446918438402,  0.00188938368768,0.0009446918438402
    //200hz---5hz
    1,                  -1.778631777825,    0.8008026466657,
    0.005542717210281,   0.01108543442056,  0.005542717210281
    //200hz---10hz
//    1,   -1.561018075801,   0.6413515380576,
//    0.02008336556421,  0.04016673112842,  0.02008336556421
    //200hz---15hz
//    1,   -1.348967745253,   0.5139818942197,
//    0.04125353724172,  0.08250707448344,  0.04125353724172
    //200hz---20hz
//    1,    -1.14298050254,   0.4128015980962,
//    0.06745527388907,   0.1349105477781,  0.06745527388907
    //200hz---30hz
//    1,  -0.7477891782585,    0.272214937925,
//    0.1311064399166,   0.2622128798333,   0.1311064399166 
}; 

_Butterworth_data   acc_butter_data[3];
/*
 * 函数名：acc_butterworth_lpf
 * 描述  ：巴特沃斯加速度低通滤波
 * 输入  ：acc_in滤波前的加速度首地址，acc_out滤波后的输出加速度数据首地址     
 * 返回  ：     
 */
void acc_butterworth_lpf(SI_F_XYZ *acc_in,SI_F_XYZ *acc_out)
{
    acc_out->x = butterworth_lpf(acc_in->x,&acc_butter_data[0],&acc_5hz_parameter);
    acc_out->y = butterworth_lpf(acc_in->y,&acc_butter_data[1],&acc_5hz_parameter);
    acc_out->z = butterworth_lpf(acc_in->z,&acc_butter_data[2],&acc_5hz_parameter);    
}
/*
 * 函数名：get_acc_g
 * 描述  ：原始加速度转为重力加速度g为单位数据
 * 输入  ：acc_in原始的加速度首地址，acc_out以g为单位的加速度数据首地址     
 * 返回  ：     
 */
void get_acc_g(SI_F_XYZ *acc_in,SI_F_XYZ *acc_out)
{
	acc_out->x = (float)(acc_in->x * acc_raw_to_g);
	acc_out->y = (float)(acc_in->y * acc_raw_to_g);
	acc_out->z = (float)(acc_in->z * acc_raw_to_g);
}
/*
 * 函数名：get_rad_s
 * 描述  ：原始陀螺仪数据转为弧度/秒为单位的数据
 * 输入  ：gyro_in原始的陀螺仪数据首地址，gyro_out以rad/s为单位的陀螺仪数据首地址     
 * 返回  ：     
 */
void get_rad_s(SI_F_XYZ *gyro_in,SI_F_XYZ *gyro_out)
{
	gyro_out->x = (float)(gyro_in->x * gyro_raw_to_radian_s);
	gyro_out->y = (float)(gyro_in->y * gyro_raw_to_radian_s);
	gyro_out->z = (float)(gyro_in->z * gyro_raw_to_radian_s);
}
/*
 * 函数名：get_deg_s
 * 描述  ：原始陀螺仪数据转为度/秒为单位的数据
 * 输入  ：gyro_in原始的陀螺仪数据首地址，gyro_deg_out以deg/s为单位的陀螺仪数据首地址     
 * 返回  ：     
 */
void get_deg_s(SI_F_XYZ *gyro_in,SI_F_XYZ *gyro_deg_out)
{
	gyro_deg_out->x = (float)(gyro_in->x * gyro_raw_to_deg_s);
	gyro_deg_out->y = (float)(gyro_in->y * gyro_raw_to_deg_s);
	gyro_deg_out->z = (float)(gyro_in->z * gyro_raw_to_deg_s);    
}

/*
 * 函数名：gyro_cal
 * 描述  ：陀螺仪零偏数据校准
 * 输入  ：gyro_in原始的静止时陀螺仪数据首地址    
 * 返回  ：     
 */
void gyro_cal(SI_F_XYZ *gyro_in)
{
    if(CalGyro.flag==1 && 1)                                    
    {
        if(CalGyro.i < GyroCalSumValue)		                                        //原始静止数据500次累加求取平均	
        {                       
            CalGyro.offset.x += gyro_in->x; 
            CalGyro.offset.y += gyro_in->y;
            CalGyro.offset.z += gyro_in->z;
            CalGyro.i++;
        }
        else
        {
            CalGyro.i = 0;
            CalGyro.OffsetFlashWrite.x = CalGyro.offset.x / GyroCalSumValue;        //得到三轴的零偏 
            CalGyro.OffsetFlashWrite.y = CalGyro.offset.y / GyroCalSumValue;        //得到三轴的零偏
            CalGyro.OffsetFlashWrite.z = CalGyro.offset.z / GyroCalSumValue;        //得到三轴的零偏

            /* 将陀螺仪零偏写入flash */
            FlashWriteNineFloat(SENSOR_CAL_ADDRESS, 0,0,0,
                                                    0,0,0,
                                                    CalGyro.OffsetFlashWrite.x,CalGyro.OffsetFlashWrite.y,CalGyro.OffsetFlashWrite.z);  
            /* 校准完数据之后立马读取出来进行使用 */
            flash_flag.sensor = FlashReadNineFloat(SENSOR_CAL_ADDRESS,  &CalGyro.None.x,
                                                                        &CalGyro.None.y,
                                                                        &CalGyro.None.z, 
                                                                        &CalGyro.None.x,    
                                                                        &CalGyro.None.y,
                                                                        &CalGyro.None.z,

                                                                        &CalGyro.OffsetFlashRead.x,
                                                                        &CalGyro.OffsetFlashRead.y,
                                                                        &CalGyro.OffsetFlashRead.z);
            /* 判断是否正确读出 */
            if(flash_flag.sensor!=0x01ff && flash_flag.sensor!=0x01C0)
            {
                _set_val(&gyro_offset,&CalGyro.OffsetFlashRead);                    //陀螺仪零偏设置
                CalGyro.success = 1;                                                //校准成功
                _led.sta = 0;
            }
            else 
            {
                CalGyro.success = 2;                                                //校准失败
            }
            CalGyro.offset.x = 0;
            CalGyro.offset.y = 0;
            CalGyro.offset.z = 0;
            CalGyro.flag = 0;                                                       //校准标志位清除
        }
    }
}

/*
 * 函数名：ReadCalData
 * 描述  ：陀螺仪零偏校准数据进行读取
 * 输入  ：      
 * 返回  ：err     
 */
int ReadCalData(void)
{
    ErrorStatus err;
    flash_flag.sensor = FlashReadNineFloat(SENSOR_CAL_ADDRESS,  &CalGyro.None.x,
                                                                &CalGyro.None.y,
                                                                &CalGyro.None.z, 
                                                                &CalGyro.None.x,    
                                                                &CalGyro.None.y,
                                                                &CalGyro.None.z,

                                                                &CalGyro.OffsetFlashRead.x,
                                                                &CalGyro.OffsetFlashRead.y,
                                                                &CalGyro.OffsetFlashRead.z);
    /* 判断是否正确读出 */
    if(flash_flag.sensor!=0x01ff && flash_flag.sensor!=0x01C0)
    {
        _set_val(&gyro_offset, &CalGyro.OffsetFlashRead);                           //陀螺仪零偏设置
        err = SUCCESS ;
    }
    else
    {
        err = ERROR;
    }
        
    return err;
}
