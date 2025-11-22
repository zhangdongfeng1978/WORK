/**********************************************************************
版权所有：源动力科技
淘    宝：https://1024tech.taobao.com/
文 件 名：imath.c
版    本：V21.01.30
摘    要：
***********************************************************************/
#include "imath.h"

// Fast inverse square-root
float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
/*
 * 函数名：f_abs
 * 描述  ：浮点型数据绝对值
 * 输入  ：f浮点数据 
 * 返回  ：绝对值 
 */
float f_abs(float f)
{
	if (f >= 0.0f)
		return f;
	return -f;
}
/*
 * 函数名：int_abs
 * 描述  ：整型数据绝对值
 * 输入  ：f整型数据 
 * 返回  ：绝对值 
 */
int int_abs(int f)
{
	if (f >= 0)
		return f;
	return -f;
}
/*
 * 函数名：i_limit
 * 描述  ：整型数据正负范围限制
 * 输入  ：_in_val输入的数据地址，standard_val边界数据
 * 返回  ：在范围之内的整型数据 
 */
int i_limit(int *_in_val,int16_t  standard_val)
{
    if(*_in_val > standard_val)
        *_in_val = standard_val;
    if(*_in_val < -standard_val)
        *_in_val = -standard_val;
    return *_in_val;
}
/*
 * 函数名：ForceIntLimit
 * 描述  ：数据范围限制
 * 输入  ：_in_val输入数据首地址， thr_min最小值，thr_max最大值
 * 返回  ：    
 */
void ForceIntLimit(int *_in_val,uint16_t thr_min,uint16_t thr_max)
{
	if(*_in_val<thr_min)	*_in_val = thr_min;
	if(*_in_val>thr_max)	*_in_val = thr_max;
}
/*
 * 函数名：to_limit
 * 描述  ：数据范围限制
 * 输入  ：thr_in输入数据， thr_min最小值，thr_max最大值
 * 返回  ：    
 */
uint16_t to_limit(uint16_t thr_in,uint16_t thr_min,uint16_t thr_max)
{
	if(thr_in<thr_min)	thr_in = thr_min;
	if(thr_in>thr_max)	thr_in = thr_max;
	return thr_in;
}


int toIntLimit(int thr_in,int thr_min,int thr_max)
{
	if(thr_in<=thr_min)	thr_in = thr_min;
	if(thr_in>=thr_max)	thr_in = thr_max;
	return thr_in;
}
/*
 * 函数名：to_zero
 * 描述  ：在一定范围内将数据置为0
 * 输入  ：in_dat输入数据， min_dat最小值，max_dat最大值
 * 返回  ：正常数据    
 */
float to_zero(float in_dat,float min_dat,float max_dat)
{
    if(in_dat > min_dat && in_dat < max_dat)  
        in_dat = 0;
    return in_dat;
}
/*
 * 函数名：set_value
 * 描述  ：给数据赋值
 * 输入  ：_in_dat输入数据首地址， value所需要赋的值
 * 返回  ：     
 */
void  set_value(SI_F_XYZ *_in_data,float value)
{
    _in_data->x = value;
    _in_data->y = value;
    _in_data->z = value;
}

void _set_val(SI_F_XYZ *_out_data,SI_F_XYZ *_in_data)
{
    _out_data->x = _in_data->x;
    _out_data->y = _in_data->y;
    _out_data->z = _in_data->z;
}

/*
 * 函数名：mymemcpy
 * 描述  ：内存数据copy
 * 输入  ：des目的首地址， src源地址，n长度
 * 返回  ：     
 */
void mymemcpy(void *des,void *src,u32 n)  
{  
    u8 *xdes=des;
	u8 *xsrc=src; 
    while(n--)
        *xdes++=*xsrc++;  
}

/*
	软延时函数
*/
void tdelay_us(unsigned int n)
{
	unsigned char j;
	while(n--)
        for(j=0;j<10;j++);
}
void tdelay_ms(unsigned int n)
{
	while(n--)
        tdelay_us(1000);
}

//防止数据跳变
uint8_t protectedData(uint8_t indat)
{
    static uint8_t p_buf[3];
    static uint8_t num = 0;
    uint8_t rdata;
    
    p_buf[num++] = indat;
    if(p_buf[1] != p_buf[0] && p_buf[1] != p_buf[2])
    {
        rdata = 0;
    }
    else 
        rdata = indat;
    
    if(num==3) 
        num = 0;
    
    return rdata;
}

