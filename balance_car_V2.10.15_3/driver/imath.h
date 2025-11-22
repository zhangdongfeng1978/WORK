#ifndef _imath_h_
#define _imath_h_

#include "stm32f10x.h"
#include "mpu6050.h"

float invSqrt(float x);
float fast_atan2(float y, float x) ;
uint16_t to_limit(uint16_t thr_in,uint16_t thr_min,uint16_t thr_max);
float to_zero(float in_dat,float min_dat,float max_dat);
float f_abs(float f);
int int_abs(int f);
int i_limit(int *_in_val,int16_t  standard_val);
void  set_value(SI_F_XYZ *_in_data,float value);
void _set_val(SI_F_XYZ *_out_data,SI_F_XYZ *_in_data);
void mymemcpy(void *des,void *src,u32 n) ; 
void tdelay_us(unsigned int n);
void tdelay_ms(unsigned int n);
uint8_t  protectedData(uint8_t indat);
int toIntLimit(int thr_in,int thr_min,int thr_max);
void ForceIntLimit(int *_in_val,uint16_t thr_min,uint16_t thr_max);

#endif

