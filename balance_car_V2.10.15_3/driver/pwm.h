#ifndef _pwm_h_
#define _pwm_h_

#include "stm32f10x.h"

#define ENCODER_A   1
#define ENCODER_B   2
#define MAX_VALUE     50000

#define AIN1_rcc RCC_APB2Periph_GPIOA
#define AIN2_rcc RCC_APB2Periph_GPIOA
#define BIN1_rcc RCC_APB2Periph_GPIOB
#define BIN2_rcc RCC_APB2Periph_GPIOB
#define STBY_rcc RCC_APB2Periph_GPIOA

#define AIN1_port GPIOA
#define AIN2_port GPIOA
#define BIN1_port GPIOB
#define BIN2_port GPIOB
#define STBY_port GPIOA

#define AIN1_pin GPIO_Pin_4
#define AIN2_pin GPIO_Pin_5
#define BIN1_pin GPIO_Pin_3
#define BIN2_pin GPIO_Pin_4
#define STBY_pin GPIO_Pin_12

#define AIN1_HIGH   GPIO_SetBits(AIN1_port,AIN1_pin)
#define AIN1_LOW    GPIO_ResetBits(AIN1_port,AIN1_pin)
#define AIN2_HIGH   GPIO_SetBits(AIN2_port,AIN2_pin)
#define AIN2_LOW    GPIO_ResetBits(AIN2_port,AIN2_pin)

#define BIN1_HIGH   GPIO_SetBits(BIN1_port,BIN1_pin)
#define BIN1_LOW    GPIO_ResetBits(BIN1_port,BIN1_pin)
#define BIN2_HIGH   GPIO_SetBits(BIN2_port,BIN2_pin)
#define BIN2_LOW    GPIO_ResetBits(BIN2_port,BIN2_pin)

#define STBY_LOW    GPIO_ResetBits(STBY_port,STBY_pin)
#define STBY_HIGH   GPIO_SetBits(STBY_port,STBY_pin)

void pwm_init(void);
void motor_pwm_out(uint16_t pwm1,uint16_t pwm2 );
void Encoder_A_init(void);
void Encoder_B_init(void);
int  read_encoder(uint8_t encoder_num);
void encoder_manage(int *data);
void driver_pin_init(void);


extern int encoder_val_a;
extern int encoder_val_b;

#endif

