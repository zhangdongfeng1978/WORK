#ifndef _led_h_
#define _led_h_

#include "stm32f10x.h"

#define led1_rcc RCC_APB2Periph_GPIOC
#define led2_rcc RCC_APB2Periph_GPIOC

#define led1_port GPIOC
#define led2_port GPIOC

#define led1_pin GPIO_Pin_13
#define led2_pin GPIO_Pin_13

typedef struct
{
    uint8_t sta;
}_LED_STATES;

typedef struct
{
    uint8_t sta;
    uint8_t last_sta;
}_CAR_STATES;

void led_init(void);
void led_off_all(void);
void led_on(uint8_t led_num);
void led_off(uint8_t led_num);
void led_toggle(uint8_t led_num);
void led_blink(void );
void _no_operation(void);

extern _LED_STATES _led;

#endif

