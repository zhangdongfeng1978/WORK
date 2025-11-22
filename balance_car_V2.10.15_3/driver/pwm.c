/**********************************************************************
版权所有：源动力科技
淘    宝：https://1024tech.taobao.com/
文 件 名：pwm.c
版    本：V21.01.30
摘    要：
***********************************************************************/
#include "pwm.h"
#include "imath.h"


/*
 * 函数名：pwm_init
 * 描述  ：PWM配置初始化
 * 输入  ：    
 * 返回  ：     
 */
void pwm_init(void)
{
	GPIO_InitTypeDef GPIO_initStructure;
	TIM_TimeBaseInitTypeDef TIM_timeBaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);	
    
	GPIO_initStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_initStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_initStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_initStructure);

	//定时器周期 t =( 72 / 7200 ) * 10^6 =  10khz    
	
	//配置时基
	TIM_timeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;         //不分频
	TIM_timeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;		//向上计数
	TIM_timeBaseInitStructure.TIM_Period = 7200-1;						//设置ARR值
	TIM_timeBaseInitStructure.TIM_Prescaler = 0;						//时钟预分频值
	TIM_TimeBaseInit(TIM2,&TIM_timeBaseInitStructure);
	
	//配置OC输出通道
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;					//采用PWM模式1输出波形
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;			//设置CH通道的有效电平
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;		//设置CH通道的空闲状态的电平
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		//使能CH通道
	TIM_OCInitStructure.TIM_Pulse = 0;									//设置TIM1的CCR值
	
    TIM_OC1Init(TIM2,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2,TIM_OCPreload_Enable);                  
	
	TIM_OC2Init(TIM2,&TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM2,TIM_OCPreload_Enable);                                   

	//使能TIM的ARR和CRR，以及使能TIM定时器,开启pwm输出
	TIM_ARRPreloadConfig(TIM2,ENABLE);								    //预加载使能	
         
	TIM_Cmd(TIM2,ENABLE);
	
	//开始启动定时器输出pwm,这句是高级定时器才有的，输出pwm必须打开
//	TIM_CtrlPWMOutputs(TIM1,ENABLE);  

    TIM2->CCR1 = 0;
    TIM2->CCR2 = 0;
}
/*
 * 函数名：motor_pwm_out
 * 描述  ：控制两路PWM输出
 * 输入  ：两路的pwm    
 * 返回  ：     
 */
void motor_pwm_out(uint16_t pwm1,uint16_t pwm2)
{
	TIM2->CCR1 = to_limit(pwm1,0,7200);
	TIM2->CCR2 = to_limit(pwm2,0,7200);
}
/*
 * 函数名：driver_pin_init
 * 描述  ：TB6612电机驱动IO初始化配置
 * 输入  ：    
 * 返回  ：     
 */
void driver_pin_init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);                    //JTAG失能 PB3/4 用于普通IO口使用
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = AIN1_pin;
    GPIO_Init(AIN1_port, &GPIO_InitStructure);  

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = AIN2_pin;
    GPIO_Init(AIN2_port, &GPIO_InitStructure);  
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = BIN1_pin;
    GPIO_Init(BIN1_port, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = BIN2_pin;
    GPIO_Init(BIN2_port, &GPIO_InitStructure);    
    
    //STBY pin
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = STBY_pin;
    GPIO_Init(STBY_port, &GPIO_InitStructure);  
    
    //使能
    STBY_HIGH;  
}

/*
 * 函数名：Encoder_A_init
 * 描述  ：编码器A端口配置初始化
 * 输入  ：    
 * 返回  ：     
 */
void Encoder_A_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;      

    //PA6 ch1, PA7 ch2 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;         
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);                           

    TIM_DeInit(TIM3);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = MAX_VALUE;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);              

    //编码器接口配置：采用模式3：双边沿触发，并且在双边沿都进行计数
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 6;               //硬件捕获滤波
    TIM_ICInit(TIM3, &TIM_ICInitStructure);

    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    //Reset counter
    TIM3->CNT = 0;

    TIM_Cmd(TIM3, ENABLE);
}
/*
 * 函数名：Encoder_B_init
 * 描述  ：编码器B端口配置初始化
 * 输入  ：    
 * 返回  ：     
 */
void Encoder_B_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;      

    //PB6 ch1, PB7 ch2 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;         
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);                           

    TIM_DeInit(TIM4);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = MAX_VALUE;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);              

    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge ,TIM_ICPolarity_BothEdge);
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 6;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);

    TIM_ClearFlag(TIM4, TIM_FLAG_Update);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    //Reset counter
    TIM4->CNT = 0;

    TIM_Cmd(TIM4, ENABLE);
}

int encoder_val_a;
int encoder_val_b;
/*
 * 函数名：read_encoder
 * 描述  ：编码器数据读取
 * 输入  ：encoder_num：要读取的编码器号    
 * 返回  ：编码值     
 */
int read_encoder(uint8_t encoder_num)
{
    int encoder_val;
    switch(encoder_num)
    {
        case ENCODER_A: encoder_val = (int)TIM3->CNT; TIM3->CNT = 0;break;
        case ENCODER_B: encoder_val = (int)TIM4->CNT; TIM4->CNT = 0;break;
        default: encoder_val = 0; break;
    }
    return encoder_val;
}
/*
 * 函数名：encoder_manage
 * 描述  ：编码器数据处理为 ：-A ~ +A 范围
 * 输入  ：data：编码器值首地址    
 * 返回  ：  
 */
void encoder_manage(int *data)
{
    //编码器反向时，转为与正向时对应的负数
    if(*data>(MAX_VALUE*0.5f))
        *data = *data - MAX_VALUE;
    else 
        *data = *data;
}




