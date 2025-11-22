/**********************************************************************
版权所有：源动力科技
淘    宝：https://1024tech.taobao.com/
文 件 名：adc.c
版    本：V21.01.30
摘    要：
***********************************************************************/
#include "adc.h"

_ADC_VALUE bat = {0 ,0 };

uint16_t adc_value[1];

/* 端口配置初始化 */
void adc_gpio_init(void)
{
	GPIO_InitTypeDef GPIO_initStructure;    
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_initStructure.GPIO_Pin = GPIO_Pin_2;	    
	GPIO_initStructure.GPIO_Mode = GPIO_Mode_AIN;								    
	GPIO_Init(GPIOA,&GPIO_initStructure);	
}

/* adc配置 */ 
void adc_config(void)
{
    ADC_InitTypeDef ADC_initStructure;
    
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	
	ADC_initStructure.ADC_ContinuousConvMode = ENABLE;					        //连续转换	
	ADC_initStructure.ADC_DataAlign = ADC_DataAlign_Right;		                //数据右对齐				
	ADC_initStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	        //软件触发 
	ADC_initStructure.ADC_Mode = ADC_Mode_Independent;							//独立工作模式
	ADC_initStructure.ADC_NbrOfChannel = 1;										//顺序进行规则转换的ADC通道的数目 1-16    
	ADC_initStructure.ADC_ScanConvMode = DISABLE;		                        //扫描模式：单次模式						
	ADC_Init(ADC1,&ADC_initStructure);

	ADC_Cmd(ADC1,ENABLE);
    
	ADC_DMACmd(ADC1,ENABLE);
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);                                           //ADC时钟分频											
                                                                                //通道配置，采样时间设置
	ADC_RegularChannelConfig(ADC1,ADC_Channel_2,1,ADC_SampleTime_239Cycles5);   
	
	ADC_ResetCalibration(ADC1);	                                                //复位校准				
	while(ADC_GetCalibrationStatus(ADC1));		                                //等待
	ADC_StartCalibration(ADC1);					                                //启动校准
	while(ADC_GetCalibrationStatus(ADC1));		                                //等待校准完成

	ADC_SoftwareStartConvCmd(ADC1,ENABLE);	                                    //开启转换                                     
}

void dma_config(void)
{
	DMA_InitTypeDef DMA_initStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	
	DMA_initStructure.DMA_BufferSize = 1;				                        //缓存的数据大小						
	DMA_initStructure.DMA_DIR = DMA_DIR_PeripheralSRC;	                        //传输方向：外设->内存						
	DMA_initStructure.DMA_M2M = DMA_M2M_Disable;								
	DMA_initStructure.DMA_MemoryBaseAddr = (u32)adc_value;				        //内存基地址
	DMA_initStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;         //DMA传输的内存数据大小：半字为单位			
	DMA_initStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;						//内存地址自增
	DMA_initStructure.DMA_Mode = DMA_Mode_Circular;								//循环模式
	DMA_initStructure.DMA_PeripheralBaseAddr = ((u32)&ADC1->DR);	            //外设地址			
	DMA_initStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//DMA传输的外设数据大小：半字为单位
	DMA_initStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			//外设地址不变
	DMA_initStructure.DMA_Priority = DMA_Priority_Medium;						//中等优先级
	DMA_Init(DMA1_Channel1,&DMA_initStructure);
    
	DMA_Cmd(DMA1_Channel1,ENABLE);	
}

void adc_init(void)
{
    adc_gpio_init();
    adc_config();
    dma_config();
}
/*
 * 函数名：voltage_detection
 * 描述  ：电源电压采集
 * 输入  ：    
 * 返回  ：     
 */
void voltage_detection(void)
{
    bat.adc = adc_value[0];
    bat.voltage = (float)(bat.adc)/4096.0f*3.3f*11.0f;      //实际电路采用电阻10:1比例进行分压
    
    if(bat.voltage<=7.0f)
    {
        bat.danger_flag = 1;
    }
    else
    {
        bat.danger_flag = 0;
    }
}    








