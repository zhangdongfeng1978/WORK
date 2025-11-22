/**********************************************************************
版权所有：源动力科技
淘    宝：https://1024tech.taobao.com/
文 件 名：bluetooth.c
版    本：V21.01.30
摘    要：
***********************************************************************/
#include "bluetooth.h"
#include "usart1.h"
#include "mpu6050.h"
#include "imu.h"
#include "imath.h"
#include "systick.h"
#include "key.h"
#include "pwm.h"
#include "pid.h"
#include "controller.h"
#include "timer.h"
#include "flash.h"
#include "string.h"
#include <stdlib.h>
#include <stdio.h>

uint16_t BlueToothBufIndex ;
uint32_t BlueToothNumbers ; 
char BlueTooth_data_to_send[100];                            //发送数据缓存
char USARTxBlueTooth_RX_DATA[USARTxBlueTooth_RX_LEN];        //接收数据缓存
dt_flag_t Bluetooth;					                        //需要发送数据的标志 
BluetoothParse BluetoothParseMsg = {2,2,0,0};

char BlueRecvData ;
uint8_t BlueRcvLen ; 
/*
 * 函数名：BluetoothSendByte
 * 描述  ：蓝牙单字符发送
 * 输入  ：pbyte发送的字符
 * 返回  ：
 */    
void BluetoothSendByte( char pbyte)      
{
    USART_SendData(USART3, pbyte);         
    while( USART_GetFlagStatus(USART3,USART_FLAG_TC)!= SET);  
}
/*
 * 函数名：BluetoothSendStr
 * 描述  ：蓝牙字符串发送
 * 输入  ：pstr发送的字符串首地址
 * 返回  ：
 */    
void BluetoothSendStr(char *pstr)
{
	unsigned short i,len;
	len = strlen(pstr);
	for(i=0; i<len; i++)
        BluetoothSendByte(*pstr++);
}
/*
 * 函数名：pu8printf
 * 描述  ：串口打印数组
 * 输入  ：name字符串文本名, pbuf待打印数组 , count数组宽度 ，Mode显示模式
 * 返回  ：
 */    
void pu8printf(char* name, uint8_t* pbuf, uint32_t count, uint8_t Mode)
{
	uint32_t index;
	printf("%s:\r\n", name);
	for(index = 0; index < count; index++)
	{
		if(0 == Mode)
		{
			printf("%02d ", pbuf[index]);
		}
		else if(1 == Mode)
		{
			printf("%02X ", pbuf[index]);
		}
        else if(2 == Mode)
        {
//            printf("%s ", pbuf[index]);
        }
	}
	printf("\r\n");
}
/*
 * 函数名：BluetoothUsart_init
 * 描述  ：蓝牙串口初始化
 * 输入  ：波特率
 * 返回  ：
 */    
void BluetoothUsart_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(BLUETOOTH_RCC | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    //  TX
	GPIO_InitStructure.GPIO_Pin = BLUETOOTH_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(BLUETOOTH_PORT, &GPIO_InitStructure);
    //  RX
	GPIO_InitStructure.GPIO_Pin = BLUETOOTH_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(BLUETOOTH_PORT, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = bound; 
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     //8bits
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                          //stop bit is 1
	USART_InitStructure.USART_Parity = USART_Parity_No;                             //no parity
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //no Hardware Flow Control
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;                 //enable tx and rx
	USART_Init(BLUETOOTH_USARTx, &USART_InitStructure);

    USART_ITConfig(BLUETOOTH_USARTx,USART_IT_RXNE,ENABLE);                          //rx interrupt is enable
	USART_Cmd(BLUETOOTH_USARTx, ENABLE);    
}  
 

/*
 * 函数名：USART3_IRQHandler
 * 描述  ：蓝牙串口接收中断处理函数
 * 输入  ：
 * 返回  ：
 */    
void USART3_IRQHandler(void)                                 
{     
    if(USART_GetITStatus(BLUETOOTH_USARTx, USART_IT_RXNE) != RESET)  
    {  
        if(BlueToothBufIndex > USARTxBlueTooth_RX_LEN)
            BlueToothBufIndex = 0;
		USARTxBlueTooth_RX_DATA[BlueToothBufIndex++] = USART_ReceiveData(BLUETOOTH_USARTx);
  
        BlueToothNumbers ++;
        USART_ClearITPendingBit(BLUETOOTH_USARTx, USART_IT_RXNE);                             //清除空闲中断标志
    }      
}   
/*
 * 函数名：ParseBluetoothMessage
 * 描述  ：解析蓝牙app发送过来的数据包
 * 输入  ：pInput收到的蓝牙遥控数据首地址 , blueParseMsg解析后保存的蓝牙数据有效信息
 * 返回  ：err
 */
/*
 * 协议形式：字符串
 * 帧头：# 帧尾：*
 * 例如：#9,1,1,2,2,1*  
 * 每位代表含义：9：数据长度(固定) 1：模式 1：关闭/开始状态 2：蓝牙摇杆X轴数据 2：蓝牙摇杆Y轴数据 1：校验值(累加和减去7)
*/
uint8_t ParseBluetoothMessage(char *pInput , BluetoothParse *blueParseMsg)
{
    unsigned char plen,sum,check;
    ErrorStatus err;
    char *pdata = pInput;
    
    while(( pdata-pInput ) < BlueToothBufIndex )
    {
        if(*pdata == '#')//  #9,1,1,2,2,1*                                                      //接收到数据的包头#
        {
            plen = (unsigned char)atof(strtok(pdata+1, ","));                                   //解析长度length
            if(plen == 9)
            {
                /* 将读出的数据进行累加校验 */ 
                sum =   (unsigned char)int_abs( ((int)atof(strtok(pdata+3, ",")) + 
                        (int)atof(strtok(NULL, ",")) +
                        (int)atof(strtok(NULL, ",")) + 
                        (int)atof(strtok(NULL, ","))) - 7 );
                /* 读出累加校验数据 */
                check = (unsigned char)atof(strtok(NULL, ",")) ;                                //累加校验数据
                if(sum == check)                                                                //校验匹配成功才进行数据解析     
                {
                    blueParseMsg->ModeRocker = (unsigned char)atof(strtok(pdata+3, ","));       //模式数据
                    blueParseMsg->OpenCloseStatus = (unsigned char)atof(strtok(pdata+5, ","));  //关闭/开始状态数据
                    blueParseMsg->Xrocker = (unsigned char)atof(strtok(pdata+7, ","));          //摇杆X数据
                    blueParseMsg->Yrocker = (unsigned char)atof(strtok(pdata+9, ","));          //摇杆Y数据
                    err = SUCCESS;
                }
                else
                {
                    err = ERROR;  
                }
            }
            else 
            {
                err = ERROR;
            }
        }
        else 
        {
            err = ERROR;
        }
        pdata++;
    }
    BlueToothBufIndex = 0;

    return err ;
}

