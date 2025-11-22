#ifndef _usart2_h_
#define _usart2_h_

#define USARTx_DEBUG

#include "stm32f10x.h"
#include "stdio.h"

#define BLUETOOTH_RCC RCC_APB2Periph_GPIOB
#define BLUETOOTH_PORT   GPIOB
#define BLUETOOTH_TX_PIN GPIO_Pin_10
#define BLUETOOTH_RX_PIN GPIO_Pin_11

#define BLUETOOTH_USARTx USART3
#define BluetoothUploadApp(pstr)     BluetoothSendStr(pstr)
#define USARTxBlueTooth_RX_LEN 100 

#define BLUE_ROCKER_MODE    0

typedef struct BluetoothParseMsg
{
	float Xrocker;
    float Yrocker;
    uint8_t ModeRocker;
    uint8_t OpenCloseStatus;
}BluetoothParse;

void BluetoothUsart_init(u32 bound);
void BluetoothSendStr(char *pstr);
void pu8printf(char* name, uint8_t* pbuf, uint32_t count, uint8_t Mode);
uint8_t ParseBluetoothMessage(char *pInput , BluetoothParse *blueParseMsg);


extern uint16_t BlueToothBufIndex ;
extern uint32_t BlueToothNumbers ; 
extern char USARTxBlueTooth_RX_DATA[USARTxBlueTooth_RX_LEN];
extern BluetoothParse BluetoothParseMsg ;
extern uint8_t BlueRcvLen ; 
extern char BlueRecvData ;

#endif






