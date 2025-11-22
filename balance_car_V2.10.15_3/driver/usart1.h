#ifndef _usart_h_
#define _usart_h_

#include "stm32f10x.h"
#include "stdio.h"

#define SENSOR_GYRO 02
#define USART_RX_LEN 100 

#define CAL_SUCCESS             0x01
#define SETUP_SUCCESS           0x31
#define RESTORE_DEFAULT_SUCCESS 0xA1

typedef struct 
{
    u8 send_version;
    u8 send_status;
    u8 send_senser;
    u8 send_pid1;
    u8 send_pid2;
    u8 send_pid3;
    u8 send_pid4;
    u8 send_pid5;
    u8 send_pid6;
    u8 send_rcdata;
    u8 send_offset;
    u8 send_motopwm;
    u8 send_power;

}dt_flag_t;

#define USART1_RX_MODE_ON

void usart1_init(u32 bound);
void usart1_send(void* buf, int len);
void USARTx_DMA_RX_Config(DMA_Channel_TypeDef* DMA_CHx,u32 peripheral_addr,u32 memory_addr,u16 data_length);
void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num);
void ANO_DMA_SEND_DATA(void);
extern uint8_t USART_RX_DATA[USART_RX_LEN];        
extern uint32_t Numbers;
extern uint16_t bufIndex;
#endif



