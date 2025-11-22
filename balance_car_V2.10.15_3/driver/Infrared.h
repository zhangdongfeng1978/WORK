/**
  ******************************************************************************
  * @file    Infrared.h
  * @author  Mr.Lin
  * @version V21.03.04
  * @date    04-Mar-2021
  * @brief   
  * @Tips    [ C - Standardization Tool ] [ By weimin ]
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 源动力科技</center></h2>
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __INFRARED_H
#define __INFRARED_H

/* Includes ------------------------------------------------------------------*/
/* Hardware and starter kit includes. */
/* Kernel includes. */
#include "stm32f10x.h"
/* application includes */
/* Exported macros -----------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum g_InfraredChannel
{
	InfraredChannel1 = (0x01 << 0),
	InfraredChannel2 = (0x01 << 1),
	InfraredChannel3 = (0x01 << 2),
	InfraredChannel4 = (0x01 << 3),
}_InfraredChannel ;

/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void Infrared_init(void);
uint8_t getInfaredStatus(void);
 
#endif /* __INFRARED_H */
/****************** (C) COPYRIGHT Mr.Lin @ 源动力科技 *********END OF FILE****/
