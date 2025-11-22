/**
  ******************************************************************************
  * @file    ultrasonic.h
  * @author  Mr.Lin
  * @version V21.03.03
  * @date    03-Mar-2021
  * @brief   
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 源动力科技</center></h2>
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
/* Hardware and starter kit includes. */
/* Kernel includes. */
/* application includes */
/* Exported macros -----------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
int HCSR04_Init(void);
void UltrasonicWave_StartMeasure(void);
void Getdis(void);


uint8_t UltrasonicCheck(void);
uint8_t IsUltrasonicOK(void);

uint8_t UltraSuccess(uint8_t rcMode);
extern int HcGetDistance(float *getDistance);
extern float getDistance;
#endif /* __ULTRASONIC_H */
/****************** (C) COPYRIGHT Mr.Lin @ 源动力科技 *********END OF FILE****/
