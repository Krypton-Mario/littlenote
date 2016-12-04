//key.h
//First edited in 2016-11-22
//Author: q.t

#ifndef		__KEY_H__
#define		__KEY_H__

#include "stm32f1xx_hal.h"

#define		KEY_PUSH_DOWN			0
#define		KEY_PUSH_UP				1

#define		KEY0							GPIOA,GPIO_PIN_0

uint8_t ReadKey();
void Beep(uint16_t volume,uint16_t time);
	
#endif

