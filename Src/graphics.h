//graphics.h
//Firsted edited in 2016-11-22
//Author: q.t
#ifndef		__GRAPHICS_H__
#define		__GRAPHICS_H__

#include "stm32f1xx_hal.h"

void LedDrawLine(uint16_t r,uint16_t g,uint16_t b);
void LedDrawBMP(uint8_t* pbmp);
void LedDrawBMP_L_TO_R(uint8_t* pbmp);
#endif

