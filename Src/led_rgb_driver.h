//led_rgb_driver.h
//First edited in 2016-11-17
//Author: QianTuo
#ifndef		__LED_RGB_DRIVER_H__
#define		__LED_RGB_DRIVER_H__

#include "stm32f1xx_hal.h"

#define		LED_ROW0			GPIOB,GPIO_PIN_6
#define		LED_ROW1			GPIOB,GPIO_PIN_7
#define		LED_ROW2			GPIOB,GPIO_PIN_8
#define		LED_ROW3			GPIOB,GPIO_PIN_9

#define		LED_LE				GPIOB,GPIO_PIN_11
#define		LED_DCLK			GPIOB,GPIO_PIN_13
#define		LED_SDI				GPIOB,GPIO_PIN_15

#define		LED_LE_Hi			HAL_GPIO_WritePin(LED_LE,GPIO_PIN_SET)
#define		LED_LE_Lo			HAL_GPIO_WritePin(LED_LE,GPIO_PIN_RESET)
#define		LED_DCLK_Hi		HAL_GPIO_WritePin(LED_DCLK,GPIO_PIN_SET)
#define		LED_DCLK_Lo		HAL_GPIO_WritePin(LED_DCLK,GPIO_PIN_RESET)
#define		LED_SDI_Hi		HAL_GPIO_WritePin(LED_SDI,GPIO_PIN_SET)
#define		LED_SDI_Lo		HAL_GPIO_WritePin(LED_SDI,GPIO_PIN_RESET)

#define LED_ON				(HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3))
#define LED_OFF				(HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3))

void LedRow(uint8_t r);
void LedCol(uint16_t* pc);
void MBI_Gain(uint8_t gain);
#endif

