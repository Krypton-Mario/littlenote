//key.c
//First edited in 2016-11-22
//Author: q.t

#include "key.h"

extern TIM_HandleTypeDef htim3;

uint8_t ReadKey()
{
	uint8_t status = KEY_PUSH_UP;
	
	if(HAL_GPIO_ReadPin(KEY0) == GPIO_PIN_RESET)
	{
		//key pushed
		HAL_Delay(20);
		if(HAL_GPIO_ReadPin(KEY0) == GPIO_PIN_RESET)
		{
			status = KEY_PUSH_DOWN;
		}
	}
	
	
	return status;
}

void Beep(uint16_t volume,uint16_t time)
{
	TIM_OC_InitTypeDef sConfigOC;
	
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = volume;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_Delay(time);
	HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);	
}

