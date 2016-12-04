//led_rgb_driver.c
//First edited in 2016-11-17
//Author: QianTuo

#include	"led_rgb_driver.h"
#include 	<string.h>

void MBI_Gain(uint8_t gain)
{
	uint8_t i,j;
	uint16_t cfg = 0x0402 | (gain << 2);
	
	LED_LE_Lo;
	for(j = 0;j < 5;j++)
	{
		for(i = 16;i > 0;i--)
		{
			//DCLK low
			LED_DCLK_Lo;
			//SDI
			HAL_GPIO_WritePin(LED_SDI,(cfg >> (i-1)) & 0x01);
			//DCLK high
			LED_DCLK_Hi;
		}	
	}
	
	//Write configration
//	LED_LE_Lo;
	for(i = 16;i > 0;i--)
	{
		//DCLK low
		LED_DCLK_Lo;
		if(i == 10)
		{
			LED_LE_Hi;
		}
		//SDI
		HAL_GPIO_WritePin(LED_SDI,(cfg >> (i-1)) & 0x01);
		//DCLK high
		LED_DCLK_Hi;
	}
	LED_LE_Lo;
	LED_DCLK_Lo;
}

void LedRow(uint8_t r)
{
	uint8_t temp;
	
	HAL_GPIO_WritePin(LED_ROW0,r & 0x01);
	HAL_GPIO_WritePin(LED_ROW1,(r>>1) & 0x01);
	HAL_GPIO_WritePin(LED_ROW2,(r>>2) & 0x01);
	HAL_GPIO_WritePin(LED_ROW3,(r>>3) & 0x01);
}

//MBI5031 data load
//MSB ,
void LedCol(uint16_t* pc)
{
	uint8_t i,j,k;
	
	LED_LE_Lo;
	//6 MBI5031 ,Port15 to Port 1,16bits once
	//15 ports
 	for(k = 0;k < 15;k++)
	{
		//6 times
		for(j = 0;j < 5;j++)
		{
			//16bits
			for(i = 16;i > 0;i--)
			{
				//DCLK low
				LED_DCLK_Lo;
				//SDI
				HAL_GPIO_WritePin(LED_SDI,(pc[k*6+j] >> (i-1)) & 0x01);
				//DCLK high
				LED_DCLK_Hi;
			}
		}
		
		for(i = 16;i > 0;i--)
		{
			//DCLK low
			LED_DCLK_Lo;
			if(i == 1)
			{
				LED_LE_Hi;
			}
			//SDI
			HAL_GPIO_WritePin(LED_SDI,(pc[k*6+j] >> (i-1)) & 0x01);
			//DCLK high
			LED_DCLK_Hi;
		}
		LED_LE_Lo;
		LED_DCLK_Lo;			
	}
	
	//6 times
	for(j = 0;j < 5;j++)
	{
		//16bits
		for(i = 16;i > 0;i--)
		{
			//DCLK low
			LED_DCLK_Lo;
			//SDI
			HAL_GPIO_WritePin(LED_SDI,(pc[k*6+j] >> (i-1)) & 0x01);
			//DCLK high
			LED_DCLK_Hi;
		}
	}
	
	for(i = 16;i > 0;i--)
	{
		//DCLK low
		LED_DCLK_Lo;
		if(i == 2)
		{
			LED_LE_Hi;
		}
		//SDI
		HAL_GPIO_WritePin(LED_SDI,(pc[k*6+j] >> (i-1)) & 0x01);
		//DCLK high
		LED_DCLK_Hi;
	}
	LED_LE_Lo;
	LED_DCLK_Lo;
		
}
