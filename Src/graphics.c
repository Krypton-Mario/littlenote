//graphics.c
//First edited in 2016-11-22
//Author:q.t

#include "graphics.h"
#include "led_rgb_driver.h"
#include <string.h>

uint16_t buf[16*6];

uint32_t tick = 0;

extern void Beep(uint16_t volume,uint16_t time);

void LedDrawLine(uint16_t r,uint16_t g,uint16_t b)
{
//	uint16_t r = 0x0001,g = 0x0001,b = 0x0001;
	uint8_t i=3;

	memset(buf,0x00,sizeof(buf));
	
#ifndef COLOR_BAR
	for(i = 32;i > 16;i--)
	{
		//High B
		buf[(32-i)*6] = b;
	}
	
	for(i = 16;i > 0;i--)
	{
		//Low B
		buf[(16-i)*6+1] = b;
	}

	for(i = 32;i > 16;i--)
	{
		//High G
		buf[(32-i)*6+2] = g;
	}
	
	for(i = 16;i > 0;i--)
	{
		//Low G
		buf[(16-i)*6+3] = g;
	}

	for(i = 32;i > 16;i--)
	{
		//High R
		buf[(32-i)*6+4] = r;
	}
	
	for(i = 16;i > 0;i--)
	{
		//Low R
		buf[(16-i)*6+5] = r;
	}
	
#else
	for(i = 32;i > 21;i--)
	{
		//High B
		buf[(32-i)*6] = b;
	}
	
//	for(i = 16;i > 0;i--)
//	{
//		//Low B
//		buf[(16-i)*6+1] = b;
//	}

	for(i = 21;i > 16;i--)
	{
		//High G
		buf[(32-i)*6+2] = g;
	}
	
	for(i = 16;i > 10;i--)
	{
		//Low G
		buf[(16-i)*6+3] = g;
	}

//	for(i = 32;i > 16;i--)
//	{
//		//High R
//		buf[(32-i)*6+4] = r;
//	}
	
	for(i = 10;i > 0;i--)
	{
		//Low R
		buf[(16-i)*6+5] = r;
	}
#endif
	
	LedCol(buf);
}

//Draw a picture
void LedDrawBMP(uint8_t* pbmp)
{
	uint8_t row,i;

	for(row = 0;row < 16;row++)
	{
		LedRow(row);

		for(i = 32;i > 16;i--)
		{
			//High B
			buf[(32-i)*6] = pbmp[row*32*3+ (32-i)*3];
		}

		for(i = 16;i > 0;i--)
		{
			//Low B
			buf[(16-i)*6+1] = pbmp[row*32*3 + (32-i)*3];
		}

		for(i = 32;i > 16;i--)
		{
			//High G
			buf[(32-i)*6+2] = pbmp[row*32*3 + (32-i)*3+1];
		}

		for(i = 16;i > 0;i--)
		{
			//Low G
			buf[(16-i)*6+3] = pbmp[row*32*3 + (32-i)*3+1];
		}

		for(i = 32;i > 16;i--)
		{
			//High R
			buf[(32-i)*6+4] = pbmp[row*32*3 + (32-i)*3+2];
		}

		for(i = 16;i > 0;i--)
		{
			//Low R
			buf[(16-i)*6+5] = pbmp[row*32*3 + (32-i)*3+2];
		}

		LedCol(buf);
		memset(buf,0x00,sizeof(buf));
	}

}

//Draw a picture
void LedDrawBMP_L_TO_R(uint8_t* pbmp)
{
	uint8_t row,i;
	
	while(1) {
		if(HAL_GetTick() - tick > 5 && tick != 0)
		{
			Beep(1,1);
		}
		tick = HAL_GetTick();
		
	for(row = 0;row < 3;row++)
	{
		for(i = 32;i > 16;i--)
		{
			//High B
			buf[(32-i)*6] = pbmp[((row+2)%16)*32*3+ i*3];
		}
		
		for(i = 16;i > 0;i--)
		{
			//Low B
			buf[(16-i)*6+1] = pbmp[((row+2)%16)*32*3 + i*3];
		}

		for(i = 32;i > 16;i--)
		{
			//High G
			buf[(32-i)*6+2] = pbmp[((row+2)%16)*32*3 + i*3+1];
		}
		
		for(i = 16;i > 0;i--)
		{
			//Low G
			buf[(16-i)*6+3] = pbmp[((row+2)%16)*32*3 + i*3+1];
		}

		for(i = 32;i > 16;i--)
		{
			//High R
			buf[(32-i)*6+4] = pbmp[((row+2)%16)*32*3 + i*3+2];
		}
		
		for(i = 16;i > 0;i--)
		{
			//Low R
			buf[(16-i)*6+5] = pbmp[((row+2)%16)*32*3 + i*3+2];
		}

		LedCol(buf);
		while(1)
		{
		for(row = 0;row < 1;row++)
	  {
			HAL_Delay(10);
		  LedRow(row);
		}
	  }
		//memset(buf,0x00,sizeof(buf));
	}
}
}
