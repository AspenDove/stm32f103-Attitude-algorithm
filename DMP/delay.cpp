#include "delay.h"

void delay_us(uint32_t n)
{
	uint32_t j;
	while(n--)
	for(j=0;j<10;j++);
}

void delay_ms(uint32_t n)
{
	while(n--)
	delay_us(1000);
}


void Delay(uint32_t time)
{
    while(time--);
    
}
