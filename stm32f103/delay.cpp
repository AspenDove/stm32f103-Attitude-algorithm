#include "delay.h"

void delay_init(uint8_t SYSCLK)
{
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);//SysTickƵ��ΪHCLK
	fac_us = SYSCLK;						//�����Ƿ�ʹ��OS,fac_us����Ҫʹ��
}

void delay_us(uint32_t nus)
{
	uint32_t ticks;
	uint32_t told, tnow, tcnt = 0;
	uint32_t reload = SysTick->LOAD;				//LOAD��ֵ	    	 
	ticks = nus*fac_us; 						//��Ҫ�Ľ����� 
	told = SysTick->VAL;        				//�ս���ʱ�ļ�����ֵ
	while (1)
	{
		tnow = SysTick->VAL;
		if (tnow != told)
		{
			if (tnow < told)tcnt += told - tnow;	//����ע��һ��SYSTICK��һ���ݼ��ļ������Ϳ�����.
			else tcnt += reload - tnow + told;
			told = tnow;
			if (tcnt >= ticks)break;			//ʱ�䳬��/����Ҫ�ӳٵ�ʱ��,���˳�.
		}
	}
}
/*void delay_us(uint32_t nus)
{
uint32_t temp;
SysTick->LOAD = fac_us*nus;
SysTick->VAL = 0X00;//��ռ�����
SysTick->CTRL = 0X01;//ʹ�ܣ����������޶����������ⲿʱ��Դ
do
{
temp = SysTick->CTRL;//��ȡ��ǰ������ֵ
} while ((temp & 0x01)&&(!(temp&(1 << 16))));//�ȴ�ʱ�䵽��
SysTick->CTRL = 0x00; //�رռ�����
SysTick->VAL = 0X00; //��ռ�����
}*/
void start(void)
{
	SysTick->LOAD = 0xffffff;
	SysTick->VAL = 0X00;//��ռ�����
	SysTick->CTRL = 0X01;//ʹ�ܣ����������޶����������ⲿʱ��Դ
}
float count(void)
{
	//SysTick->CTRL = 0x00; //�رռ�����
	//SysTick->VAL = 0X00; //��ռ�����

	return (0xffffff - SysTick->VAL) / 180.f;
}
void delay_ms(uint16_t nms)
{
	for (uint32_t i = 0; i < nms; i++) delay_us(1000);
}


