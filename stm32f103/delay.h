#pragma once
#include "stm32f1xx_hal.h"

static uint32_t fac_us = 0;							//us��ʱ������
void delay_init(uint8_t SYSCLK);
void delay_ms(uint16_t nms);
void delay_us(uint32_t nus);
void start(void);
float count(void);

