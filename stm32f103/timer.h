#pragma once
#include <stm32f1xx_hal.h>
#include <functional>
#define GENERAL_TIMx                     TIM2
#define GENERAL_TIM_RCC_CLK_ENABLE()     __HAL_RCC_TIM2_CLK_ENABLE()
#define GENERAL_TIM_RCC_CLK_DISABLE()    __HAL_RCC_TIM2_CLK_DISABLE()
#define GENERAL_TIM_IRQ                  TIM2_IRQn
#define GENERAL_TIM_INT_FUN              TIM2_IRQHandler
// 定义定时器预分频，定时器实际时钟频率为：72MHz/（GENERAL_TIMx_PRESCALER+1）
#define GENERAL_TIM_PRESCALER            71  // 实际时钟频率为：1MHz

// 定义定时器周期，当定时器开始计数到GENERAL_TIMx_PERIOD值是更新定时器并生成对应事件和中断
#define GENERAL_TIM_PERIOD               1000000/120 - 1  // 定时器产生中断频率为：1MHz/1000=1KHz，即1ms定时周期

extern TIM_HandleTypeDef htimx;

void GENERAL_TIMx_Init(void);