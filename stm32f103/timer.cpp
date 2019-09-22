#include "timer.h"

TIM_HandleTypeDef htimx;

void GENERAL_TIMx_Init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htimx.Instance = GENERAL_TIMx;
	htimx.Init.Prescaler = GENERAL_TIM_PRESCALER;
	htimx.Init.CounterMode = TIM_COUNTERMODE_UP;
	htimx.Init.Period = GENERAL_TIM_PERIOD;
	htimx.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&htimx);

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htimx, &sClockSourceConfig);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htimx, &sMasterConfig);
	HAL_TIM_Base_Start_IT(&htimx);
}

/**
* 函数功能: 基本定时器硬件初始化配置
* 输入参数: htim_base：基本定时器句柄类型指针
* 返 回 值: 无
* 说    明: 该函数被HAL库内部调用
*/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
	if (htim_base->Instance == GENERAL_TIMx)
	{
		/* 基本定时器外设时钟使能 */
		GENERAL_TIM_RCC_CLK_ENABLE();

		/* 外设中断配置 */
		HAL_NVIC_SetPriority(GENERAL_TIM_IRQ, 0, 0);
		HAL_NVIC_EnableIRQ(GENERAL_TIM_IRQ);
	}
}

/**
* 函数功能: 基本定时器硬件反初始化配置
* 输入参数: htim_base：基本定时器句柄类型指针
* 返 回 值: 无
* 说    明: 该函数被HAL库内部调用
*/
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

	if (htim_base->Instance == GENERAL_TIMx)
	{
		/* 基本定时器外设时钟禁用 */
		GENERAL_TIM_RCC_CLK_DISABLE();

		/* 关闭外设中断 */
		HAL_NVIC_DisableIRQ(GENERAL_TIM_IRQ);
	}
}

extern "C" void TIM2_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htimx);
}
