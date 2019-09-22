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
* ��������: ������ʱ��Ӳ����ʼ������
* �������: htim_base��������ʱ���������ָ��
* �� �� ֵ: ��
* ˵    ��: �ú�����HAL���ڲ�����
*/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
	if (htim_base->Instance == GENERAL_TIMx)
	{
		/* ������ʱ������ʱ��ʹ�� */
		GENERAL_TIM_RCC_CLK_ENABLE();

		/* �����ж����� */
		HAL_NVIC_SetPriority(GENERAL_TIM_IRQ, 0, 0);
		HAL_NVIC_EnableIRQ(GENERAL_TIM_IRQ);
	}
}

/**
* ��������: ������ʱ��Ӳ������ʼ������
* �������: htim_base��������ʱ���������ָ��
* �� �� ֵ: ��
* ˵    ��: �ú�����HAL���ڲ�����
*/
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

	if (htim_base->Instance == GENERAL_TIMx)
	{
		/* ������ʱ������ʱ�ӽ��� */
		GENERAL_TIM_RCC_CLK_DISABLE();

		/* �ر������ж� */
		HAL_NVIC_DisableIRQ(GENERAL_TIM_IRQ);
	}
}

extern "C" void TIM2_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htimx);
}
