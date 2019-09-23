#define DEBUG_DEFAULT_INTERRUPT_HANDLERS
#include <stm32f1xx_hal.h>
#include <math.h>
#include "IMU.h"
#include "MahonyAHRS.h"
#include "delay.h"
#include "usart.h" 
#include "timer.h"
#include "KalmanFilter.h"

#ifdef __cplusplus
extern "C"
#endif
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Initializes the CPU, AHB and APB busses clocks
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/**Initializes the CPU, AHB and APB busses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
		| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

	/**Configure the Systick interrupt time
	*/
	uint32_t hclk = HAL_RCC_GetHCLKFreq();
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	*/
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
extern "C" void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}
void HAL_SYSTICK_Callback(void)
{

}
extern float32_t data_matrix_X_prev[4];
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t txData[300];
	bool updated = IMU_Measure();
	//if (updated)
	//{
		sprintf((char*)txData, "%.lf %.lf %.lf %.lf %.lf %.lf %.lf %.lf %.lf\r\n",
			accel.x, accel.y, accel.z,
			gyro.x, gyro.y, gyro.z,
			mag.x, mag.y, mag.z);

	//	HAL_UART_Transmit_DMA(&huart1, txData, strlen((const char*)txData));
	//	//for d3d9 visualization
	////	float32_t q[4] = { q0,q1,q2,q3 };
	////	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)(&q), sizeof(q));
	//}
	Axyz[0] = accel.x;
	Axyz[1] = accel.y;
	Axyz[2] = accel.z;

	Gxyz[0] = gyro.x;
	Gxyz[1] = gyro.y;
	Gxyz[2] = gyro.z;

	Mxyz[0] = mag.x;
	Mxyz[1] = mag.y;
	Mxyz[2] = mag.z;
	//MahonyAHRSupdate(gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z, mag.y, mag.x, -mag.z, updated);
	//runKalmanFilter();
	//float32_t q0 = data_matrix_X_prev[3];
	//float32_t q1 = data_matrix_X_prev[0];
	//float32_t q2 = data_matrix_X_prev[1];
	//float32_t q3 = data_matrix_X_prev[2];

	//angle.roll = atan2f(q2 * q3 + q0 * q1, 0.5f - (q1*q1 + q2*q2));
	//angle.pitch = asinf(2.f*(q0 * q2 - q1 * q3));
	//angle.yaw = atan2f(q1 * q2 + q0 * q3, 0.5f - (q2*q2 + q3*q3));
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

int main(void)
{
	SystemClock_Config();
	delay_init(72);

	__HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_InitTypeDef  GPIO_InitStructure;

	GPIO_InitStructure.Pin = GPIO_PIN_13;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

	IMU_Init();
	USART1_UART_Init();

	IMU_Calibrate();
	initMatrix();
	GENERAL_TIMx_Init();
	

	uint8_t txData[100];

	while (1)
	{
		//sprintf((char*)txData, "%.2lf\t%.2lf\t%.2lf\r\n", mag.x, mag.y, mag.z);
		//HAL_UART_Transmit_DMA(&huart1, txData, strlen((const char*)txData));

	}
}
