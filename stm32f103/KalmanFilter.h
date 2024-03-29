#pragma once
#include <arm_math.h>
#include "stm32f1xx_hal.h"

extern float32_t Axyz[3];
extern float32_t Gxyz[3];
extern float32_t Mxyz[3];

extern float32_t MagConst[3];

void initMatrix(void);
void runKalmanFilter(void);
