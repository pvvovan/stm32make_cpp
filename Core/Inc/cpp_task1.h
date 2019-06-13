#ifndef _CPP_TASK1_H
#define _CPP_TASK1_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void start_cpp_task1(TIM_HandleTypeDef* hwtimer, UART_HandleTypeDef* huart);

#ifdef __cplusplus
}
#endif

#endif