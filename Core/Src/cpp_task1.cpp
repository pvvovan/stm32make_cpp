#include "cpp_task1.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <vector>
#include "nmea2pwm.h"

extern "C" void start_cpp_task1(TIM_HandleTypeDef* hwtimer, UART_HandleTypeDef* huart)
{
    // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    Model::nmea2pwm(*hwtimer).start(*huart);

	for(;;)
	{
        //HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);

        osDelay(1000);		
        // htim2.Instance->ARR = 200;
        // htim2.Instance->CCR1 = 100;
        // htim2.Instance->CNT = 1;
        std::vector<int> vec{};
        vec.emplace_back(4);

        osDelay(1000);
        // htim2.Instance->ARR = 1000;
        // htim2.Instance->CCR1 = 500;
        // htim2.Instance->CNT = 1;

	}
}