#include "cpp_task1.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <vector>
#include "nmea2pwm.h"

extern "C" void start_cpp_task1(TIM_HandleTypeDef* hwtimer, UART_HandleTypeDef* huart)
{
	Model::nmea2pwm(*hwtimer).start(*huart);

	for(;;)
	{
        	osDelay(1000);
	}
}
