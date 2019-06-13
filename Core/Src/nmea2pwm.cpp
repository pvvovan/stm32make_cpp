#include "nmea2pwm.h"
#include "cmsis_os.h"
#include <vector>
#include <mutex>
#include "uart.h"
#include <memory>
#include "TinyGPSpp.h"

namespace Model
{

    void interrupt_lock::lock()
    {
        m_primask = __get_PRIMASK();
        __disable_irq();
    }
    
    void interrupt_lock::unlock()
    {
        if (!m_primask)
            __enable_irq();
    }

    nmea2pwm::nmea2pwm(TIM_HandleTypeDef& timer):
        m_barrier {0},
        hwtimer {timer}
    {

    }

    nmea2pwm::timer_set nmea2pwm::get_pwm_set(double speed)
    {
        constexpr double threshold = 0.9;
        constexpr uint32_t base_period = 10000;

        nmea2pwm::timer_set set{};
        set.period = base_period / speed;
        set.duty_cycle = set.period / 2;
        set.threshold = threshold * set.period;
        return set;
    }

    void nmea2pwm::set_timer(timer_set pwm_set, TIM_HandleTypeDef& timer)
    {
        interrupt_lock ilock { };
        std::scoped_lock sl (ilock);
        timer.Instance->ARR = pwm_set.period;
        timer.Instance->CCR1 = pwm_set.duty_cycle;
        if (timer.Instance->CNT > pwm_set.threshold) {
            timer.Instance->CNT = 1;
        }
    }


    void nmea2pwm::start(UART_HandleTypeDef& huart)
    {
        auto serial {std::make_unique<gnss_radar::serial::uart>(4096, 4096, huart)};
        serial->available();
        TinyGPSPlus gps{};


        HAL_TIM_PWM_Start(&hwtimer, TIM_CHANNEL_1);
        for(;;) {
            osDelay(300);
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);

            auto buf = serial->read();
            for(auto c : buf) {
                gps.encode(c);
            }
            auto speed = gps.speed.kmph();
            if (speed == 0)
                speed = 1;
            auto pwm_set = get_pwm_set(speed);
            set_timer(pwm_set, hwtimer);
        }
    }
}
