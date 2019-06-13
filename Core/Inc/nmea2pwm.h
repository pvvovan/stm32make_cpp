#ifndef _NMEA_TO_PWM_H
#define _NMEA_TO_PWM_H

#include "stm32f4xx_hal.h"
#include <atomic>

namespace Model
{
    class interrupt_lock
    {
    private:
        uint32_t m_primask {0};
    public:
        void lock();
        void unlock();

        interrupt_lock() = default;
        ~interrupt_lock() = default;
        interrupt_lock& operator=(const interrupt_lock&) = delete;
        interrupt_lock(const interrupt_lock&) = delete;
    };
    



    class nmea2pwm
    {
    private:
        struct timer_set
        {
            uint32_t period;
            uint32_t duty_cycle;
            uint32_t threshold;
        };

        std::atomic<int> m_barrier;
        TIM_HandleTypeDef& hwtimer;
        

        void set_timer(timer_set pwm_set, TIM_HandleTypeDef& timer);
        timer_set get_pwm_set(double speed);


    public:
        nmea2pwm(TIM_HandleTypeDef& timer);
        ~nmea2pwm() = default;
        void start(UART_HandleTypeDef& huart);

        
    };

    
    
}

#endif /* _NMEA_TO_PWM_H */