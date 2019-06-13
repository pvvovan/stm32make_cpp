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

    // void nmea2pwm::set_timer(nmea2pwm::timer_set pwm_set, TIM_HandleTypeDef& timer)
    // {
    //     uint32_t prim = __get_PRIMASK();
    //     __disable_irq();
    //     timer.Instance->ARR = pwm_set.period;
    //     timer.Instance->CCR1 = pwm_set.duty_cycle;
    //     if (timer.Instance->CNT > pwm_set.threshold) {
    //         timer.Instance->CNT = 1;
    //     }
    //     if (!prim)
    //         __enable_irq();
    // }

    // void nmea2pwm::set_timer(nmea2pwm::timer_set pwm_set, TIM_HandleTypeDef& timer)
    // {
    //     m_barrier.store(0);
    //     uint32_t const cur_pri = __get_BASEPRI(); m_barrier++;
    //     __disable_irq(); m_barrier++;
    //     __set_BASEPRI(1 << 4); m_barrier++;
    //     timer.Instance->ARR = pwm_set.period; m_barrier++;
    //     timer.Instance->CCR1 = pwm_set.duty_cycle; m_barrier++;
    //     if (timer.Instance->CNT > pwm_set.threshold) {
    //         timer.Instance->CNT = 1; m_barrier++;
    //     }
    //      __set_BASEPRI(cur_pri); m_barrier++;
    //      __enable_irq(); m_barrier++;
    // }

    void nmea2pwm::start(UART_HandleTypeDef& huart)
    {
        auto serial {std::make_unique<gnss_radar::serial::uart>(4096, 4096, huart)};
        serial->available();
        TinyGPSPlus gps{};

        // gnss_radar::serial::uart a {gnss_radar::serial::uart(1, 1)};
        // a.available();
        // a = std::move(a);
        // gnss_radar::serial::uart b {std::move(a)};

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

            // osDelay(1000);
            // auto pwm_set = get_pwm_set(3);
            // set_timer(pwm_set, hwtimer);

            // osDelay(1000);
            // pwm_set = get_pwm_set(9);
            // set_timer(pwm_set, hwtimer);
            

            // continue;
            // osDelay(1000);		
            // hwtimer.Instance->ARR = 200;
            // hwtimer.Instance->CCR1 = 100;
            // hwtimer.Instance->CNT = 1;

            // osDelay(1000);
            // hwtimer.Instance->ARR = 1000;
            // hwtimer.Instance->CCR1 = 500;
            // hwtimer.Instance->CNT = 1;

            // serial->write(std::vector<uint8_t>{'H', '\n'});
            // TinyGPSPlus gps{};
            // const char* rmc = "$GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68\r\n";
            // for(char const* c = rmc; *c; c++) {
            //     gps.encode(*c);
            // }
            // auto speed = gps.speed.kmph();
            // HAL_Delay(speed);
        }
    }
}