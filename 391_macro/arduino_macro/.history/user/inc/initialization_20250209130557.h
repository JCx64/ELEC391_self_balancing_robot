/**
 * @file set_pwm.h
 * @author Jiayi Chen
 * @brief Set left wheel and right wheel pwm (mapping to -100~100)
 * @date 2025/02/07
 *
 * @copyright UBC ELEC391 2024-2025 Winter Term 2 Team B2
 *
 */

#include "Arduino_BMI270_BMM150.h"
#include "NRF52_MBED_TimerInterrupt.h"
#include "angle_measurement.h"
#include "pid_alg.h"
#include "set_pwm.h"

#define TIMER1_INTERVAL_MS 2000

class class_initialization{
    private:

    public:
        void init_imu();
        void TimerHandl1();
        void init_timer();
        void init_all();
        int TIMER1_INTERVAL_MS
}