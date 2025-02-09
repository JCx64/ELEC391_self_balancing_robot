/**
 * @file set_pwm.cpp
 * @author Jiayi Chen
 * @brief Set left wheel and right wheel pwm (mapping to -100~100)
 * @date 2025/02/07
 *
 * @copyright UBC ELEC391 2024-2025 Winter Term 2 Team B2 (Jiayi Chen)
 *
 */

#include "initialization.h"

void init_imu(){
    if(!IMU.begin()){
        while (1);
    }
}

void init_timer(){
    NRF52_MBED_Timer ITimer0 (NRF_TIMER_3); //生成一个Timer对象，这里用ITimer0命名，0和所选的timer3无关
}