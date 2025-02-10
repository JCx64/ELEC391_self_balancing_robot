/**
 * @file set_pwm.cpp
 * @author Jiayi Chen, Jinke Su
 * @brief Set left wheel and right wheel pwm (mapping to -100~100)
 * @date 2025/02/07
 *
 * @copyright UBC ELEC391 2024-2025 Winter Term 2 Team B2
 *
 */

#include "initialization.h"

void class_initialization::init_imu(){
    if(!IMU.begin()){
        while (1);
    }
}

void class_initialization::TimerHandler1(){
    pid_TimerElapsedCallback();
}

void class_initialization::init_timer(){
    static NRF52_MBED_Timer Timer1 (NRF_TIMER_1); //生成一个Timer对象，这里用Timer1命名，1和所选的timer1无关
    Timer1.attachInterruptInterval(TIMER1_INTERVAL_MS * 1000, TimerHandler1); //设置对应定时器的中断频率和中断回调函数
}

void class_initialization::init_all(){
    init_imu();
    init_timer();
}