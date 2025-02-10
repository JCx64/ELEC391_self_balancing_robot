/**
 * @file set_pwm.cpp
 * @author Jiayi Chen
 * @brief Set left wheel and right wheel pwm (mapping to -100~100)
 * @date 2025/02/07
 *
 * @copyright UBC ELEC391 2024-2025 Winter Term 2 Team B2 (Jiayi Chen)
 *
 */

#include "set_pwm.h"

void class_pwm::set_left_pwm(float pid_result){
    if(pid_result > 0){
        digitalWrite(5, HIGH);
        analogWrite(6, pid_result * reduction_factor);
    }else if(pid_result < 0){
        digitalWrite(5, LOW);
        analogWrite(6, pid_result * reduction_factor);
    }else{
        return;
    }
}

void set_right_pwm(float pid_result){
    if(pid_result > 0){
        digitalWrite(9, HIGH);
        analogWrite(10, pid_result * reduction_factor);
    }else if(pid_result < 0){
        digitalWrite(9, LOW);
        analogWrite(10, pid_result * reduction_factor);
    }else{
        return;
    }
}
