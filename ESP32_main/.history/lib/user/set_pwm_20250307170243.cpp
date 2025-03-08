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

PWMClass :: PWMClass(){
    
}

void  PWMClass::set_left_pwm (float pid_result){
    if(pid_result > 0){
        ledcWrite(1, 0);
        ledcWrite(255.0 - pid_result * reduction_factor, 255.0);
    }else if(pid_result < 0){
        ledcWrite(0, 0);
        ledcWrite(1, -pid_result * reduction_factor);
    }else{
        return;
    }
}

void PWMClass::set_right_pwm (float pid_result){
    if(pid_result > 0){
        ledcWrite(2, 0);
        ledcWrite(3, pid_result * reduction_factor);
    }else if(pid_result < 0){
        ledcWrite(3, 0);
        ledcWrite(2, -pid_result * reduction_factor);
    }else{
        return;
    }
}

void PWMClass::init(){
    ledcSetup(0, 30000, 8);
    ledcSetup(1, 30000, 8);
    ledcSetup(2, 30000, 8);
    ledcSetup(3, 30000, 8);

    ledcAttachPin(Left_pwm_A,0);
    ledcAttachPin(Left_pwm_B,1);
    ledcAttachPin(Right_pwm_A,2);
    ledcAttachPin(Right_pwm_B,3);
}
