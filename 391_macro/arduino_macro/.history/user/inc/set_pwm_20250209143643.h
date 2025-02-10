/**
 * @file set_pwm.h
 * @author Jiayi Chen
 * @brief Set left wheel and right wheel pwm (mapping to -100~100)
 * @date 2025/02/07
 *
 * @copyright UBC ELEC391 2024-2025 Winter Term 2 Team B2 (Jiayi Chen)
 *
 */

#ifndef SET_PWM.H
#define SET_PWM.H

#define pwm_pin1 5
#define pwm_pin2 6
#define pwm_pin3 9
#define pwm_pin4 10
#define reduction_factor 2.55 //mapping Arduino 0-255 pwm to 0-100 range

class class_pwm{
    private:

    public:
        class_pwm();
        void set_left_pwm(float pid_result);
        void set_right_pwm(float pid_result);
}

#endif