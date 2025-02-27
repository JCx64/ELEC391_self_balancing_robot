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

void set_left_pwm(float pid_result);
void set_right_pwm(float pid_result);

#endif