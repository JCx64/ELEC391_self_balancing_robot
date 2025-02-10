/**
 * @file pid_alg.cpp
 * @author Jiayi Chen
 * @brief Basic PID algorithm
 * @date 2025/02/08
 *
 * @copyright UBC ELEC391 2024-2025 Winter Term 2 Team B2 (Jiayi Chen)
 *
 */

#include "user/inc/pid_alg.h"
#include "user/inc/"
class_pid::class_pid(){

}


/**
 * @brief set k_p value
 *
 * @param _K_P proportional
 */
float class_pid::set_kp(float _K_P){
    pid.k_p = _K_P;
    return pid.k_p;
}

/**
 * @brief set k_i value
 *
 * @param _K_I integral
 */
float class_pid::set_ki(float _K_I){
    pid.k_i = _K_I;
    return pid.k_i;
}

/**
 * @brief set k_d value
 *
 * @param _K_D derivative
 */
float class_pid::set_kd(float _K_D){
    pid.k_d = _K_D;
    return pid.k_d;
}

/**
 * @brief set k_f value
 *
 * @param _K_F feedforward
 */
float class_pid::set_kf(float _K_F){
    pid.k_f = _K_F;
    return pid.k_f;
}

/**
 * @brief set maximum integral value
 *
 * @param _integral_max maximum interal value
 */
float class_pid::set_integral_max(float _integral_max){
    pid.integral_max = _integral_max;
    return pid.integral_max;
}

/**
 * @brief Timer elapsed call back function
 */
void pid_TimerElapsedCallback(){
    //get needed angle measurements
    get_acc_angle();
    get_gyro_angle();
    get_comp_filter_angle();

    //update error and sum error
    angle_error = angle_target - angle.comp_filter_angle;
    sum_error += angle_error;

    //calculate output for kp
    kp_out = angle_error * pid.k_p;

    //calculate output for ki, consider integral limit
    if(abs(ki_out) > pid.integral_max){
        if(ki_out > 0){
            ki_out = pid.integral_max;
        }else{
            ki_out = -pid.integral_max;
        }
    }else{
        ki_out = sum_error * pid.k_i;
    }

    //calculate output for kd
    kd_out = (prev_error - angle_error) * pid.k_d;

    //calculate pid output
    pid.output = kp_out + ki_out + kd_out;

    //update previous error
    prev_error = angle_error;

    return;
}