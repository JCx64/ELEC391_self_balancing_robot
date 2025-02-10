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
#include "user/inc/angle_measurement.h"

class_pid::class_pid(){

}

class_angle angle_update;
class_pid pid_feeder;

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
void class_pid::pid_TimerElapsedCallback(){
    //get needed angle measurements
    angle_update.get_acc_angle();
    angle_update.get_gyro_angle();
    angle_update.get_comp_filter_angle();

    //update error and sum error
    pid_feeder.angle_error = angle_target - angle.comp_filter_angle;
    pid_feeder.sum_error += pid_feeder.angle_error;

    //calculate output for kp
    pid_feeder.kp_out = pid_feeder.angle_error * pid.k_p;

    //calculate output for ki, consider integral limit
    if(abs(pid_feeder.ki_out) > pid.integral_max){
        if(pid_feeder.ki_out > 0){
            pid_feeder.ki_out = pid.integral_max;
        }else{
            pid_feeder.ki_out = -pid.integral_max;
        }
    }else{
        pid_feeder.ki_out = pid_feeder.sum_error * pid.k_i;
    }

    //calculate output for kd
    pid_feeder.kd_out = (pid_feeder.prev_error - pid_feeder.angle_error) * pid.k_d;

    //calculate pid output
    pid.output = pid_feeder.kp_out + pid_feeder.ki_out + pid_feeder.kd_out;

    //update previous error
    pid_feeder.prev_error = pid_feeder.angle_error;

    return;
}