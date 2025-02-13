/**
 * @file pid_alg.cpp
 * @author Jiayi Chen
 * @brief Basic PID algorithm
 * @date 2025/02/08
 *
 * @copyright UBC ELEC391 2024-2025 Winter Term 2 Team B2 (Jiayi Chen)
 *
 */

#include "pid_alg.h"
/**
 * @brief Initialize PID Class
 * 
 * @param p p coefficient
 * @param i i coefficient
 * @param d d coefficient
 * @param f feedforward coefficient
 * @param i_max maximum integral
 * @param o_max maximum output
 * @param deltaT time interval in second 
 * @return PIDClass 
 */
PIDClass :: PIDClass(float p, float i, float d, float f, float i_max, float o_max, float deltaT) {
    k_p = p;
    k_i = i;
    k_d = d;
    k_f = f;
    integral_max = i_max;
    output_max = o_max;
    timeInterval = deltaT;

    error_last = 0;
    integral_last = 0;
}

void PIDClass :: pid_setTarget(float t){
    target = t;
}

/**
 * @brief Timer elapsed call back function
 */
float PIDClass :: pid_TimerElapsedCallback(float input){
    float error = input - target;
    float output_p = error * k_p;

    integral_last += error * timeInterval;
    if(abs(integral_last) > integral_max){
        if(integral_last > 0){
            integral_last = integral_max;
        }
        else{
            integral_last = -integral_max;
        }
    }
    float output_i = integral_last*k_i;

    float output_d = k_d * (error - error_last) / timeInterval;
    error_last = error;
    
    float output = output_p + output_i + output_d;
    if(abs(output) > output_max){
        if(output > 0){
            output = output_max;
        }
        else{
            output = -output_max;
        }
    }
    return output;
}