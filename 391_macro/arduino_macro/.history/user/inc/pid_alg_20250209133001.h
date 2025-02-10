/**
 * @file pid_alg.h
 * @author Jiayi Chen
 * @brief Basic PID algorithm
 * @date 2025/02/08
 *
 * @copyright UBC ELEC391 2024-2025 Winter Term 2 Team B2 (Jiayi Chen)
 *
 */

#ifndef PID_ALG.H
#define PID_ALG.H

#include "angle_measurement.h"

#define angle_target 0

// PID parameter structure
struct PID_params{
    float k_p;
    float k_i;
    float k_d;
    float k_f;
    float integral_max;
    float output;
} pid;

class class_pid{
    private:
        float angle_error, prev_error = 0, sum_error;
        float kp_out, ki_out, kd_out;

    public:
        void set_kp();
        void set_ki();
        void set_kd();
        void set_kf();
        void set_integral_max();
        void pid_TimerElapsedCallback();
}

#endif