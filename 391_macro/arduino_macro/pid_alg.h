/**
 * @file pid_alg.h
 * @author Jiayi Chen
 * @brief Basic PID algorithm
 * @date 2025/02/08
 *
 * @copyright UBC ELEC391 2024-2025 Winter Term 2 Team B2 (Jiayi Chen)
 *
 */

#ifndef PID_ALG_H
#define PID_ALG_H

#include "Arduino.h"

class PIDClass{
    public:
        PIDClass(float p, float i, float d, float f, float i_max, float o_max, float deltaT);
        float pid_TimerElapsedCallback(float input);
        void pid_setTarget(float t);
    private:
        int k_p;
        int k_i;
        int k_d;
        int k_f;

        float integral_max;
        float output_max;

        float timeInterval;
        float target;

        float error_last;
        float integral_last;
};

#endif