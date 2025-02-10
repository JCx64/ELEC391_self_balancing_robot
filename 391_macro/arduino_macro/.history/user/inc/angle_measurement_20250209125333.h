/**
 * @file angle_measurement.h
 * @author Jiayi Chen
 * @brief all angle measurement fucntions
 * @date 2025/02/08
 *
 * @copyright UBC ELEC391 2024-2025 Winter Term 2 Team B2 (Jiayi Chen)
 *
 */

#ifndef ANGLE_MEASUREMENT.H
#define ANGLE_MEASUREMENT.H

#include "Arduino_BMI270_BMM150.h"

#define pi 3.141592653589793

struct ANGLE{
    float acc_angle;
    float gyro_angle;
    float comp_filter_angle;
}angle;

class class_angle{
    private:

    public:
        ANGLE_MEASUREMENT();
        float get_acc_angle();
        float get_gyro_angle();
        float get_comp_filter_angle();
}

#endif