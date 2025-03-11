/**
 * @file angle_measurement.h
 * @author Jiayi Chen
 * @brief all angle measurement fucntions
 * @date 2025/02/08
 *
 * @copyright UBC ELEC391 2024-2025 Winter Term 2 Team B2 (Jiayi Chen)
 *
 */

#ifndef COMP_FILTER.H
#define COMP_FILTER.H

#include "Arduino_BMI270_BMM150.h"

#define TIMER_INTERVAL_MS 2000
#define pi 3.141592653589793

struct ANGLE{
    float acc_angle;
    float gro_angle;
    float comp_filter_angle;
}angle;

class comp_filter
void get_acc_angle();
void get_gyro_angle();
void get_comp_filter_angle();

#endif