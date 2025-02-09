/**
 * @file arduino_macro.ino
 * @author Jiayi Chen, Huiyu Chen, Jinke Su
 * @brief arduino code for ELEC391, self-balancing robot
 * @date 2025/02/08
 *
 * @copyright UBC ELEC391 2024-2025 Winter Term 2 Team B2
 *
 */

#include "angle_measurement.h"
#include "set_pwm.h"
#include "pid_alg.h"

void setup() {
    
}

void loop(){
    set_left_pwm(50);
    set_right_pwm(50);
    delay(50);
}