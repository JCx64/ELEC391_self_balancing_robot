/**
 * @file arduino_macro.ino
 * @author Jiayi Chen
 * @brief test if set_pwm.h works
 * @date 2025/02/08
 *
 * @copyright UBC ELEC391 2024-2025 Winter Term 2 Team B2
 *
 */

#include <set_pwm.h>
#include <arduino.json>

class_pwm pwm_feeder;

void setup() {
    
}

void loop(){
    pwm_feeder.set_left_pwm(50);
    pwm_feeder.set_right_pwm(50);
}