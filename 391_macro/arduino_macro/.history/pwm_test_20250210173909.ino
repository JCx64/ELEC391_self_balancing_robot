/**
 * @file pwm_test.ino
 * @author Jiayi Chen
 * @brief test if set_pwm.h works
 * @date 2025/02/08
 *
 * @copyright UBC ELEC391 2024-2025 Winter Term 2 Team B2
 *
 */

#include "./user/inc/set_pwm.h"

class_pwm pwm_feeder;

void setup() {
  Serial.begin(9600);
  while (!Serial);
}

void loop(){
    
}