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
  angle_update.get_acc_angle();
  angle_update.get_gyro_angle();
  angle_update.get_comp_filter_angle();
  Serial.print(angle.acc_angle);
  Serial.print(angle.gyro_angle);
  Serial.println(angle.comp_filter_angle);
}