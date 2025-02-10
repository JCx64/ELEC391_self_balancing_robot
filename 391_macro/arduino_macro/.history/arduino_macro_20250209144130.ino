/**
 * @file arduino_macro.ino
 * @author Jiayi Chen, Huiyu Chen, Jinke Su
 * @brief arduino code for ELEC391, self-balancing robot
 * @date 2025/02/08
 *
 * @copyright UBC ELEC391 2024-2025 Winter Term 2 Team B2
 *
 */

#include "user/inc/initialization.h"
#include "user/inc/angle_measurement.h"
#include "user/inc/pid_alg.h"
#include "user/inc/set_pwm.h"

void setup() {

  init_all();
  Serial.begin(9600);
  while (!Serial);
}

void loop(){
  get_acc_angle();
  get_gyro_angle();
  get_comp_filter_angle();
  Serial.print(angle.acc_angle);
  Serial.print(angle.gyro_angle);
  Serial.println(angle.comp_filter_angle);
}