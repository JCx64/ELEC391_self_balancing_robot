/*
@Team: ELEC391 B2
@Author: Jiayi Chen
@description: This is a test for Arduino Nano 33 BLE, using accelerometer and gyroscope to measure yaw angle of the board.
@Date: 2025/1/17
@Note: Tested algorithm is feasible to use, but may not used in final project.
*/

#include "Arduino_BMI270_BMM150.h"

float x, y, z, yaw_theta;
float pi = 3.141592653589793;

//Start serial port and IMU
void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    while (1);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
