/*
@team: ELEC391 B2
@author: Jiayi Chen
@description: This is a test for Arduino Nano 33 BLE, using accelerometer to measure yaw angle of the board.
@date: 2025/1/17
@note: Only measure yaw angle at specific posture, tested algorithm not feasible for use. Measured xy are inverse to what implies on the board.
*/

#include "Arduino_BMI270_BMM150.h"

float x, y, z, tilt_angle;
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
//Read IMU measurements
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(y, x, z);
  }

//Calculate and display yaw angle (to be plotted)
  tilt_angle = atan2(x,y)*180/pi;
  Serial.println(tilt_angle);

  delay(50);
}