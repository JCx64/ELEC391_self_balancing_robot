/*
@team: ELEC391 B2
@author: Jiayi Chen
@description: This is a test for Arduino Nano 33 BLE, using gyroscope to measure tilt angle of the board.
@date: 2025/1/17
@note: Tested algorithm not feasible for use.
*/

#include "Arduino_BMI270_BMM150.h"

float pi = 3.141592653589793;
float x, y, z, sample_time, tilt_angle, theta_zero=0;
unsigned long now_time, prev_time=0;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    while (1);
  }
}

void loop() {
  if (IMU.gyroscopeAvailable()) {
    now_time = millis();
    //get angular speed and sample rate/time
    IMU.readGyroscope(x, y, z);
    sample_time = (now_time - prev_time)/1000.0;

    //calculate tilt_angle based on sample_time and previous tilt_angle
    tilt_angle = theta_zero + z*sample_time;

    //send tilt angle (to be plotted)
    Serial.println(tilt_angle);

    Serial.print("x: ");
    Serial.println(x);
    Serial.print("y: ");
    Serial.println(y);
    Serial.print("z: ");
    Serial.println(z);
    
    //update previous_time and theta_zero
    prev_time = now_time;
    theta_zero = tilt_angle;
  }

  delay(500);
}