/*
@Team: ELEC391 B2
@Author: Jiayi Chen
@description: This is a test for Arduino Nano 33 BLE, using accelerometer and gyroscope to measure yaw angle of the board.
@Date: 2025/1/17
@Note: Tested algorithm is feasible to use, but may not used in final project.
*/

#include "Arduino_BMI270_BMM150.h"

//accelerometer parameters
float acc_x, acc_y, acc_z, tilt_angle_acc;
//gyroscope parameter
float gyro_x, gyro_y, gyro_z, tilt_angle_gyro, theta_zero=0;
//sample time
float sample_time;
unsigned long now_time, prev_time=0;
//complementary filter parameter
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
  //start timing
  now_time = millis();
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()){
    //read accelerometer and gyroscope data
    IMU.readAcceleration(acc_x, acc_y, acc_z);
    IMU.readGyroscope(gyro_x, gyro_y, gyro_z);

    //calculate tilt angle using accelerometer
    tilt_angle_acc = atan(acc_x/acc_y)*180/pi;

    //calculate tilt angle using gyroscope
    sample_time = (now_time - prev_time)/1000.0;
    //calculate yaw_angle based on sample_time and previous yaw_angle
    tilt_angle_gyro = theta_zero + z*sample_time;

    //applying complementary filter to tilt_angle_acc and tilt_angle_gyro to get precise tilt angle
    //update previous_time and theta_zero
    prev_time = now_time;
    theta_zero = yaw_angle;

  }
}
