/*
@Team: ELEC391 B2
@Author: Jiayi Chen
@description: This is a test for Arduino Nano 33 BLE, using accelerometer and gyroscope to measure tilt angle of the board.
@Date: 2025/1/21
@Note: Tested algorithm is feasible to use, but may not used in final project.
*/

#include "Arduino_BMI270_BMM150.h"

//accelerometer parameters
float acc_x, acc_y, acc_z, tilt_angle_acc;
//gyroscope parameters
float gyro_x, gyro_y, gyro_z, tilt_angle_gyro, theta_zero=0;
//sample time parameters
float sample_time;
unsigned long now_time, prev_time=0;
//complementary filter parameters
float tilt_angle, prev_tilt_angle=0;
float k = 0.4; //gyroscope weight, can be modified accordingly, based on your condition
//general parameters
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
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()){ //check accelerometer and gyroscope ready
    //start timing
    now_time = millis();

    //read accelerometer and gyroscope data
    IMU.readAcceleration(acc_y, acc_x, acc_z);
    IMU.readGyroscope(gyro_x, gyro_y, gyro_z);

    //calculate tilt angle using accelerometer
    tilt_angle_acc = atan2(acc_x,acc_z)*180/pi;

    //calculate tilt angle using gyroscope
    sample_time = (now_time - prev_time)/1000.0; //convert sample time to sec
    tilt_angle_gyro = theta_zero + gyro_z*sample_time;

    //applying complementary filter to tilt_angle_acc and tilt_angle_gyro to get precise tilt angle
    tilt_angle = k*(prev_tilt_angle+gyro_z*sample_time)+(1-k)*tilt_angle_acc; //formula provided may be wrong, modified by J.C.

    //update previous_time and theta_zero
    prev_time = now_time;
    theta_zero = tilt_angle_gyro;
    prev_tilt_angle = tilt_angle;

    //send filtered tilt_angle (to be plotted)
    Serial.println(tilt_angle);

    delay(100);

  }
}
