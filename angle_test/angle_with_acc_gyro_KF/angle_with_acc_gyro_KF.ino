/*
@Team: ELEC391 B2
@Author: Jiayi Chen, Huiyu Chen
@description: This is a test for Arduino Nano 33 BLE, using accelerometer and gyroscope to measure tilt angle of the board by Kalman Filter.
@Date: 2025/2/3
@Note: Tested algorithm is feasible to use, but may not used in final project.
*/

#include "Arduino_BMI270_BMM150.h"

//accelerometer parameters
float acc_x, acc_y, acc_z, tilt_angle_acc;
//gyroscope parameters
float gyro_x, gyro_y, gyro_z, tilt_angle_gyro, gyro_rate;
//sample time parameters
unsigned long now_time, prev_time=0;
//Kalman Filter parameters
float angle_est = 0.0; //Estimated tilt angle
float bias_est = 0.0;   //Estimated gyro bias
float P[2][2] = {{1.0, 0.0}, {0.0, 1.0}}; //Covariance matrix
const float Q_angle = 0.001; //Tilt angle process noise
const float Q_bias = 0.003;  //Gyro bias process noise
const float R_measure = 0.03; //Measurement noise
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
    float sample_time = (now_time - prev_time) / 1000.0; //convert sample time to sec
    prev_time = now_time;

    //read accelerometer and calculate tilt angle using accelerometer
    IMU.readAcceleration(acc_y, acc_x, acc_z);
    tilt_angle_acc = atan2(acc_x,acc_z) * 180 / pi;

    //read gyroscope and calculate tilt angle using gyroscope
    IMU.readGyroscope(gyro_x, gyro_y, gyro_z);

    // Kalman Filter: Prediction
    // Prior Estimation, x_prior = A * x_prev + B * u
     gyro_rate = gyro_x - bias_est; // (theta_k - theta_k-1) / sample_time = gyro_rate = gyro_x - bias_est
     angle_est += gyro_rate * sample_time; // Calculate tilt angle estimation

     // Prior estimated Cov. matrix
      P[0][0] += sample_time * (sample_time * P[1][1] - P[0][1] - P[1][0] + Q_angle);
      P[0][1] -= sample_time * P[1][1];
      P[1][0] = P[0][1];
      P[1][1] += Q_bias * sample_time;

    // Kalman Filter: Correction
    //Kalman Gain, Kk = (P_prior * H) / (H * P_prior * H' + R)
      float S = P[0][0] + R_measure; //Innovation covariance, denominator
      float K0 = {P[0][0] / S}; //Kalman gain for angle
      float K1 = {P[1][0] / S}; //Kalman gain for bias
    
    //Posterior Estimation, x_posterior = x_prior + Kk * (z - H * x_prior)
      float y = tilt_angle_acc - angle_est; //Innovation, measurement - estimation, H = [1 0]
      angle_est += K0 * y; //Update tilt angle estimation
      bias_est += K1 * y; //Update gyro bias estimation

    //Renew Cov. matrix, P = (I - Kk * H) * P_prior, H = [1 0]
      float P00_prior = P[0][0];
      float P01_prior = P[0][1];

      P[0][0] -= K0 * P00_prior;
      P[0][1] -= K0 * P01_prior;
      P[1][0] -= K1 * P00_prior;
      P[1][1] -= K1 * P01_prior;
      P[0][1] = P[1][0]; // Covariance matrix is symmetric




    //send filtered tilt_angle (to be plotted)
    Serial.println(angle_est);

    delay(100);

  }
}
