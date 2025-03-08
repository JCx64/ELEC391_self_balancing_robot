/*
@Team: ELEC391 B2
@Author: Huiyu Chen
@description: Measure the Var. and Cov. (Q and R) for KF using simplified Allan's method.
@Date: 2025/2/6
@Note: Tested algorithm is feasible to use, but may not used in final project.
*/

#include "Arduino_BMI270_BMM150.h"

const int sample_time = 30; //sample time in sec
const int sample_rate1 = 1600; // 100 Hz
const int sample_rate2 = 1600; // 100 Hz
const int num_samples1 = sample_time * sample_rate1; //number of samples for 100 Hz
const int num_samples2 = sample_time * sample_rate2; //number of samples for 100 Hz

// Parameters
float gyro_x[num_samples1];
float gyro_bias[num_samples1];
float acc_angle[num_samples2];

// Gyro Variables
float gyro_mean = 0.0;
float gyro_var_raw = 0.0;
float gyro_var_bias = 0.0;

// Acc Variables
float acc_mean = 0.0;
float acc_var = 0.0;

// Constants
const float pi = 3.141592653589793;

void measureNoise_gyro(){
  float sum_raw, sum_square_raw, sum_bias, sum_square_bias = 0.0;
  float bias_est = 0.0;
  float bias_mean = 0.0;
  float gx, gy, gz = 0.0;
  float alpha = 0.98;

  for(int i = 0; i < num_samples1; i++){
    if (IMU.gyroscopeAvailable()){
      IMU.readGyroscope(gx, gy, gz);
      gyro_x[i] = gx;

      bias_est = alpha * bias_est + (1 - alpha) * gx; // low-pass filter
      gyro_bias[i] = bias_est;
      
      sum_raw += gx;
      sum_square_raw += gx * gx;
      sum_bias += bias_est;
      sum_square_bias += bias_est * bias_est;

      delay (1000 / sample_rate1);
    }
  }


  gyro_mean = sum_raw / num_samples1;
  gyro_var_raw = (sum_square_raw / num_samples1) - gyro_mean * gyro_mean; // Var = E[X^2] - E[X]^2

  bias_mean = sum_bias / num_samples1;
  gyro_var_bias = (sum_square_bias / num_samples1) - bias_mean * bias_mean; // Var = E[X^2] - E[X]^2

}

void measureNoise_acc(){
  float sum, sum_square = 0.0;
  float ax, ay, az = 0.0;

  for(int i = 0; i < num_samples2; i++){
    if (IMU.accelerationAvailable()){
      IMU.readAcceleration(ax, ay, az);
      acc_angle[i] = atan2(ax, az) * 180 / pi;

      sum += acc_angle[i];
      sum_square += acc_angle[i] * acc_angle[i];

      delay (1000 / sample_rate2);
    }
  }

  acc_mean = sum / num_samples2;
  acc_var = (sum_square / num_samples2) - acc_mean * acc_mean; // Var = E[X^2] - E[X]^2

}

void Getresults(){

  const float R = acc_var;
  const float window_size = sample_time / 2.0;
  const float Q_angle = gyro_var_raw * window_size;
  const float Q_bias = gyro_var_bias * window_size * window_size;

  Serial.print("Q_angle: "); Serial.println(Q_angle, 6);
  Serial.print("Q_bias: "); Serial.println(Q_bias, 6);
  Serial.print("R: "); Serial.println(R, 6);
}

void setup() {
  Serial.begin(9600);
    while (!Serial);

  if (!IMU.begin()) {
    while (1);
  }

  delay(1000); // wait until the equipment is stable

  measureNoise_gyro();
  measureNoise_acc();
  Getresults();

}

void loop() {
  // put your main code here, to run repeatedly:

}
