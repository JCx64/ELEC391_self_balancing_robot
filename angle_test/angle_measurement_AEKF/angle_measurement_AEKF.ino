/*
@Team: ELEC391 B2
@Author: Huiyu Chen
@description: This is a test for Arduino Nano 33 BLE, using accelerometer and gyroscope to measure tilt angle of the board by Adaptive Extended Kalman Filter.
@Date: 2025/2/14
@Note: Tested algorithm is feasible to use, but may not used in final project.
*/

#include "Arduino_BMI270_BMM150.h"

// Accelerometer parameters
float acc_x, acc_y, acc_z, tilt_angle_acc;

// Gyroscope parameters
float gyro_x, gyro_y, gyro_z, tilt_angle_gyro, gyro_rate;

// Sample time parameters
unsigned long now_time, prev_time = 0;

// Kalman Filter parameters
float angle_est = 0.0; //Estimated tilt angle
float bias_est = 0.0;   //Estimated gyro bias
float P[2][2] = {{1.0, 0.0}, {0.0, 1.0}}; //Covariance matrix
const float Q_angle_offline = 0.032811; //Tilt angle process noise
const float Q_bias_offline = 0.051578;  //Gyro bias process noise
const float R_offline = 0.002952; //Measurement noise

// Adaptive Extended Kalman Filter parameters
const int window_size = 10; // Window size
float innovation[window_size]; // Innovation
int innovation_i = 0; // Index of innovation
float alpha_init = 0.4; // weighting factor
float beta_init = 0.3; // weighting factor
float dT_avg = 0.01; // Average sample time, need to be adjusted
int warmup_count = 0;
const int WARMUP_STEPS = 10; // 0.1s


// General parameters
const float pi = 3.141592653589793;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    while (1);
  }

  // Initialize the innovation array
  for (int i = 0; i < window_size; i++){
    innovation[i] = 0.0;
  }
}

void loop() {
    now_time = micros();
    float dt = (now_time - prev_time) / 1000000.0; // Convert sample time to sec
    prev_time = now_time;

    IMU.readAcceleration(acc_y, acc_x, acc_z);
    tilt_angle_acc = atan2(acc_x,acc_z) * 180 / pi;
    IMU.readGyroscope(gyro_x, gyro_y, gyro_z);

    gyro_rate = gyro_x - bias_est;


    // Prior estimation
    float damping =   0.05 * (angle_est - tilt_angle_acc); // Damping to reduce overshoot
    angle_est += dt * gyro_rate - damping; 
    // Prior estimated Cov. matrix
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle_offline);
    P[0][1] -= dt * P[1][1];
    P[1][0] = P[0][1];
    P[1][1] += Q_bias_offline * dt;

    // Innovation calculation
    float y = tilt_angle_acc - angle_est;

    /*Adaptive Extended Kalman Filter*/
    // Renew the innovation value
    innovation[innovation_i] = y;
    innovation_i = (innovation_i + 1) % window_size;

    // Calculate the Var. Matrix of innovation
    float C_rho = 0.0;
    for (int i = 0; i < window_size; i++){
      C_rho += innovation[i] * innovation[i];
    }

    C_rho /= window_size; // Average of the innovation

    // Adaptive Process 
    float H_P_HT = P[0][0]; // H = [1 0]
    float R_k = C_rho - H_P_HT; 
    R_k = max(R_k, 0.0); // R_k should be positive

    float Q_k = (C_rho > 0.0) ? sqrt(C_rho) * Q_angle_offline : Q_angle_offline; 

    float y_abs = abs(y);
    float alpha_adaptive = (y_abs / 15.0) * alpha_init; //Denominator need to be adjusted
    alpha_adaptive = constrain(alpha_adaptive, 0.0, 0.5); // Limit the range of alpha_adaptive

    float beta_adaptive = (dt / dT_avg) * beta_init; //Denominator need to be adjusted
    beta_adaptive = constrain(beta_adaptive, 0.0, 0.3);

    // Final R and Q
    float R_adaptive = (1 - alpha_adaptive) * R_offline + alpha_adaptive * R_k;
    float Q_angle_adaptive = (1 - beta_adaptive) * Q_angle_offline + beta_adaptive * Q_k;

    // In case of R_adaptive < 0
    if (R_adaptive < 0) {
      R_adaptive = R_offline;
      alpha_adaptive = 0.0;
    }

    float S_adaptive = P[0][0] + R_adaptive;
    float K0_adaptive = P[0][0] / S_adaptive;
    float K1_adaptive = P[1][0] / S_adaptive;

    angle_est += K0_adaptive * y;

    if (warmup_count < WARMUP_STEPS){
      warmup_count++;
    }else{
      bias_est += K1_adaptive * y;
      bias_est = constrain(bias_est, -2.0, 2.0); // Limit the range of bias_est
    }
    

    // Renew Cov. matrix, P = (I - Kk * H) * P_prior, H = [1 0]
    float P00_prior = P[0][0];
    float P01_prior = P[0][1];

    P[0][0] -= K0_adaptive * P00_prior;
    P[0][1] -= K0_adaptive * P01_prior;
    P[1][0] -= K1_adaptive * P00_prior;
    P[1][1] -= K1_adaptive * P01_prior;
    P[0][1] = P[1][0]; // Covariance matrix is symmetric

    Serial.println(angle_est);
    // Serial.println(IMU.accelerationSampleRate());
    // Serial.println(IMU.gyroscopeSampleRate());
    // Serial.println(dt);


}
