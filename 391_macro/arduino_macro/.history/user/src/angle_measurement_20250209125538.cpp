/**
 * @file angle_measurement.cpp
 * @author Jiayi Chen
 * @brief all angle measurement fucntions
 * @date 2025/02/08
 *
 * @copyright UBC ELEC391 2024-2025 Winter Term 2 Team B2 (Jiayi Chen)
 *
 */

#include "angle_measurement.h"

/**
 * @brief get angle calculated by accelerometer
 */
float class_angle::get_acc_angle(){
    float x, y, z;
    if(IMU.accelerationAvailable()){
        IMU.readAcceleration(y, x, z);
        angle.acc_angle = atan2(x,z)*180/pi;
    }
  return angle.acc_angle;
}

/**
 * @brief get angle calculated by gyroscope
 */
float class_angle::get_gyro_angle(){
    float x, y, z, theta_zero;
    if (IMU.gyroscopeAvailable()) {
        //get angular speed
        IMU.readGyroscope(x, y, z);

        //calculate tilt_angle based on sample_time and previous tilt_angle
        angle.gyro_angle = theta_zero + x * TIMER_INTERVAL_MS;

        //update theta zero
        theta_zero = angle.gro_angle;
    }
    return angle.gyro_angle;
}

/**
 * @brief get angle calculated by complementary filter
 */
void get_comp_filter_angle(){
    float acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
    float tilt_angle_acc, tilt_angle_gyro, theta_zero, prev_tilt_angle;
    float k;

    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()){ //check accelerometer and gyroscope ready
        //read accelerometer and gyroscope data
        IMU.readAcceleration(acc_y, acc_x, acc_z);
        IMU.readGyroscope(gyro_x, gyro_y, gyro_z);

        //calculate tilt angle using accelerometer
        tilt_angle_acc = atan2(acc_x,acc_z)*180/pi;

        //calculate tilt angle using gyroscope
        tilt_angle_gyro = theta_zero + gyro_x * TIMER1_INTERVAL_MS;

        //applying complementary filter to tilt_angle_acc and tilt_angle_gyro to get precise tilt angle
        angle.comp_filter_angle = k*(prev_tilt_angle+gyro_x * TIMER1_INTERVAL_MS)+(1-k)*tilt_angle_acc; //formula provided may be wrong, modified by J.C.

        //update theta_zero and previoud comp_tilt_angle
        theta_zero = tilt_angle_gyro;
        prev_tilt_angle = angle.comp_filter_angle;
    }
    return;
}