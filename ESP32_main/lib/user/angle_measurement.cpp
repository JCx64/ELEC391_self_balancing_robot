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

AngleClass :: AngleClass(){

}

void AngleClass::init()
{
    Wire.begin(I2C_SDA, I2C_SCL);
    mpu.begin(MPU6050_I2CADDR_DEFAULT, &Wire);
    mpu.setSampleRateDivisor(0);
    //mahony.Mahony_Init(333); // 333Hz
}

SimpleKalmanFilter kalmanX(1.f, 3.f, 0.01f);
SimpleKalmanFilter kalmanY(1.f, 3.f, 0.01f);
SimpleKalmanFilter kalmanZ(2.f, 2.f, 0.01f);

/**
 * @brief update angle
 */
void AngleClass::update(){
    mpu.getEvent(&acc, &gyro, &temp);

    w_yaw = gyro.gyro.y;

    float accAngleX = atan2(acc.acceleration.y, acc.acceleration.z) * 180 / PI; 
    float accelAngleY = atan2(-acc.acceleration.x, sqrt(acc.acceleration.y * acc.acceleration.y + acc.acceleration.z * acc.acceleration.z)) * 180.0 / PI;

    float gyroX = gyro.gyro.x * 180.0 / PI;
    float gyroY = gyro.gyro.y * 180.0 / PI;

    roll = kalmanX.updateEstimate(accAngleX + 0.3f * gyroX * 0.01f); // Row
    pitch = kalmanY.updateEstimate(accelAngleY + gyroY * 0.01f);  // Pitch
    // // mahony.Mahony_update(gyro.gyro.x,gyro.gyro.y,gyro.gyro.z,
    // //                 acc.acceleration.x,acc.acceleration.y,acc.acceleration.z,
    // //                 0,0,0);
    // // mahony.Mahony_computeAngles();
}

float AngleClass::get_row(){
    return roll;
}

float AngleClass::get_pitch(){
    return pitch;
}

float AngleClass::get_yaw(){
    return yaw;
}

float AngleClass::get_w_yaw()
{
    return w_yaw;
}
