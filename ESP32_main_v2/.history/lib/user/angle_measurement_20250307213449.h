/**
 * @file angle_measurement.h
 * @author Jiayi Chen
 * @brief all angle measurement fucntions
 * @date 2025/02/08
 *
 * @copyright UBC ELEC391 2024-2025 Winter Term 2 Team B2 (Jiayi Chen)
 *
 */

#ifndef ANGLE_MEASUREMENT_H
#define ANGLE_MEASUREMENT_H

#include "Arduino.h"
#include <HardwareSerial.h>
#include "Adafruit_MPU6050.h"
#include <Adafruit_Sensor.h>
#include "SimpleKalmanFilter.h"
#include <Wire.h>
#include "MahonyAHRS.h"

#define pi 3.14159f
#define I2C_SDA GPIO_NUM_6 // Data line
#define I2C_SCL GPIO_NUM_5 // Clock line

class AngleClass{
    private:
        Adafruit_MPU6050 mpu;
        MahonyClass mahony;
        sensors_event_t acc, gyro, temp;
        volatile float roll, pitch, yaw;
        volatile float w_yaw;
    public:
        AngleClass();
        float get_row();
        float get_pitch();
        float get_yaw();
        float get_w_yaw();

        void update();
        
        void init();
};

#endif