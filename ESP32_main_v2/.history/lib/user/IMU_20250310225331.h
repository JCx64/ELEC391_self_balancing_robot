#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"

#define I2C_SDA GPIO_NUM_6
#define I2C_SCL GPIO_NUM_5
#define pi 3.1415926

class IMU{
    public :
        IMU();
        void begin(bool useDMP = false);
        void calibrate();
        void update();
        void getEulerAngles(float &roll, float &pitch, float &yaw);
        float getPitchAngularVelocity();
        float getXAcceleration();
        float getYawAngularVelocity();

    private:
        MPU6050 mpu;
        uint8_t fifoBuffer[64];
        Quaternion q;
        VectorFloat gravity;
        float ypr[3];

        float yaw_offset, pitch_offset, roll_offset;
        unsigned long lastTime;
        float dt;

        int16_t ax, ay, az, gx, gy, gz;
        float aax, aay, aaz, agx, agy, agz;
        long axo, ayo, azo, gxo, gyo, gzo;
        float gyrox, gyroy, gyroz;

        float AcceRatio;
        float GyroRatio;

        float Px, Rx, Kx, Sx;
        float Py, Ry, Ky, Sy;
        float Pz, Rz, Kz, Sz;
        bool useDMP;
};

#endif