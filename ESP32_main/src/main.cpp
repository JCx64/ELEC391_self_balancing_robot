#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "SimpleKalmanFilter.h"

#define I2C_SDA GPIO_NUM_6 // Data line
#define I2C_SCL GPIO_NUM_5 // Clock line

Adafruit_MPU6050 mpu;
SimpleKalmanFilter kalmanX(2, 2, 0.01);
SimpleKalmanFilter kalmanY(2, 2, 0.01);

float angleX = 0, angleY = 0;

void setup()
{
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  mpu.begin(MPU6050_I2CADDR_DEFAULT, &Wire);

  printf("I2C scanning...\n");
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    int error = Wire.endTransmission();
    printf("Checking address 0x%02X, result: %d\n", address, error);
    if (error == 0) {
      printf("I2C device found at 0x%02X\n", address);
    }
  }
}

void loop()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accAngleX = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI; 
  float accelAngleY = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

  float gyroX = g.gyro.x * 180.0 / PI;
  float gyroY = g.gyro.y * 180.0 / PI;

  // Kalman filter
  angleX = kalmanX.updateEstimate(accAngleX + gyroX * 0.01); // Roll
  angleY = kalmanY.updateEstimate(accelAngleY + gyroY * 0.01);  // Pitch

  printf("X: %f, Y: %f\n", angleX, angleY);



  // printf("A: %f %f %f  ", a.acceleration.x, a.acceleration.y, a.acceleration.z);
  // printf("G: %f %f %f  ", g.gyro.x, g.gyro.y, g.gyro.z);
  // printf("T: %f\n", temp.temperature);

  // delay(100);

}
