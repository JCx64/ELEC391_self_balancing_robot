#include "Arduino_BMI270_BMM150.h"

float x, y, z, yaw_theta, roll_theta, pitch_theta;
float pi = 3.141592653589793;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    while (1);
  }

  // Serial.print("Accelerometer sample rate = ");
  // Serial.print(IMU.accelerationSampleRate());
  // Serial.println(" Hz");
}

void loop() {
  float x, y, z, yaw_theta, roll_theta, pitch_theta;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(y, x, z);
  }
  Serial.print("x acc is:");
  Serial.print(x);
  Serial.print("  y acc is:");
  Serial.print(y);
  Serial.print("  z acc is:");
  Serial.println(z);

  yaw_theta = atan(x/y)*180/pi;
  roll_theta = atan(z/x)*180/pi;
  pitch_theta = atan(z/y)*180/pi;
  Serial.print("yaw angle is:");
  Serial.println(yaw_theta);
  Serial.print("roll angle is:");
  Serial.println(roll_theta);
  Serial.print("pitch angle is:");
  Serial.println(pitch_theta);
  Serial.println();

  delay(500);
}
