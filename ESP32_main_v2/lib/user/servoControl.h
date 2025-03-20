#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>

#define SERVO_LEFT_PIN 17   // 左舵机 GPIO
#define SERVO_RIGHT_PIN 18  // 右舵机 GPIO
#define LEDC_FREQ 50  // 50Hz 适用于舵机
#define LEDC_RESOLUTION 12  // 12-bit 分辨率 4095

class ServoController {
public:
    ServoController();
    void begin();
    void setLeftServoAngle(int angle);
    void setRightServoAngle(int angle);
    
private:
    int leftChannel = 1;  // ESP32 LEDC 通道
    int rightChannel = 0;
};

#endif