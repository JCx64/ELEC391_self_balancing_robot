#include "servoControl.h"

ServoController::ServoController() {}

void ServoController::init() {
    ledcSetup(leftChannel, LEDC_FREQ, LEDC_RESOLUTION);
    ledcAttachPin(SERVO_LEFT_PIN, leftChannel);

    ledcSetup(rightChannel, LEDC_FREQ, LEDC_RESOLUTION);
    ledcAttachPin(SERVO_RIGHT_PIN, rightChannel);
}

void ServoController::setLeftServoAngle(int angle) {
    int duty = map(180-angle, 0, 180, 103, 511);  // 12-bit 分辨率
    ledcWrite(leftChannel, duty);
}

void ServoController::setRightServoAngle(int angle) {
    int duty = map(angle, 0, 180, 103, 511);  // 12-bit 分辨率
    ledcWrite(rightChannel, duty);
}
