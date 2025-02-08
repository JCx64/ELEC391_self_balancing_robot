/*
@Team: ELEC391 B2
@Author: Jiayi Chen
@description: This is a test for Arduino Nano 33 BLE, testing PWM
@Date: 2025/1/21
@Note: Only test for PWM, not for final project use.
*/

#define pwm_pin1 5
#define pwm_pin2 6
#define pwm_pin3 9
#define pwm_pin4 10

void setup() {
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
}

void loop(){
  analogWrite(5, 80);
  analogWrite(6, 0);
  analogWrite(9, 80);
  analogWrite(10, 0);
}