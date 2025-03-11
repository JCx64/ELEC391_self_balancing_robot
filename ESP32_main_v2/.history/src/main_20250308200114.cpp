#include <Arduino.h>
#include "angle_measurement.h"
#include "set_pwm.h"
#include "pid_alg.h"
#include "esp_timer.h"
#include "encoder.h"
#include "HardwareSerial.h"
#include "stdint.h"

#define TIMER1_INTERVAL_MS 1.f
#define BALANCE_ANGLE 1.9f

AngleClass robot_angle;
EncoderClass robot_encoder;
PWMClass robot_pwm;

PIDClass robot_velocity_Left_PID(0.3, 0, 0, 0, 50, 100, 0.2, 5.f/1000.f);
PIDClass robot_velocity_Right_PID(1.95, 0.9, 0.003, 0, 50, 100, 0.2, 5.f/1000.f);

HardwareSerial SerialPort(2);

unsigned long previousMillis = 0.f;
String xbox_string = "";
char xbox_char = '\0';
String i2c_str = "";
char i2c_char = '\0';

float rpm_L, rpm_R;
int cur_Encoder_Left, cur_Encoder_Right;
int pre_Encoder_Left, pre_Encoder_Right;

float get_Left_drpm(int __cur_Encoder_Left, int __pre_Encoder_Left){
    float d_Encoder = __cur_Encoder_Left - __pre_Encoder_Left;
    if(abs(d_Encoder) > 2000)
    {
      if(d_Encoder > 0)
      d_Encoder -= 4095;
    else
      d_Encoder += 4095;
    }
    if(abs(d_Encoder)<10)
      d_Encoder = 0.f;
    pre_Encoder_Left = __cur_Encoder_Left;
    return d_Encoder * 60.f / (4095.f); // d_Encoder * 360 / 4095 -> w   rpm = w / 6
}

float get_Right_drpm(int __cur_Encoder_Right, int __pre_Encoder_Right){
    float d_Encoder = __cur_Encoder_Right - __pre_Encoder_Right;
    if(abs(d_Encoder) > 2000)
    {
      if(d_Encoder > 0)
      d_Encoder -= 4095;
    else
      d_Encoder += 4095;
    }
    if(abs(d_Encoder)<10)
      d_Encoder = 0.f;
    pre_Encoder_Right = __cur_Encoder_Right;
    return d_Encoder * 60.f / (4095.f); // d_Encoder * 360 / 4095 -> w   rpm = w / 6
}

void writeString_2(String stringData){
  for(int i = 0; i < stringData.length(); i++){
    Serial2.write(stringData[i]);
  }
}

void setup()
{
  Serial.begin(9600);
  Serial2.begin(115200, SERIAL_8N1, 44, 43);

  robot_angle.init();
  robot_pwm.init();
  robot_encoder.init();
}

void swapEndian(float *f) {
    uint8_t *p = (uint8_t *)f;
    uint8_t temp[4];
    temp[0] = p[3];
    temp[1] = p[2];
    temp[2] = p[1];
    temp[3] = p[0];
    *f = *(float *)temp;
}

void loop()
{
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis >= 5){
    robot_angle.update();
    robot_encoder.update();

      while (Serial2.available()) {
      // xbox_char = Serial2.read();

      // if (xbox_char == '\n') {  
      //   writeString_2(xbox_string + "\n");  
      //   xbox_string = " ";        
      // } else {
      //   xbox_string += xbox_char; 
      // }
      i2c_char = Serial2.read(); 
      if(i2c_char == '\n'){
        cur_Encoder_Left = i2c_str.toInt();
        i2c_str = "";
      }else{
        i2c_str += i2c_char;
      }
    }
    
    // receive xbox controller from uart2
    rpm_L = get_Left_drpm(cur_Encoder_Left, pre_Encoder_Left)/0.005f;
    rpm_R = get_Right_drpm(robot_encoder.cur_Encoder_Right, pre_Encoder_Right)/0.005f;

    float output_balance = 1.f * (robot_angle.get_pitch()-BALANCE_ANGLE) + 0.001f * robot_angle.get_w_yaw();

    robot_velocity_Left_PID.pid_setTarget(output_balance);
    float motor_left_velocity = robot_velocity_Left_PID.pid_TimerElapsedCallback(rpm_L);

    robot_velocity_Right_PID.pid_setTarget(-output_balance);
    float motor_right_velocity = robot_velocity_Right_PID.pid_TimerElapsedCallback(rpm_R);

    robot_pwm.set_left_pwm(motor_left_velocity);
    //robot_pwm.set_right_pwm(-motor_left_velocity);

    float data[3];
    //float a = cur_Encoder_Left;
    //float b = pre_Encoder_Left;
    data[0] = rpm_L;
    data[1] = output_balance;
    data[2] = motor_left_velocity;
    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};  // JustFloat 帧尾

    // 发送 float 数据
    Serial.write((uint8_t*)data, sizeof(data));

    // 发送 JustFloat 帧尾
    Serial.write(tail, sizeof(tail));

    //printf("left_encoder:%d t:%lu\n",cur_Encoder_Left,(currentMillis - previousMillis));
    previousMillis = currentMillis;
  }
}
