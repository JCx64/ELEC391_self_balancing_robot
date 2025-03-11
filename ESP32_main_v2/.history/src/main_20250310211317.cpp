#include <Arduino.h>
#include "angle_measurement.h"
#include "set_pwm.h"
#include "pid_alg.h"
#include "esp_timer.h"
#include "HardwareSerial.h"
#include "stdint.h"
#include "encoder.h"
#include "myiic.h"
#include "IMU.h"

#define TIMER1_INTERVAL_MS 1.f
#define BALANCE_ANGLE 5.f

IMU imu;
EncoderClass robot_encoder;
PWMClass robot_pwm;
myiic myi2c;

PIDClass robot_velocity_Left_PID(0.3, 0.8, 0, 0, 30, 100, 0.2, 5.f/1000.f);
PIDClass robot_velocity_Right_PID(0.3, 0.8, 0, 0, 30, 100, 0.2, 5.f/1000.f);

HardwareSerial SerialPort(2);

unsigned long prev_time = 0.f;
String xbox_string = "";
char xbox_char = '\0';

float rpm_L, rpm_R;
float last_rpm_L, last_rpm_R;
int cur_Encoder_Left;
volatile int pre_Encoder_Left, pre_Encoder_Right;
bool sendRequest = false;

float roll,pitch,yaw;

float get_Left_drpm(int __cur_Encoder_Left, int __pre_Encoder_Left){
    float d_Encoder = __cur_Encoder_Left - __pre_Encoder_Left;
    if(abs(d_Encoder) > 2000)
    {
      if(d_Encoder > 0)
      d_Encoder -= 4095;
    else
      d_Encoder += 4095;
    }
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
    pre_Encoder_Right = __cur_Encoder_Right;
    return d_Encoder * 60.f / (4095.f); // d_Encoder * 360 / 4095 -> w   rpm = w / 6
}

void writeString_2(String stringData){
  for(int i = 0; i < stringData.length(); i++){
    Serial2.write(stringData[i]);
  }
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

void setup()
{
  Serial.begin(115200);
  // Serial2.begin(115200, SERIAL_8N1, 9, 10);

  imu.begin(false);
  robot_pwm.init();
  robot_encoder.init();

  //simulate two pin as i2c SCL and SDA
  pinMode(SIM_SDA_PIN, OUTPUT);
  pinMode(SIM_SCL_PIN, OUTPUT);
  digitalWrite(SIM_SDA_PIN, HIGH);
  digitalWrite(SIM_SCL_PIN, HIGH);
}

void loop()
{
  unsigned long current_time = millis();
  if(current_time - prev_time > 5){

    imu.update();
    imu.getEulerAngles(roll,pitch,yaw);

    //update right encoder using real i2c
    robot_encoder.update();

    //update left encoder using sim i2c
    uint16_t cur_Encoder_Left = myi2c.readAS5600Angle();
    // printf("%d, %d\n", cur_Encoder_Left, robot_encoder.cur_Encoder_Right);

    // receive xbox controller from uart2
    // while (Serial2.available()) {
    //   xbox_char = Serial2.read();

    //   if (xbox_char == '\n') {  
    //     writeString_2(xbox_string + "\n");  
    //     xbox_string = " ";        
    //   } else {
    //     xbox_string += xbox_char; 
    //   }
    // }
    
    rpm_L = get_Left_drpm(cur_Encoder_Left, pre_Encoder_Left)/0.005f;
    rpm_R = get_Right_drpm(robot_encoder.cur_Encoder_Right, pre_Encoder_Right)/0.005f;

    rpm_L = 0.25*rpm_L+(1-0.25)*last_rpm_L;
    rpm_R = 0.25*rpm_R+(1-0.25)*last_rpm_R;

    float output_balance = 18.f * (robot_angle.get_pitch()-BALANCE_ANGLE) + 16.f * robot_angle.get_w_yaw(); //d 10

    robot_velocity_Left_PID.pid_setTarget(output_balance);
    //robot_velocity_Left_PID.pid_setTarget(100.f);
    float motor_left_velocity = robot_velocity_Left_PID.pid_TimerElapsedCallback(rpm_L);

    robot_velocity_Right_PID.pid_setTarget(-output_balance);
    float motor_right_velocity = robot_velocity_Right_PID.pid_TimerElapsedCallback(rpm_R);

    robot_pwm.set_left_pwm(motor_left_velocity);
    robot_pwm.set_right_pwm(motor_right_velocity);

    //send message to vofa
    // float data[5];
    // //float a = cur_Encoder_Left;
    // //float b = pre_Encoder_Left;
    // data[0] = rpm_L;
    // data[1] = rpm_R;
    // data[2] = motor_left_velocity;
    // data[3] = motor_right_velocity;
    // data[4] = robot_angle.get_pitch();
    // uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};  // JustFloat 帧尾

    // // 发送 float 数据
    // Serial.write((uint8_t*)data, sizeof(data));

    // // 发送 JustFloat 帧尾
    // Serial.write(tail, sizeof(tail));
   
    //update prev values
    prev_time = current_time;
    last_rpm_L = rpm_L;
    last_rpm_R = rpm_R;
  }
}

  //self-designed communication protocol
  // if(sendRequest && Serial2.available() >= 2)
  // {
  //   uint8_t high = Serial2.read();
  //   uint8_t low = Serial2.read();
  //   cur_Encoder_Left = (high << 8) | low;
  //   sendRequest = false;
  //   Serial2.flush();
  // }
  // // 还没发送请求
  // else
  // {
  //   Serial2.println("REQ\n");
  //   sendRequest = true;
  //   Serial2.flush();
  // }
