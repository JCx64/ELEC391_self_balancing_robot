#include <Arduino.h>
#include "angle_measurement.h"
#include "set_pwm.h"
#include "pid_alg.h"
#include "esp_timer.h"
#include "encoder.h"
#include "HardwareSerial.h"
#include "esp_timer.h"

#define TIMER1_INTERVAL_MS 1.f
#define BALANCE_ANGLE 1.9f

AngleClass robot_angle;
EncoderClass robot_encoder;
PWMClass robot_pwm;

PIDClass robot_velocity_Left_PID(1.95, 0.9, 0.003, 0, 50, 100, 0.2, 5.f/1000.f);
PIDClass robot_velocity_Right_PID(1.95, 0.9, 0.003, 0, 50, 100, 0.2, 5.f/1000.f);

HardwareSerial SerialPort(2);

unsigned long previousMillis = 0.f;
String xbox_string = "";
char xbox_char = '\0';

SemaphoreHandle_t xIMUDataReady;
SemaphoreHandle_t xEncoderDataReady;

float rpm_L, rpm_R;

void vSensor_IMUReadTask(void *pvParameters) {
    while(1){
        robot_angle.update();
        xSemaphoreGive(xIMUDataReady);
        // 延时5ms (200Hz)
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

void vSensor_EncoderReadTask(void *pvParameters) {
  while(1){
    if (xSemaphoreTake(xIMUDataReady, portMAX_DELAY) == pdTRUE) {
    rpm_L = robot_encoder.get_Left_drpm()/0.005f;
    rpm_R = robot_encoder.get_Right_drpm()/0.005f;
     xSemaphoreGive(xEncoderDataReady);
    // 延时5ms (200Hz)
    vTaskDelay(5 / portTICK_PERIOD_MS);
    }
  }
}

void vMotorControlTask(void *pvParameters) {
    while(1) {
        if (xSemaphoreTake(xEncoderDataReady, portMAX_DELAY) == pdTRUE) {

          //目标RPM
          float output_balance = 3.f * (robot_angle.get_pitch()-BALANCE_ANGLE) + 0.01f * robot_angle.get_w_yaw();

          robot_velocity_Left_PID.pid_setTarget(output_balance);
          float motor_left_velocity = robot_velocity_Left_PID.pid_TimerElapsedCallback(rpm_L);

          robot_velocity_Right_PID.pid_setTarget(-output_balance);
          float motor_right_velocity = robot_velocity_Right_PID.pid_TimerElapsedCallback(rpm_R);

          robot_pwm.set_left_pwm(motor_left_velocity);
          robot_pwm.set_right_pwm(motor_right_velocity);
          // 延时5ms (200Hz)
          vTaskDelay(5 / portTICK_PERIOD_MS);
        }
    }
}

void writeString_2(String stringData){
  for(int i = 0; i < stringData.length(); i++){
    Serial2.write(stringData[i]);
  }
}

void task1(void* arg){
  robot_encoder.update();
  rpm_L = robot_encoder.get_Left_drpm()/0.005f;
  rpm_R = robot_encoder.get_Right_drpm()/0.005f;
  float output_balance = 1.f * (robot_angle.get_pitch()-BALANCE_ANGLE) + 0.01f * robot_angle.get_w_yaw();

  robot_velocity_Left_PID.pid_setTarget(output_balance);
  float motor_left_velocity = robot_velocity_Left_PID.pid_TimerElapsedCallback(rpm_L);

  robot_velocity_Right_PID.pid_setTarget(-output_balance);
  float motor_right_velocity = robot_velocity_Right_PID.pid_TimerElapsedCallback(rpm_R);

  robot_pwm.set_left_pwm(motor_left_velocity);
  robot_pwm.set_right_pwm(motor_right_velocity);
}

void setup()
{
  Serial2.begin(115200, SERIAL_8N1, 44, 43);

  robot_angle.init();
  robot_pwm.init();
  robot_encoder.init();

   // 创建定时器
  esp_timer_handle_t timer;
  esp_timer_create_args_t timer_args = {
      .callback = task1,   // 设置回调函数
      .arg = NULL,
      .name = "timer_example"
  };

  // 创建定时器句柄
  esp_timer_create(&timer_args, &timer);

  // 设置定时器的时间间隔（单位：微秒）
  esp_timer_start_periodic(timer, 5000);  // 每秒触发一次

  //xIMUDataReady = xSemaphoreCreateBinary();
  //xEncoderDataReady = xSemaphoreCreateBinary();
  //xTaskCreate(vSensor_IMUReadTask,"vSensor_IMUReadTask",2048,NULL,3,NULL);
  //xTaskCreate(vSensor_EncoderReadTask,"vSensor_EncoderReadTask",2048,NULL,2,NULL);
  //xTaskCreate(vMotorControlTask,"vMotorControlTask",2048,NULL,1,NULL);
  //vTaskStartScheduler();
}

void loop()
{
  robot_angle.update();
  robot_encoder.update();

  rpm_L = robot_encoder.get_Left_drpm()/0.005f;
  rpm_R = robot_encoder.get_Right_drpm()/0.005f;

  float output_balance = 1.f * (robot_angle.get_pitch()-BALANCE_ANGLE) + 0.01f * robot_angle.get_w_yaw();

  robot_velocity_Left_PID.pid_setTarget(output_balance);
  float motor_left_velocity = robot_velocity_Left_PID.pid_TimerElapsedCallback(rpm_L);

  robot_velocity_Right_PID.pid_setTarget(-output_balance);
  float motor_right_velocity = robot_velocity_Right_PID.pid_TimerElapsedCallback(rpm_R);

  robot_pwm.set_left_pwm(motor_left_velocity);
  robot_pwm.set_right_pwm(motor_right_velocity);
  // robot_angle.update();

  // receive xbox controller from uart2
  while (Serial2.available()) {
    xbox_char = Serial2.read();

    if (xbox_char == '\n') {  
      writeString_2(xbox_string + "\n");  
      xbox_string = " ";        
    } else {
      xbox_string += xbox_char; 
    }
  }

  Serial2.write("done");

  // float output_balance = 4.f * (robot_angle.get_pitch()-BALANCE_ANGLE) + 0.1f * robot_angle.get_w_yaw();
  // if(abs(output_balance) > 100.f){
  //   if(output_balance > 0.f){
  //     output_balance = 100.f;
  //   }
  //   else{
  //     output_balance = -100.f;
  //   }
  // }

  // float motor_right_output = output_balance;
  // float motor_left_output = output_balance;

  // printf("pitch: %f\n",output_balance*2.55f);

  // robot_pwm.set_left_pwm(motor_left_output);
  // robot_pwm.set_right_pwm(motor_right_output);
  //  unsigned long currentMillis = millis();
  // if (currentMillis - previousMillis >= 5) {
  //     robot_encoder.update();
  //      // 更新时间戳
  //     float w_R = robot_encoder.get_Right_drpm()/0.005f;
  //     float w_L = robot_encoder.get_Left_drpm()/0.005f;

  //     robot_velocity_Right_PID.pid_setTarget(-100.f);
  //     float output_R = robot_velocity_Right_PID.pid_TimerElapsedCallback(w_R); 

  //     robot_velocity_Left_PID.pid_setTarget(200.f);
  //     float output_L = robot_velocity_Left_PID.pid_TimerElapsedCallback(w_L);

  //     robot_pwm.set_right_pwm(output_R);
  //     robot_pwm.set_left_pwm(output_L);

  //     printf("w_L: %f w_R: %f t: %lu\n",w_L,w_R,(currentMillis - previousMillis));
  //     previousMillis = currentMillis; 
  //   }
  //printf("left err: %f right err: %f\n",robot_encoder.get_Left_drpm(),robot_encoder.get_Right_drpm());

}
