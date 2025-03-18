#include <Arduino.h>
#include "set_pwm.h"
#include "pid_alg.h"
#include "esp_timer.h"
#include "HardwareSerial.h"
#include "stdint.h"
#include "encoder.h"
#include "myiic.h"
#include "IMU.h"
#include "XboxSeriesXControllerESP32_asukiaaa.hpp"

#define TIMER1_INTERVAL_MS 1.f
#define BALANCE_ANGLE -1.9f //-3.117 // -1.81
IMU imu;
EncoderClass robot_encoder;
PWMClass robot_pwm;
myiic myi2c;
XboxSeriesXControllerESP32_asukiaaa::Core
    xboxController("f4:6a:d7:8d:e7:a6");

// Define x_box controller button string to be sent to esp32-s3
String xbox_string()
{
  String str = String(xboxController.xboxNotif.btnY) + "," + // Y: 0/1 bool
               String(xboxController.xboxNotif.btnX) + "," + // X: 0/1 bool
               String(xboxController.xboxNotif.btnB) + "," + // B: 0/1 bool
               String(xboxController.xboxNotif.btnA) + "," + // A: 0/1 bool
               String(xboxController.xboxNotif.btnLB) + "," + // Left_Button: 0/1 bool
               String(xboxController.xboxNotif.btnRB) + "," + // Right_Button: 0/1 bool
               String(xboxController.xboxNotif.btnSelect) + "," + // Window_button: 0/1 (中左) bool
               String(xboxController.xboxNotif.btnStart) + "," + // Menue_button: 0/1 (中右) bool
               String(xboxController.xboxNotif.btnXbox) + "," + // Xbox_button: 0/1 (中上) bool
               String(xboxController.xboxNotif.btnShare) + "," + // Uplaod_button: 0/1 (中下) bool
               String(xboxController.xboxNotif.btnLS) + "," + // Left_joystick_push: 0/1 bool
               String(xboxController.xboxNotif.btnRS) + "," + // Right_joystie_push: 0/1 bool
               String(xboxController.xboxNotif.btnDirUp) + "," + // Up_direction: 0/1 方向键上 bool
               String(xboxController.xboxNotif.btnDirRight) + "," + // Right_direction: 0/1 方向键右 bool
               String(xboxController.xboxNotif.btnDirDown) + "," + // Down_direction: 0/1 方向键下 bool
               String(xboxController.xboxNotif.btnDirLeft) + "," + // Left_direction: 0/1 方向键左 bool
               String(xboxController.xboxNotif.joyLHori) + "," + // Left_JS_Horizontal: 0-32990-65535 (too sensitive, need consider deadzone) uint16_t
               String(xboxController.xboxNotif.joyLVert) + "," + // Left_JS_Vertical: 0-32630-65535 (too sensitive, need consider deadzone) uint16_t
               String(xboxController.xboxNotif.joyRHori) + "," + // Right_JS_Horizontal: 0-32990-65535 (too sensitive, need consider deadzone) uint16_t
               String(xboxController.xboxNotif.joyRVert) + "," + // Right_JS_Vertical: 0-32630-65535 (too sensitive, need consider deadzone) uint16_t
               String(xboxController.xboxNotif.trigLT) + "," + // Left_trigger: 0-1023 uint16_t
               String(xboxController.xboxNotif.trigRT) + "\n"; // Right_trigger: 0-1023 uint16_t
  return str;
};

PIDClass robot_velocity_Left_PID(0.3, 0.8, 0, 0, 30, 100, 0.2, 5.f/1000.f);
PIDClass robot_velocity_Right_PID(0.3, 0.8, 0, 0, 30, 100, 0.2, 5.f/1000.f);
PIDClass robot_velocity_PID(107, 0.060, 0, 0, 800, 1000, 0.2, 5.f/1000.f); //105 //0.15   、、、、0.060
// p<80 小车会站不住，一直往一个方向运动
PIDClass robot_rotate_PID(0, 0, 0, 0, 50, 100, 0.2, 5.f/1000.f); //0.5

HardwareSerial SerialPort(2);

float rpm_L, rpm_R;
float last_rpm_L, last_rpm_R;
int cur_Encoder_Left;
volatile int pre_Encoder_Left, pre_Encoder_Right;
bool sendRequest = false;

float roll,pitch,yaw;

//Xbox variables
String responseBuffer = ""; //用于接收数据
uint16_t angleLeft = 0;

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

// Xbox functions below
/*
支持四种振动模式
left：上左电机动
right：上右电机动
center：下左电机和下右电机一起动，频率高力量小
shake：下左电机和下右电机一起动，频率低力量大

测试结果：
四种模式都可以调振动力度
下左电机和下右电机是绑定的，只能一起动，但是提供了两种振动模式，个人猜测是两种模式的原理是给电机不同的电压
可以随意搭配使用，但center和shake一起用的话执行的应该是shake
*/

// 配置参考
// repo.v.select.center = 0;
// repo.v.select.left = 0;
// repo.v.select.right = 0;
// repo.v.select.shake = 0;
// repo.v.power.center = 0; // x% power
// repo.v.power.left = 0;
// repo.v.power.right = 30;
// repo.v.power.shake = 0;
// repo.v.timeActive = 0; // 振动 x/100 秒，最大2.56秒(uint8_t)
// repo.v.timeSilent = 0;   // 静止 x/100 秒
// repo.v.countRepeat = 0;  // 循环次数 x+1

// 官方例程 后期加extra featrue可以看这个 选择不同振动模式
void demoVibration()
{
  XboxSeriesXHIDReportBuilder_asukiaaa::ReportBase repo;
  Serial.println("full power for 1 sec");
  xboxController.writeHIDReport(repo);
  delay(2000);

  repo.v.select.center = true;
  repo.v.select.left = false;
  repo.v.select.right = false;
  repo.v.select.shake = false;
  repo.v.power.center = 30; // 30% power
  repo.v.timeActive = 50;   // 0.5 second
  Serial.println("run center 30\% power in half second");
  xboxController.writeHIDReport(repo);
  delay(2000);

  repo.v.select.center = false;
  repo.v.select.left = true;
  repo.v.power.left = 30;
  Serial.println("run left 30\% power in half second");
  xboxController.writeHIDReport(repo);
  delay(2000);

  repo.v.select.left = false;
  repo.v.select.right = true;
  repo.v.power.right = 30;
  Serial.println("run right 30\% power in half second");
  xboxController.writeHIDReport(repo);
  delay(2000);

  repo.v.select.right = false;
  repo.v.select.shake = true;
  repo.v.power.shake = 30;
  Serial.println("run shake 30\% power in half second");
  xboxController.writeHIDReport(repo);
  delay(2000);

  repo.v.select.shake = false;
  repo.v.select.center = true;
  repo.v.power.center = 50;
  repo.v.timeActive = 20;
  repo.v.timeSilent = 20;
  repo.v.countRepeat = 2;
  Serial.println("run center 50\% power in 0.2 sec 3 times");
  xboxController.writeHIDReport(repo);
  delay(2000);
}

// 振动反馈，根据扳机按压力度调整振动力度 **正在使用 左右扳机会根据按压力度调整震动力度**
void trigger_vibration_press_ctrl()
{
  XboxSeriesXHIDReportBuilder_asukiaaa::ReportBase repo;
  static uint16_t TrigMax = XboxControllerNotificationParser::maxTrig;
  String str_1;
  repo.setAllOff();
  repo.v.select.left = true;
  repo.v.select.right = true;
  repo.v.power.left = (uint8_t)((float)xboxController.xboxNotif.trigLT / (float)TrigMax * 100);
  repo.v.power.right = (uint8_t)((float)xboxController.xboxNotif.trigRT / (float)TrigMax * 100);
  repo.v.timeActive = 50;

  // // activate center mode 中心震动，力度小，频率高
  // repo.v.select.center = true;
  // repo.v.power.center = 50;

  // // activate shake mode 对角震动，力度大，频率低
  // repo.v.select.shake = true;
  // repo.v.power.shake = 60;
  xboxController.writeHIDReport(repo);
  str_1 = String(repo.v.power.left) + "," + String(repo.v.power.right) + "\n";

  //Serial.print(str_1);
  //delay(50);
}

void setup()
{
  Serial.begin(115200);
  // Serial2.begin(115200, SERIAL_8N1, 9, 10);

  imu.begin(false);
  robot_pwm.init();
  robot_encoder.init();
  xboxController.begin();

  //simulate two pin as i2c SCL and SDA
  pinMode(SIM_SDA_PIN, OUTPUT);
  pinMode(SIM_SCL_PIN, OUTPUT);
  digitalWrite(SIM_SDA_PIN, HIGH);
  digitalWrite(SIM_SCL_PIN, HIGH);
  
  delay(2000); 
}

unsigned long prev_time_200hz = 0;
unsigned long prev_time_20hz = 0;

uint16_t input_move;

void loop()
{
  unsigned long current_time = micros();

  if(current_time - prev_time_20hz >= 50000){
    if(xboxController.isConnected()){
      if (xboxController.isWaitingForFirstNotification())
      {
        Serial.println("waiting for first notification");
      }
      else
      {
        // printf("%s\n", xbox_string().c_str());
        input_move = xboxController.xboxNotif.joyLVert;
        //demoVibration();
        trigger_vibration_press_ctrl();
      }
    }
    else{
      xboxController.onLoop();
      // printf("Xbox connecting......\n");
    }
    prev_time_20hz = current_time;
  }
  else if(current_time - prev_time_200hz >= 5000){
    imu.update();
    imu.getEulerAngles(roll,pitch,yaw);

    //update right encoder using real i2c
    robot_encoder.update();

    //update left encoder using sim i2c
    uint16_t cur_Encoder_Left = myi2c.readAS5600Angle();
    
    rpm_L = get_Left_drpm(cur_Encoder_Left, pre_Encoder_Left)/0.005f;
    rpm_R = get_Right_drpm(robot_encoder.cur_Encoder_Right, pre_Encoder_Right)/0.005f;

    // EMWA, used to smooth the rpm value
    rpm_L = 0.125*rpm_L+(1-0.125)*last_rpm_L;
    rpm_R = 0.125*rpm_R+(1-0.125)*last_rpm_R;

    float angle_error = pitch-BALANCE_ANGLE;
    if(abs(angle_error)<0.1f){
      angle_error = 0;
    }
    float output_balance = 0.6f * (48.f * angle_error + 160.f * imu.getPitchAngularVelocity()); //48 158


    robot_rotate_PID.pid_setTarget(0);
    float output_rotate = robot_rotate_PID.pid_TimerElapsedCallback(imu.getYawAngularVelocity());

    float velocity_Forward = 3.1415926f * 0.08f * (rpm_L - rpm_R) / 30.f;
    float moveSpeed = 0;
    if(input_move > 35000){
      moveSpeed = -0.3;
    }
    else if (input_move <31000){
      moveSpeed = 0.3;
    }
    robot_velocity_PID.pid_setTarget(moveSpeed);
    float output_velocity = robot_velocity_PID.pid_TimerElapsedCallback(velocity_Forward);

    robot_velocity_Left_PID.pid_setTarget(-output_balance + output_velocity - output_rotate);
    float motor_left_velocity = robot_velocity_Left_PID.pid_TimerElapsedCallback(rpm_L);

    robot_velocity_Right_PID.pid_setTarget(output_balance - output_velocity + output_rotate);
    float motor_right_velocity = robot_velocity_Right_PID.pid_TimerElapsedCallback(rpm_R);

    robot_pwm.set_left_pwm(motor_left_velocity);
    robot_pwm.set_right_pwm(motor_right_velocity);

    //send message to vofa
    float data[4];
    data[0] = -output_balance + output_velocity - output_rotate;
    data[1] = cur_Encoder_Left;
    data[2] = output_balance - output_velocity + output_rotate;
    data[3] = rpm_R;
    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};  // JustFloat 帧尾

    //发送 float 数据
    //Serial.write((uint8_t*)data, sizeof(data));
    //发送 JustFloat 帧尾
    //Serial.write(tail, sizeof(tail));

    printf("Left: %f Right: %f Angle: %f\n",rpm_L,rpm_R,pitch); 
    // printf("%lu\n", current_time - prev_time_200hz);
   
    //update prev values
    prev_time_200hz = current_time;
    last_rpm_L = rpm_L;
    last_rpm_R = rpm_R;
  }
}