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
#include "Adafruit_NeoPixel.h"
#include "servoControl.h"

// Struct Definition
struct __controlParameter{
  float p_v; //速度环的P
  float i_v; //速度环的I
  float p_b; //直立环的P
  float d_b; //直立环的D
  float offset_b; //直立环的位移补偿
  float balanceAngle; //机器的平衡角度
  float servoAngle; //当前站立角度
  float rollAngle; //压弯角度
  float moveSpeed_forward; //前进时的目标速度
  float moveSpeed_backward; //后退时的目标速度
  float moveAngle_forward; //前进时的前倾角度
  float moveAngle_backward; //后退时的后仰角度
  float rotateSpeed; //转向的PWM输出
};
__controlParameter ZeroAngle = {
    .p_v = 160.5,
    .i_v = 0.6,
    .p_b = 80,
    .d_b = 340,
    .offset_b = 10,
    .balanceAngle = -2.3,
    .servoAngle = 0,
    .rollAngle = 10,
    .moveSpeed_forward = 1.5,
    .moveSpeed_backward = -0.003,
    .moveAngle_forward = -2.3,
    .moveAngle_backward = 0,
    .rotateSpeed = 30
};

__controlParameter FifteenAngle = {
    .p_v = 160.5,
    .i_v = 0.6,
    .p_b = 85,
    .d_b = 345,
    .offset_b = 70,
    .balanceAngle = -4.0,
    .servoAngle = 15,
    .rollAngle = 10,
    .moveSpeed_forward = 0.8,
    .moveSpeed_backward = -0.003,
    .moveAngle_forward = -2.3,
    .moveAngle_backward = -0.5,
    .rotateSpeed = 30
};

__controlParameter *controlPara = &ZeroAngle;

enum __controlState{
 ZERO_ANGLE,
 FIFTEEN_ANGLE
};
__controlState controlState;

// Class Initialization
IMU imu;
EncoderClass robot_encoder;
PWMClass robot_pwm;
myiic myi2c;
Adafruit_NeoPixel strip(1, 48, NEO_GRB + NEO_KHZ800);
HardwareSerial SerialPort(2);
ServoController robot_servo;
XboxSeriesXControllerESP32_asukiaaa::Core
    xboxController("f4:6a:d7:8d:e7:a6");

// PID Initialization
PIDClass robot_velocity_Left_PID(0.3, 0.8, 0, 0, 30, 100, 0.2, 5.f/1000.f);
PIDClass robot_velocity_Right_PID(0.3, 0.8, 0, 0, 30, 100, 0.2, 5.f/1000.f);
//105 //0.15   、、、、0.060  p<80 小车会站不住，一直往一个方向运动
PIDClass robot_velocity_PID(controlPara->p_v, controlPara->i_v, 0, 0, 500, 1000, 0.2, 5.f/1000.f); 

// Variable Initialization
float  BALANCE_ANGLE, static_BalanceAngle;
float rpm_L, rpm_R;
int cur_Encoder_Left;
float last_rpm_L, last_rpm_R;
float last_roll;
volatile int pre_Encoder_Left, pre_Encoder_Right;
float moveSpeed;
float output_rotate;
bool sendRequest = false;
bool skipVelocity_rotate, skipVelocity_move = false;
float roll,pitch,yaw;
uint8_t green = 255;
String responseBuffer = ""; //用于接收数据
uint16_t angleLeft = 0;
unsigned long prev_time_200hz = 0;
unsigned long prev_time_20hz = 0;
unsigned long prev_time_1hz = 0;
uint16_t input_move;
uint8_t input_rotate_L,input_rotate_R;
bool input_button_L, input_button_R;
bool isPressed = false;
bool prev_button_L, prev_button_R = false;
int balanceServoAngle_L,balanceServoAngle_R;

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

// Function Statement
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

String readString_2(String stringData){
  char charData = '\0';
  while(Serial2.read()!='\n')
  {
    stringData += charData;
    charData = Serial2.read();
  }
  return stringData+"\n";
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

void checkControlState(__controlState cs){
  switch (cs)
  {
  case ZERO_ANGLE:
    controlPara = &ZeroAngle;
    robot_velocity_PID.pid_setP(controlPara->p_v);
    robot_velocity_PID.pid_setI(controlPara->i_v);
    BALANCE_ANGLE = controlPara->balanceAngle;
    static_BalanceAngle = BALANCE_ANGLE;
    balanceServoAngle_L = controlPara->servoAngle;
    balanceServoAngle_R = controlPara->servoAngle;
    break;
  case FIFTEEN_ANGLE:
    controlPara = &FifteenAngle;
    robot_velocity_PID.pid_setP(controlPara->p_v);
    robot_velocity_PID.pid_setI(controlPara->i_v);
    BALANCE_ANGLE = controlPara->balanceAngle;
    static_BalanceAngle = BALANCE_ANGLE;
    balanceServoAngle_L = controlPara->servoAngle;
    balanceServoAngle_R = controlPara->servoAngle;
    break;
  default:
    break;
  }
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
  strip.begin();          // 初始化
  strip.show();           // 清空灯光
  strip.setBrightness(50); // 可调亮度 (0-255)


  //Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 9, 10);
  //人脸识别
  String uart2String;
  // while (true) {
  //   Serial.print("等待接收\n");
  //   if (Serial2.available()) {
  //       uart2String = Serial2.readStringUntil('\n'); // 读取 UART2 数据
  //       uart2String.trim();  // 去除首尾空格或换行符
  //       Serial.print("收到: ");
  //       Serial.println(uart2String);
  //       if (uart2String == "sjk") {
  //           Serial.println("识别到 'sjk'，继续执行...");
  //           break; // 跳出循环，进入主程序
  //       }
  //   }
  // }
  Serial2.end();
  //人脸识别结束后 亮红灯
  strip.setPixelColor(0, strip.Color(255, 0, 0));
  strip.show();  

  xboxController.begin();
  while(!xboxController.isConnected()){
    xboxController.onLoop();
    delay(500);
  }
  //接收到xbox按钮信号后，亮蓝灯
  strip.setPixelColor(0, strip.Color(0, 0, 255));
  strip.show();  

  //system start
  robot_pwm.init();
  robot_encoder.init();
  robot_servo.init();
  moveSpeed = 0.f;
  output_rotate = 0.f;
  BALANCE_ANGLE = controlPara->balanceAngle;
  static_BalanceAngle = BALANCE_ANGLE;
  controlState = ZERO_ANGLE;

  //simulate two pin as i2c SCL and SDA
  pinMode(SIM_SDA_PIN, OUTPUT);
  pinMode(SIM_SCL_PIN, OUTPUT);
  digitalWrite(SIM_SDA_PIN, HIGH);
  digitalWrite(SIM_SCL_PIN, HIGH);
  robot_servo.setLeftServoAngle(controlPara->servoAngle);
  robot_servo.setRightServoAngle(controlPara->servoAngle);
  delay(4000);
  // 红灯闪两下提示IMU准备初始化
  bool flash = true;
  for(int i = 0;i<3;i++){
    int light = flash? 255:0;
    strip.setPixelColor(0, strip.Color(light, 0, 0));
    strip.show();  
    flash = !flash;
    delay(500);
  }

  // 开始初始化亮蓝灯
  strip.setPixelColor(0, strip.Color(0, 0, 255));
  strip.show();  
  imu.begin(false);
  // IMU初始化完毕 关灯
  strip.setPixelColor(0, strip.Color(0, 0, 0));
  strip.show(); 

  delay(2000);
  strip.setPixelColor(0, strip.Color(0, 255, 0));
  strip.show();
}

void loop()
{
  unsigned long current_time = micros();

  if(current_time - prev_time_1hz >= 100000){
    // 机器不水平
    if(abs(roll) > 3){
      // 机器没在转向
      if(!skipVelocity_rotate){
        // 左边低右边
        if(roll > 0){
          // 如果之前是右边腿高
          if(balanceServoAngle_R != controlPara->servoAngle){
            // 把右边腿收回来
            balanceServoAngle_R = controlPara->servoAngle;
          }
          // 如果左边腿低
          else{
             // 从转向压弯状态恢复
            if(balanceServoAngle_L < controlPara->servoAngle)
              balanceServoAngle_L = controlPara->servoAngle; 
            else
              balanceServoAngle_L = balanceServoAngle_L + 1;
          }
        }

        // 右边低左边高
        else{
          // 如果之前是左边腿高
          if(balanceServoAngle_L != controlPara->servoAngle){
            // 把左边腿收回来
            balanceServoAngle_L = controlPara->servoAngle;
          }
          //如果右边腿低
          else{
            // 从转向压弯状态恢复
            if(balanceServoAngle_R < controlPara->servoAngle)
              balanceServoAngle_R = controlPara->servoAngle; 
            else
              balanceServoAngle_R += 1;
          }
        }
        robot_servo.setRightServoAngle(balanceServoAngle_R);
        robot_servo.setLeftServoAngle(balanceServoAngle_L);
      }
    }
    printf("Roll Angle = %f L:= %d R:= %d\n",roll,balanceServoAngle_L,balanceServoAngle_R);
    prev_time_1hz = current_time;
  }
  else if(current_time - prev_time_20hz >= 50000){
    xboxController.onLoop();
    if(xboxController.isConnected()){
      input_move = xboxController.xboxNotif.joyLVert;
      input_rotate_L = xboxController.xboxNotif.trigLT;
      input_rotate_R = xboxController.xboxNotif.trigRT;
      input_button_L = xboxController.xboxNotif.btnLB;
      input_button_R = xboxController.xboxNotif.btnRB;
      trigger_vibration_press_ctrl();
      //---机器高度控制---//
      //左右没有同时按下
      if(!input_button_L || !input_button_R)
      {
        //左键状态改变
        if(input_button_L != prev_button_L){
          //如果之前是按下了，那现在就是松开
          if(isPressed){
            isPressed = false;
            controlState = ZERO_ANGLE;
            checkControlState(controlState);
            robot_servo.setLeftServoAngle(controlPara->servoAngle);
            robot_servo.setRightServoAngle(controlPara->servoAngle);
          }
          //如果之前是松开，现在是按下
          else{
            isPressed = true;
          }
        }
        prev_button_L = input_button_L;

        //右键状态改变
        if(input_button_R != prev_button_R){
          //如果之前是按下了，那现在就是松开
          if(isPressed){
            isPressed = false;
            controlState = FIFTEEN_ANGLE;
            checkControlState(controlState);
            robot_servo.setLeftServoAngle(controlPara->servoAngle);
            robot_servo.setRightServoAngle(controlPara->servoAngle);
          }
          //如果之前是松开，现在是按下
          else{
            isPressed = true;
          }
        }
        prev_button_R = input_button_R;
      }
      //---机器转向控制---//
      // 左右同时按下
      if(input_rotate_R != 0 && input_rotate_L !=0){
        skipVelocity_rotate = false;
        if(abs(output_rotate) > 0.01f)
          output_rotate = output_rotate + 0.5*(0.f - output_rotate);
      }
      // 右边按下
      else if(input_rotate_R != 0){
        output_rotate = -controlPara->rotateSpeed;
        if(controlState == ZERO_ANGLE){
          balanceServoAngle_L = controlPara->servoAngle + controlPara->rollAngle;
          robot_servo.setLeftServoAngle(balanceServoAngle_L);
        }
        else{
          balanceServoAngle_R = controlPara->servoAngle - controlPara->rollAngle;
          robot_servo.setRightServoAngle(balanceServoAngle_R);
        }
        skipVelocity_rotate = true;
      }
      // 左边按下
      else if(input_rotate_L != 0){
        output_rotate = controlPara->rotateSpeed;
        if(controlState == ZERO_ANGLE){
          balanceServoAngle_R = controlPara->servoAngle + controlPara->rollAngle;
          robot_servo.setRightServoAngle(balanceServoAngle_R);
        }
        else{
          balanceServoAngle_L = controlPara->servoAngle - controlPara->rollAngle;
          robot_servo.setLeftServoAngle(balanceServoAngle_L);
        }
        skipVelocity_rotate = true;
      }
      // 两边都不按
      else{
        skipVelocity_rotate = false;
        if(abs(output_rotate) > 0.01f)
          output_rotate = output_rotate + 0.5*(0.f - output_rotate);
      }
      //---机器转向操作---//
      // 后退
      if(input_move > 45000){ 
        moveSpeed = controlPara->moveSpeed_backward;
        BALANCE_ANGLE = controlPara->moveAngle_backward;
        skipVelocity_move = true;
      }
      // 前进
      else if (input_move <25000){
        moveSpeed = controlPara->moveSpeed_forward;
        //存在转向操作
        if(skipVelocity_rotate)
          moveSpeed = moveSpeed / 2;
        else
          skipVelocity_move = true;
      }
      // 没油门
      else{
        skipVelocity_move = false;
        if(abs(moveSpeed) > 0.01f)
          moveSpeed = moveSpeed - 0.1* moveSpeed;
        if(BALANCE_ANGLE - static_BalanceAngle > 0.01f)
          BALANCE_ANGLE = BALANCE_ANGLE + 0.5*(static_BalanceAngle - BALANCE_ANGLE);
      }
    }
    prev_time_20hz = current_time;
  }
  else if(current_time - prev_time_200hz >= 5000){
    imu.update();
    imu.getEulerAngles(roll,pitch,yaw);
    roll = 0.125*roll+(1-0.125)*last_roll;
    robot_encoder.update();
    uint16_t cur_Encoder_Left = myi2c.readAS5600Angle();
    
    rpm_L = get_Left_drpm(cur_Encoder_Left, pre_Encoder_Left)/0.005f;
    rpm_R = get_Right_drpm(robot_encoder.cur_Encoder_Right, pre_Encoder_Right)/0.005f;

    // EMWA, used to smooth the rpm value
    rpm_L = 0.125*rpm_L+(1-0.125)*last_rpm_L; 
    rpm_R = 0.125*rpm_R+(1-0.125)*last_rpm_R;

    float angle_error = pitch-BALANCE_ANGLE;
    float angle_velocity = imu.getPitchAngularVelocity();
    if(abs(angle_error)<0.3f)
      angle_error = 0;
    if(abs(angle_velocity)<0.08f)
      angle_velocity = 0;
    float output_balance = 0.6f * (controlPara->p_b * angle_error + controlPara->d_b * angle_velocity) + controlPara->offset_b; //78 220

    float output_velocity = 0.f;
    if(skipVelocity_move || skipVelocity_rotate)
      robot_velocity_PID.pid_seperateI();
    float velocity_Forward = 3.1415926f * 0.08f * (rpm_L - rpm_R) / 30.f;
    robot_velocity_PID.pid_setTarget(moveSpeed);
    output_velocity = robot_velocity_PID.pid_TimerElapsedCallback(velocity_Forward);

    robot_velocity_Left_PID.pid_setTarget(-output_balance+output_velocity);
    float motor_left_velocity = robot_velocity_Left_PID.pid_TimerElapsedCallback(rpm_L);
    robot_velocity_Right_PID.pid_setTarget(output_balance-output_velocity);
    float motor_right_velocity = robot_velocity_Right_PID.pid_TimerElapsedCallback(rpm_R);

    robot_pwm.set_left_pwm(motor_left_velocity+output_rotate);
    robot_pwm.set_right_pwm(motor_right_velocity+output_rotate);

    //send message to vofa
    float data[3];
    data[0] = motor_left_velocity;
    data[1] = motor_right_velocity;
    data[2] = pitch;
    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};  // JustFloat 帧尾

    //发送 float 数据
    //Serial.write((uint8_t*)data, sizeof(data));
    //发送 JustFloat 帧尾
    //Serial.write(tail, sizeof(tail));

    // printf("Left: %f Right: %f Angle: %f\n",rpm_L,rpm_R,pitch); 
    //printf("time = %f Roll Angle = %f L:= %d R:= %d\n", (float)(current_time-prev_time_200hz),roll,balanceServoAngle_L,balanceServoAngle_R);
   
    //update prev values
    prev_time_200hz = current_time;
    last_rpm_L = rpm_L;
    last_rpm_R = rpm_R;
    last_roll = roll;
  }
}