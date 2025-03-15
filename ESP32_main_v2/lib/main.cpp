#include <Arduino.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include <HardwareSerial.h>
#include <encoder.h>

HardwareSerial SerialPort(2);
EncoderClass robot_encoder;

// Required to replace with your xbox address
// 需要在此替换成自己的手柄蓝牙MAC地址
XboxSeriesXControllerESP32_asukiaaa::Core
    xboxController("f4:6a:d7:8d:e7:a6");

// char angle_str[20];

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

// 官方例程
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

// 振动反馈，根据扳机按压力度调整振动力度
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

  Serial.print(str_1);
  delay(50);
}

void writeString_2(String stringData) 
{ 
  for (int i = 0; i < stringData.length(); i++)
  {
    Serial2.write(stringData[i]);  
  }
}

bool parseResponseFrame(String frame, uint16_t &angle){
  if (!frame.startsWith("$")) return false;
  int starIdx = frame.indexOf("*");
  if (starIdx == -1) return false;

  // 提取数据部分
  String dataPart = frame.substring(1, starIdx);
  // 提取校验码
  String checksumStr = frame.substring(starIdx + 1);
  checksumStr.trim();

  uint8_t checksum = 0;
  for (int i = 1; i < frame.length(); i++) {
    char c = frame.charAt(i);
    if (c == '*') break;
    checksum ^= c;
  }

  uint8_t recvChecksum = strtol(checksumStr.c_str(), NULL, 16);
  if (checksum != recvChecksum) return false;

  // 提取数据部分
  angle = dataPart.toInt();
  return true;
  
}


void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 13, 14); //RX:44 TX:43 Baudrate:115200
  robot_encoder.init(); //initialize i2c
  xboxController.begin();

  // Serial.println("Scanning I2C devices...");
  // for (uint8_t address = 1; address < 127; address++) {
  //   Wire.beginTransmission(address);
  //   if (Wire.endTransmission() == 0) {
  //     Serial.print("I2C device found at address 0x");
  //     Serial.println(address, HEX);
  //   }
  // }
}

String responseBuffer = ""; //用于接收数据

uint16_t angleLeft = 0;

void loop()
{
  robot_encoder.update();

  // // read xbox controller data
  // xboxController.onLoop();
  // if (xboxController.isConnected())
  // {
  //   if (xboxController.isWaitingForFirstNotification())
  //   {
  //     Serial.println("waiting for first notification");
  //   }
  //   else
  //   {
  //     // printf("%s\n", xbox_string().c_str());
  //     //demoVibration();
  //     trigger_vibration_press_ctrl();
  //   }
  // }
  // else
  // {
  //   // Serial.println("not connected");
  //   // printf("0\n");
  //   // if (xboxController.getCountFailedConnection() > 2)
  //   // {
  //   //   ESP.restart();
  //   // }
  // }

  // read left encoder data
  float rpm_L = robot_encoder.get_Left_drpm()/0.005f;

  //send message through uart2
  if(Serial2.available()){
    // writeString_2(xbox_string().c_str());
    uint16_t angle = robot_encoder.readMagEncoderLeft();
    // printf("%d\n", angle);
    // writeString_2(String(rpm_L)+"\n");
    writeString_2(String(angle)+"\n");
    
    // clear send buffer, ready for next transmmission
    Serial2.flush();
  }

//  if(Serial2.available()){
//     String req = Serial2.readStringUntil('\n'); //读取一行命令
//     req.trim(); //去除空格
//     if(req == "REQ"){//若收到请求
//       //读取并计算编码器数据
//       uint16_t angle = robot_encoder.readMagEncoderLeft();
      
//       //构造响应帧
//       //String response = constructResponseFrame(angle);
//       //Serial2.write(response.c_str()); //发送响应帧

//       uint8_t high = angle >> 8;
//       Serial2.write(high);
//       uint8_t low = angle;
//       Serial2.write(low);

//       Serial2.flush(); //清空串口缓冲区
//     }
//   }
}
