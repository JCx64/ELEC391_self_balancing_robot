#include "encoder.h"

EncoderClass::EncoderClass(){

}

float EncoderClass::get_Left_drpm()
{
    float d_Encoder = cur_Encoder_Left - pre_Encoder_Left;
    if(abs(d_Encoder) > 1000)
    {
      if(d_Encoder > 0)
      d_Encoder -= 4095;
    else
      d_Encoder += 4095;
    }
    pre_Encoder_Left = cur_Encoder_Left;
    return d_Encoder * 60.f / (4095.f); // d_Encoder * 360 / 4095 -> w   rpm = w / 6
}

float EncoderClass::get_Right_drpm()
{
    float d_Encoder = cur_Encoder_Right - pre_Encoder_Right;
    if(abs(d_Encoder) > 1000)
    {
      if(d_Encoder > 0)
      d_Encoder -= 4095;
    else
      d_Encoder += 4095;
    }
    pre_Encoder_Right = cur_Encoder_Right;
    return d_Encoder * 60.f / (4095.f); // d_Encoder * 360 / 4095 -> w   rpm = w / 6
}

uint16_t readMagEncoderRight() {
  Wire1.requestFrom(MAG_ENCODER_ADDR, 2);

  if (Wire1.available() == 2){
    uint16_t highByte = Wire1.read(); // 读取高字节
    uint16_t lowByte = Wire1.read();  // 读取低字节
    return (highByte << 8) | lowByte;
  }
  else {
    return 0;
  }
} 

void EncoderClass::update()
{
    cur_Encoder_Right = readMagEncoderRight();
}

void EncoderClass::init()
{
    Wire1.begin(I2C_SDA1, I2C_SCL1, 400000);

    Wire1.beginTransmission(MAG_ENCODER_ADDR);
    Wire1.write(0x0E);
    Wire1.endTransmission(false);
}
