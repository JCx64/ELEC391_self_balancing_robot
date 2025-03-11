#ifndef MYIIC_H
#define MYIIC_H

#include "Arduino.h"

//to be determined, see esp32
#define SDA_PIN  1  // SDA 引脚
#define SCL_PIN  2  // SCL 引脚

#define AS5600_I2C_ADDRESS 0x36
#define RAW_ANGLE_REGISTER 0x0C

class myiic{
    private:

    public:
        myiic();
        void I2C_Start();
        void I2C_Stop();
        void I2C_WriteBit(bool bit);
        bool I2C_ReadBit();
        bool I2C_WriteByte(uint8_t data);
        uint8_t I2C_ReadByte(bool ack);
        uint16_t readAS5600Angle();
};

#endif