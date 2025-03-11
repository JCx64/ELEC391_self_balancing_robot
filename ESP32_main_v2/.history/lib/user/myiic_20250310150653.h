#ifndef MYIIC_H
#define MYIIC_H

#define SDA_PIN  1  // SDA 引脚
#define SCL_PIN  2  // SCL 引脚

#define AS5600_I2C_ADDRESS 0x36
#define RAW_ANGLE_REGISTER 0x0C

class myiic{
    private:

    public:
        myi2c();
};

#endif