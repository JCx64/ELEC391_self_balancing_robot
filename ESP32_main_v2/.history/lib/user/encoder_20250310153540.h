#ifndef ENCODER_H
#define ECNODER_H

#include "stdint.h"
#include "math.h"
#include "Arduino.h"
#include "Wire.h"

// 右侧电机
#define I2C_SDA1 GPIO_NUM_3 // Data line1
#define I2C_SCL1 GPIO_NUM_8 // Clock line1

#define MAG_ENCODER_ADDR 0x36
#define RAW_ANGLE_REGISTER 0x0C

class EncoderClass{
    private:

    public:
        EncoderClass();
        float get_Right_drpm();
        void update();
        void init();
        int16_t pre_Encoder_Right = 0;
        int16_t cur_Encoder_Right;
};

#endif