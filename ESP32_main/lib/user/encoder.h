#ifndef ENCODER_H
#define ECNODER_H

#include "stdint.h"
#include "math.h"
#include "Arduino.h"
#include "Wire.h"

// 左侧电机
#define I2C_SDA GPIO_NUM_6 // Data line
#define I2C_SCL GPIO_NUM_5 // Clock line

// 右侧电机
#define I2C_SDA1 GPIO_NUM_3 // Data line1
#define I2C_SCL1 GPIO_NUM_8 // Clock line1

#define MAG_ENCODER_ADDR 0x36

class EncoderClass{
    private:
        int16_t pre_Encoder_Left;
        int16_t pre_Encoder_Right;

        int16_t cur_Encoder_Left;
        int16_t cur_Encoder_Right;
    public:
        EncoderClass();
        float get_Left_drpm();
        float get_Right_drpm();
        void update();
        void init();
};
#endif