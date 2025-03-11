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

class EncoderClass{
    private:
        int16_t pre_Encoder_Right;
        int16_t cur_Encoder_Right;
        int16_t pre_Encoder_Left;
        int16_t cur_Encoder_Left;
    public:
        EncoderClass();
        float get_Right_drpm();
        float get_Left_drpm();
        void update();
        void init();
};
#endif