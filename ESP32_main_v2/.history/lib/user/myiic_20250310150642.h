#ifndef MYIIC_H
#define MYIIC_H

#define SDA_PIN  1  // SDA 引脚
#define SCL_PIN  2  // SCL 引脚

#define AS5600_I2C_ADDRESS 0x36
#define RAW_ANGLE_REGISTER 0x0C

class myiic{
    private:
        int Left_pwm_A = 15;
        int Left_pwm_B = 16;
        int Right_pwm_A = 17; 
        int Right_pwm_B  = 18;
    public:
        PWMClass();
        void set_left_pwm(float pid_result);
        void set_right_pwm(float pid_result);
        void init();
};

#endif