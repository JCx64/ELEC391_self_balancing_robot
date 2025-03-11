#ifndef MYIIC_H
#define SET_PWM_H

#define reduction_factor 2.55 //mapping Arduino 0-255 pwm to 0-100 range

#include "Arduino.h"

class PWMClass{
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