/**
 * @file arduino_macro.ino
 * @author Jiayi Chen
 * @brief test if set_pwm.h works
 * @date 2025/02/08
 *
 * @copyright UBC ELEC391 2024-2025 Winter Term 2 Team B2
 *
 */

#include "set_pwm.h"
#include "pid_alg.h"
#include "NRF52_MBED_TimerInterrupt.h"

#define TIMER1_INTERVAL_MS 2000

NRF52_MBED_Timer ITimer1 (NRF_TIMER_1); 
PIDClass robot_pitch_PID(1,0,0,0,5,100,1);
PWMClass robotPWM();

float angle_pitch;
float motor_left_output, motor_right_output;

void TimerHandler1()
{
    float output_balance = robot_pitch_PID.pid_TimerElapsedCallback(angle_pitch);
    motor_right_output = output_balance;
    motor_left_output = output_balance;
}

void setup() {
    robot_pitch_PID.pid_setTarget(0);
    ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS, TimerHandler1);
}

void loop(){
    
}