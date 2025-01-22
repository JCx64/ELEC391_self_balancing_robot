// 根据Example说明，必须包含下面两个变量定义
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     3

#include "NRF52_MBED_TimerInterrupt.h"

//总共有三个Timer可选，NRF_TIMER_1,NRF_TIMER_3,NRF_TIMER_4，这次test使用的是timer3

#define TIMER0_INTERVAL_MS 2000  //中断触发的频率（per ms）

static bool toggle0 = false; //LED开关变量

NRF52_MBED_Timer ITimer0 (NRF_TIMER_3); //生成一个Timer对象，这里用ITimer0命名，0和所选的timer3无关

// 对应的中断回调函数
// 切记，中断回调里不能添加serial.print，会造成中断时序混乱
void TimerHandler0()
{
  //timer interrupt toggles pin LED_BUILTIN
  digitalWrite(LED_BUILTIN, toggle0);
  toggle0 = !toggle0;
}

void setup() {
 pinMode(LED_BUILTIN,  OUTPUT);
 ITimer0.attachInterruptInterval(TIMER0_INTERVAL_MS * 1000, TimerHandler0); //设置对应定时器的中断频率和中断回调函数
}

void loop() {
  // 主循环中的其他代码
}
