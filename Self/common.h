#ifndef COMMON_H
#define COMMON_H

//CubeMX 生成的头文件
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

// 用户代码头文件
#include "uart.h"
#include "motor.h"
#include "angle.h"
#include "pid.h"
#include <stdio.h>
#include "servo.h"

//
extern uint8_t DEBUG; // =1 开机即输出调试信息
extern uint32_t debug_last_tick; // 上次输出调试信息的时刻
extern uint8_t servo_open; // =1 允许操作舵机
//
void Init_Common(void);
void main_task(void);

#endif /* COMMON_H */
