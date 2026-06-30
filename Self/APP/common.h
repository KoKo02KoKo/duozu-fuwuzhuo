#ifndef COMMON_H
#define COMMON_H

//CubeMX 生成的头文件
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

// 用户代码头文件
#include "uart.h"
#include "../BSP/motor.h"
#include "motion.h"
#include "angle.h"
#include "pid.h"
#include <stdio.h>
#include "servo.h"
#include "face_track.h"
#include "radar.h"
#include "xiaozhi.h"
#include "obstacle.h"


//
extern uint8_t DEBUG; // =1 开机即输出调试信息
extern uint32_t debug_last_tick; // 上次输出调试信息的时刻
extern uint8_t servo_mode; // =1 允许操作舵机
extern uint8_t motor_mode; // =1 允许操作电机
extern uint8_t xiaozhi_enable ;
extern uint8_t obstacle_enable ;//
//
void Init_Common(void);
void main_task(void);

#endif /* COMMON_H */
