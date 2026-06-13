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

//
void Init_Common(void);
void main_task(void);

#endif /* COMMON_H */
