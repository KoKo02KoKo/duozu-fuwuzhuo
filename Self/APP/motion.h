#ifndef _MOTION_H
#define _MOTION_H

#include <stdint.h>
#include "pid.h"
#include "../BSP/motor.h"

typedef struct Motion_Controller_ Motion_Controller;


extern Motion_Controller motion;
extern PID_TypeDef yaw_pid;  // 偏航角 PID 实例（带角度环绕)
extern MOTOR ml; // 定义一个 MOTOR 实例
extern MOTOR mr; // 定义一个 MOTOR 实例

/// @brief 运动控制结构体（陀螺仪yaw闭环）
/// 通过目标偏航角 + PID 实现双轮差分转向闭环控制
typedef struct Motion_Controller_ {
    int speed;              // 基础线速度 0~100
    float target_yaw;       // 目标偏航角 (°)，陀螺仪闭环目标
    uint8_t enabled;        // 运动使能标志：0=停止，1=运行

    PID_TypeDef *yaw_pid;   // 偏航角 PID 控制器指针

    // 方法
    void (*set_target_yaw)(struct Motion_Controller_ *self, float yaw);
    void (*set_speed)(struct Motion_Controller_ *self, int speed);
    void (*enable)(struct Motion_Controller_ *self, uint8_t en);
    void (*update)(struct Motion_Controller_ *self, float dt);
} Motion_Controller;

/* ── 运动模式枚举 ── */
typedef enum {
    MODE_FORWARD   = 0,
    MODE_BACKWARD  = 1,
    MODE_RIGHT     = 2,
    MODE_LEFT      = 3,
    MODE_TURN      = 4,
    MODE_CIRCLE    = 5,
    MODE_STOP      = 6,
    MODE_LIGHT_ON  = 7,
    MODE_LIGHT_OFF = 8,
    MODE_FAN_ON    = 9,
    MODE_FAN_OFF   = 10,
    MODE_NIGHT     = 11,
    MODE_UNKNOWN   = 12
} motor_mode_t;


void motion_init(Motion_Controller *m, int speed, PID_TypeDef *pid);
void motion_set_target_yaw(Motion_Controller *m, float yaw);
void motion_set_speed_m(Motion_Controller *m, int speed);
void motion_enable_m(Motion_Controller *m, uint8_t enable);
void motion_update_m(Motion_Controller *m, float dt);

#endif /* _MOTION_H */
