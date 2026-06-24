#include "motion.h"
#include "motor.h"
#include "angle.h"
#include <stdlib.h>

Motion_Controller motion;
PID_TypeDef yaw_pid;   // 偏航角 PID 实例
MOTOR ml; // 定义一个 MOTOR 实例
MOTOR mr; // 定义一个 MOTOR 实例

/* ===================== 内部辅助函数 ===================== */

/// @brief 限幅到 [-100, 100]
static int clamp_speed(int v)
{
    if (v > 100)  return 100;
    if (v < -100) return -100;
    return v;
}

/* ===================== 对外接口 ===================== */

/// @brief 初始化运动控制器
/// @param m     控制器实例
/// @param speed 基础线速度 (0~100)
/// @param pid   偏航角 PID 控制器指针（如 &yaw_pid）
void motion_init(Motion_Controller *m, int speed, PID_TypeDef *pid)
{
    m->speed      = (speed > 100) ? 100 : (speed < -100 ? -100 : speed);
    m->target_yaw = 0.0f;
    m->enabled    = 0;
    m->yaw_pid    = pid;

    // 绑定方法
    m->set_target_yaw = motion_set_target_yaw;
    m->set_speed      = motion_set_speed_m;
    m->enable         = motion_enable_m;
    m->update         = motion_update_m;
}

/// @brief 设置目标偏航角（陀螺仪闭环）
/// @param m   控制器实例
/// @param yaw 目标偏航角，范围 -180° ~ +180°
void motion_set_target_yaw(Motion_Controller *m, float yaw)
{
    m->target_yaw = yaw;
}

/// @brief 设置基础线速度
/// @param m     控制器实例
/// @param speed 速度值 0~100
void motion_set_speed_m(Motion_Controller *m, int speed)
{
    if (speed > 100) speed = 100;
    if (speed < -100) speed = -100;
    m->speed = speed;
}

/// @brief 运动使能/停止
/// @param m      控制器实例
/// @param enable 1=启动运动，0=停止并刹车
void motion_enable_m(Motion_Controller *m, uint8_t enable)
{
    uint8_t was_enabled = m->enabled;
    m->enabled = enable;
    if (!enable) {
        ml.set_speed(&ml, 0);
        mr.set_speed(&mr, 0);
    }
    // 从停止→启动时复位PID积分，防止上一模式的积分残留导致拐弯
    if (enable && !was_enabled && m->yaw_pid) {
        m->yaw_pid->integral = 0.0f;
        m->yaw_pid->prev_error = 0.0f;
    }
}

/// @brief 运动更新（需在主循环中周期调用）
///
/// 核心闭环逻辑：
///   1. 读取陀螺仪当前 yaw 值 (angle.yaw)
///   2. 计算 yaw 误差 = target_yaw - current_yaw（带 ±180° 环绕处理）
///   3. PID 计算转向修正量 turn
///   4. 混入电机：left = speed - turn, right = speed + turn
///
///   效果：
///   - 误差=0  → 左右同速，直行
///   - 误差≠0 → PID 自动修正，差速转向直到对准目标
///
/// @param m  控制器实例
/// @param dt 控制周期 (秒)，如 0.01 表示 10ms
void motion_update_m(Motion_Controller *m, float dt)
{
    if (!m->enabled) return;
    if (!m->yaw_pid) return;

    // 1. 读取当前陀螺仪 yaw
    float current_yaw = angle.yaw;

    // 2. PID 计算转向修正量（PID_Update 内部已处理角度环绕）
    float turn = m->yaw_pid->update(m->yaw_pid, m->target_yaw, current_yaw, dt);
    
    // 3. 差分混入电机速度
    int left_speed  = clamp_speed((int)(m->speed - turn));
    int right_speed = clamp_speed((int)(m->speed + turn));

    // 4. 输出到左右电机
    ml.set_speed(&ml, left_speed);
    mr.set_speed(&mr, right_speed);
}
