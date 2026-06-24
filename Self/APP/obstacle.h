/**
  ******************************************************************************
  * @file    obstacle.h
  * @brief   基于标志位查表的避障算法
  *
  * 用法:
  *   1. 在主循环中定期获取四个传感器距离值
  *   2. obstacle_make_flags()  将距离转为 4-bit 标志
  *   3. obstacle_decide()      查表获取动作
  *   4. obstacle_execute()     执行动作 (设置 motion 参数, 由 TIM4 中断闭环输出)
  ******************************************************************************
  */
#ifndef __OBSTACLE_H__
#define __OBSTACLE_H__

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── 距离阈值 (cm) ── */
#define DANGER_DIST     40      /* 小于此值判定为有障碍 (桌子深35+余量5) */

/* ── 避障动作枚举 ── */
typedef enum {
    OA_GO           = 0,        /* 直行（无障碍）           */
    OA_SLIGHT_LEFT  = 1,        /* 微左转 (target_yaw +15°) */
    OA_SLIGHT_RIGHT = 2,        /* 微右转 (target_yaw -15°) */
    OA_LEFT         = 3,        /* 左转   (target_yaw +35°) */
    OA_RIGHT        = 4,        /* 右转   (target_yaw -35°) */
    OA_HARD_LEFT    = 5,        /* 急左转 (target_yaw +60°) */
    OA_HARD_RIGHT   = 6,        /* 急右转 (target_yaw -60°) */
    OA_BACK         = 7,        /* 后退 + 随机转向           */
    OA_STOP         = 8,        /* 紧急停止                  */
    OA_BACK_THEN_LEFT  = 9,     /* 先后退500ms, 再左转       */
    OA_BACK_THEN_RIGHT = 10     /* 先后退500ms, 再右转       */
} oa_action_t;

/* ── 全局变量 ── */
extern volatile uint8_t oa_enable;     /* 避障开关: 1=开启, 0=关闭 */
extern volatile oa_action_t oa_action; /* 当前避障动作 (调试用)    */
extern volatile uint8_t oa_pattern;    /* 当前传感器标志 (调试用)  */

/* ── 全局距离数组 (由用户传感器驱动填充) ── */
extern volatile uint16_t obs_dist[4];  /* [S1左端, S2左中, S3右中, S4右端] */

/* ── API ── */
void obstacle_make_flags(const uint16_t dist_cm[4], uint8_t flag[4]);
oa_action_t obstacle_decide(const uint8_t flag[4]);
void obstacle_execute(oa_action_t action);
int  obstacle_parse_frame(const uint8_t *buf, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __OBSTACLE_H__ */
