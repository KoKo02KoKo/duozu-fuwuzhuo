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
    OA_GO                  = 0,  /* 直行（无障碍）                    */
    OA_LEFT   = 1,  /* 左转90°→直行→右转90°            */
    OA_RIGHT  = 2,  /* 右转90°→直行→左转90°            */
    OA_STOP                = 4   /* 紧急停止                          */
} oa_action_t;

/* ── 全局变量 ── */
extern volatile oa_action_t oa_action; /* 当前避障动作 (调试用)    */
extern volatile uint8_t oa_pattern;    /* 当前传感器标志 (调试用)  */
extern volatile uint8_t oa_phase;     /* 当前阶段 (调试用)        */

void obstacle_make_flags(const uint8_t dist_cm[4], uint8_t flag[4]);
oa_action_t obstacle_decide(const uint8_t flag[4]);
void obstacle_execute(oa_action_t action);

#ifdef __cplusplus
}
#endif

#endif /* __OBSTACLE_H__ */
