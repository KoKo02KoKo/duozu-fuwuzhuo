/**
  ******************************************************************************
  * @file    obstacle.c
  * @brief   基于标志位查表的避障算法实现
  *
  * 思路:
  *   四个传感器 → 4-bit 标志 → 16 种模式 → 查表得动作 → motion 控制器闭环
  *
  *   传感器排列:  S1(左端)  S2(左中)  S3(右中)  S4(右端)
  *   对应 bit:    bit3      bit2      bit1      bit0
  *
  *   标志含义:  0 = 安全(无障碍),  1 = 有障碍
  *
  *   运动输出统一由 motion.update() 在主循环中完成 PID 闭环，
  *   本文件仅负责传感器解析 + 决策 + 设置 motion 参数。
  ******************************************************************************
  */
#include "obstacle.h"
#include "angle.h"     /* angle.yaw */
#include "motion.h"
#include <math.h>      /* fabsf */
  


/* ── 全局变量 ── */
volatile oa_action_t  oa_action  = OA_STOP;    /* 当前动作 */
volatile uint8_t      oa_pattern = 0;        /* 当前传感器 pattern */
volatile uint8_t      oa_phase = 0;        

/* ── 规则表 ────────────────────────────────────────────────────────
 *
 *   pattern  S1 S2 S3 S4    含义                      动作
 *  ───────────────────────────────────────────────────────────────
 *   0b0000   〇 〇 〇 〇    全空                    → 直行
 *   0b0001   〇 〇 〇 ●    最右端有物              → 左转90度 直行一段  右转90度
 *   0b0010   〇 〇 ● 〇    右中有物                → 左转90度 直行一段  右转90度
 *   0b0011   〇 〇 ● ●    右侧全挡                → 左转90度 直行一段  右转90度
 *   0b0100   〇 ● 〇 〇    左中有物                → 右转90度 直行一段  左转90度
 *   0b0101   〇 ● 〇 ●    两边有中间空            → 左转90度 直行一段  右转90度
 *   0b0110   〇 ● ● 〇    中间两挡两端空           → 左转90度 直行一段  右转90度
 *   0b0111   〇 ● ● ●    仅左端空                 → 左转90度 直行一段  右转90度
 *   0b1000   ● 〇 〇 〇    最左端有物              → 右转90度 直行一段  左转90度
 *   0b1001   ● 〇 〇 ●    两端有中间空            → 右转90度 直行一段  左转90度
 *   0b1010   ● 〇 ● 〇    间隔挡                  → 右转90度 直行一段  左转90度
 *   0b1011   ● 〇 ● ●    仅S2左中空              → 左转90度 直行一段  右转90度
 *   0b1100   ● ● 〇 〇    左侧全挡                → 右转90度 直行一段  左转90度
 *   0b1101   ● ● 〇 ●    仅S3右中空              → 右转90度 直行一段  左转90度
 *   0b1110   ● ● ● 〇    仅右端空                → 右转90度 直行一段  左转90度
 *   0b1111   ● ● ● ●    全堵                    → 右转90度 直行一段  左转90度
 *  ─────────────────────────────────────────────────────────────── */

typedef struct {
    uint8_t     pattern;
    oa_action_t action;
} oa_rule_t;

static const oa_rule_t rule_table[] = {
    { 0b0000, OA_GO                  },   /* 全空 → 直行               */
    { 0b0001, OA_LEFT  },   /* 最右端有物 → 左转直行右转   */
    { 0b0010, OA_LEFT  },   /* 右中有物 → 左转直行右转     */
    { 0b0011, OA_LEFT  },   /* 右侧全挡 → 左转直行右转     */
    { 0b0100, OA_RIGHT },   /* 左中有物 → 右转直行左转     */
    { 0b0101, OA_LEFT  },   /* 两边有中间空 → 左转直行右转  */
    { 0b0110, OA_LEFT  },   /* 中间两挡两端空 → 左转直行右转 */
    { 0b0111, OA_LEFT  },   /* 仅左端空 → 左转直行右转     */
    { 0b1000, OA_RIGHT },   /* 最左端有物 → 右转直行左转   */
    { 0b1001, OA_RIGHT },   /* 两端有中间空 → 右转直行左转  */
    { 0b1010, OA_RIGHT },   /* 间隔挡 → 右转直行左转       */
    { 0b1011, OA_LEFT  },   /* 仅S2空 → 左转直行右转       */
    { 0b1100, OA_RIGHT },   /* 左侧全挡 → 右转直行左转     */
    { 0b1101, OA_RIGHT },   /* 仅S3空 → 右转直行左转       */
    { 0b1110, OA_RIGHT },   /* 仅右端空 → 右转直行左转     */
    { 0b1111, OA_RIGHT },   /* 全堵 → 右转直行左转         */
};

/**
 * @brief 将四个距离值转换为标志位
 *
 *         距离 < DANGER_DIST  → flag = 1 (有障碍)
 *         距离 >= DANGER_DIST → flag = 0 (安全)
 *
/// @param dist_cm  距离数组(cm), uint8_t 即可（web 数据 0~255cm，DANGER_DIST=40）
 * @param flag     输出的标志数组
 */
void obstacle_make_flags(const uint8_t dist_cm[4], uint8_t flag[4])
{
    for (int i = 0; i < 4; i++) {
        flag[i] = (dist_cm[i] < DANGER_DIST) ? 1 : 0;
    }
}

/**
 * @brief 查表决策: 将 4-bit 标志映射为避障动作
 *
 * @param flag  标志数组 [S1, S2, S3, S4]
 * @return oa_action_t  对应动作
 */
oa_action_t obstacle_decide(const uint8_t flag[4])
{
    /* 拼成 4-bit: bit3=S1, bit2=S2, bit1=S3, bit0=S4 */
    uint8_t pattern = (flag[0] << 3) | (flag[1] << 2)
                    | (flag[2] << 1) | (flag[3] << 0);

    /* 公开当前 pattern 供调试查看 */
    oa_pattern = pattern;

    for (int i = 0; i < (int)(sizeof(rule_table) / sizeof(rule_table[0])); i++) {
        if (rule_table[i].pattern == pattern) {
            return rule_table[i].action;
        }
    }
    return OA_STOP;  /* 安全兜底 */
}

/**
 * @brief 执行避障动作
 *
 * 通过 motion 控制器的 yaw 轴 PID 闭环完成转向，
 * 统一由 motion.update() 在主循环中每10ms输出电机，保证架构一致。
 *
 * @param action  由 obstacle_decide() 返回的动作
 */
void obstacle_execute(oa_action_t action)
{
    /* ── 多阶段状态机 (用于 TURN_LEFT_GO_RIGHT / TURN_RIGHT_GO_LEFT) ── */
    /*     阶段0=空闲, 阶段1=转向90°, 阶段2=直行, 阶段3=转回90°      */
    static oa_action_t saved_action       = OA_GO;
    static oa_action_t last_simple_action = OA_GO;
    static uint32_t    phase_tick         = 0;
    static uint8_t     phase              = 0;

    /* 动作变化 → 重置阶段（复合动作进行中不因 OA_GO 中断） */
    if (action != saved_action) {
        if ((saved_action == OA_LEFT || saved_action == OA_RIGHT)
            && action == OA_GO && phase != 0) {
            /* 继续当前复合动作 */
            action = saved_action;
        } else {
            phase = 0;
            saved_action = action;
        }
    }
    else {
        return;
    }

    /* 记录当前动作供调试 */
    oa_phase = phase;
    oa_action = action;

    

    /* ══════════════════════════════════════════════════════════════
     *  复合动作: 转向90°→ 直行 → 转回90°
     * ══════════════════════════════════════════════════════════════ */
    if (action == OA_LEFT || action == OA_RIGHT) {

        if (phase == 0) {
            phase = 1;
            phase_tick = HAL_GetTick();
        }

        if (phase == 1) {
            static uint8_t phase1_turn = 0;
            /* ── 第一阶段: 转向90° ── */
            if(phase1_turn == 0)
            {
                phase1_turn = 1;
                motion.enable(&motion, 0);
                if (action == OA_LEFT) {
                    motion.set_target_yaw(&motion, -90);
                } else {
                    motion.set_target_yaw(&motion, 90);
                }
                motion.set_speed(&motion, 0);
                motion.enable(&motion, 1);
            }
            /* 角度闭环判定: yaw误差≤3°认为转向到位 */
            {
                float yaw_err = motion.target_yaw - angle.yaw;
                if (yaw_err > 180.0f)  yaw_err -= 360.0f;
                if (yaw_err < -180.0f) yaw_err += 360.0f;
                if (yaw_err < 5.0f && yaw_err > -5.0f) {
                    phase = 2;
                    phase_tick = HAL_GetTick();
                }
            }
            //  if (HAL_GetTick() - phase_tick >= 4000) {
            //     phase = 2;
            //     phase1_turn = 0;
            //     phase_tick = HAL_GetTick();
            //  }
        } else if (phase == 2) {
            /* ── 第二阶段: 直行 ── */
                motion.enable(&motion, 0);
                motion.set_target_yaw(&motion, 0);  /* 锁定当前朝向直行 */
                motion.set_speed(&motion, 50);
                motion.enable(&motion, 1);
            
            if (HAL_GetTick() - phase_tick >= 4000) {
                phase = 3;
                phase_tick = HAL_GetTick();
            }
        } else if (phase == 3) {
            static uint8_t phase3_turn = 0;
            /* ── 第三阶段: 转回90° ── */
            if(phase3_turn == 0){
				phase3_turn = 1;
                motion.enable(&motion, 0);
                if (action == OA_LEFT) {
                    motion.set_target_yaw(&motion, 90);
                } else {
                    motion.set_target_yaw(&motion, -90);
                }
                motion.set_speed(&motion, 0);
                motion.enable(&motion, 1);
            }
            /* 角度闭环判定: yaw误差≤3°认为转回到位 */
            {
                float yaw_err = motion.target_yaw - angle.yaw;
                if (yaw_err > 180.0f)  yaw_err -= 360.0f;
                if (yaw_err < -180.0f) yaw_err += 360.0f;
                if (yaw_err < 5.0f && yaw_err > -5.0f) {
                    phase = 0;  /* 回到空闲，等待 sensor 重判 */
                }
            }
            // if (HAL_GetTick() - phase_tick >= 4000) {
            //     phase = 0;
            //     phase3_turn = 0;
            //     phase_tick = HAL_GetTick();
            //  }
        }

    } else {
        /* ══════════════════════════════════════════════════════════
         *  普通动作 (无阶段): OA_GO / OA_STOP
         * ══════════════════════════════════════════════════════════ */
        switch (action) {

        case OA_GO:
            if (last_simple_action != OA_GO) {
                motion.enable(&motion, 0);
                motion.set_target_yaw(&motion, 0);
                motion.set_speed(&motion, 0);
                motion.enable(&motion, 1);
            }
            break;

        case OA_STOP:
        default:
            if (last_simple_action != OA_STOP) {
                motion.enable(&motion, 0);
            }
            break;
        }

        /* 记录本次动作，防止下次重复调用导致 target 漂移 + PID 反复重置 */
        last_simple_action = action;
    }

}


