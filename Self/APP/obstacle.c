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
  *   运动输出统一由 motion.update() 在 TIM4 中断中完成 PID 闭环，
  *   本文件仅负责传感器解析 + 决策 + 设置 motion 参数。
  ******************************************************************************
  */
#include "obstacle.h"
#include "motor.h"     /* MOTOR 类型 */
#include "angle.h"     /* angle.yaw */
#include "motion.h"      
#include "others.h"      


/* ── 全局变量 ── */
volatile uint8_t      oa_enable  = 1;        /* 避障开关, 默认开启 */
volatile oa_action_t  oa_action  = OA_GO;    /* 当前动作 */
volatile uint8_t      oa_pattern = 0;        /* 当前传感器 pattern */
volatile uint16_t     obs_dist[4] = {999, 999, 999, 999}; /* 距离数组, 初始化为无穷远 */

/* ── 规则表 ────────────────────────────────────────────────────────
 *
 *   pattern  S1 S2 S3 S4    含义                      动作
 *  ───────────────────────────────────────────────────────────────
 *   0b0000   〇 〇 〇 〇    全空                    → 直行
 *   0b0001   〇 〇 〇 ●    最右端有物              → 微左转
 *   0b0010   〇 〇 ● 〇    右中有物                → 先后退再左转
 *   0b0011   〇 〇 ● ●    右侧全挡                → 先后退再左转
 *   0b0100   〇 ● 〇 〇    左中有物                → 先后退再右转
 *   0b0101   〇 ● 〇 ●    两边有中间空            → 先后退再左转
 *   0b0110   〇 ● ● 〇    中间两挡两端空          → 先后退再右转
 *   0b0111   〇 ● ● ●    仅左端空                → 先后退再左转
 *   0b1000   ● 〇 〇 〇    最左端有物              → 微右转
 *   0b1001   ● 〇 〇 ●    两端有中间空            → 先后退再右转
 *   0b1010   ● 〇 ● 〇    间隔挡                  → 先后退再右转
 *   0b1011   ● 〇 ● ●    仅S2左中空              → 先后退再左转
 *   0b1100   ● ● 〇 〇    左侧全挡                → 先后退再右转
 *   0b1101   ● ● 〇 ●    仅S3右中空              → 先后退再右转
 *   0b1110   ● ● ● 〇    仅右端空                → 先后退再右转
 *   0b1111   ● ● ● ●    全堵                    → 后退随机转
 *
 *  S2或S3触发=中间有障碍 → 先后退再转; 仅S1或S4触发=边缘 → 微转
 *  ─────────────────────────────────────────────────────────────── */

typedef struct {
    uint8_t     pattern;
    oa_action_t action;
} oa_rule_t;

static const oa_rule_t rule_table[] = {
    { 0b0000, OA_GO              },   /* 全空 → 直行               */
    { 0b0001, OA_SLIGHT_LEFT     },   /* 最右端有物 → 微左转        */
    { 0b0010, OA_BACK_THEN_LEFT  },   /* 右中有物 → 先后退再左转     */
    { 0b0011, OA_BACK_THEN_LEFT  },   /* 右侧全挡 → 先后退再左转     */
    { 0b0100, OA_BACK_THEN_RIGHT },   /* 左中有物 → 先后退再右转     */
    { 0b0101, OA_BACK_THEN_LEFT  },   /* 两边有中间空 → 先后退再左转  */
    { 0b0110, OA_BACK_THEN_RIGHT },   /* 中间两挡两端空 → 先后退再右转 */
    { 0b0111, OA_BACK_THEN_LEFT  },   /* 仅左端空 → 先后退再左转     */
    { 0b1000, OA_SLIGHT_RIGHT    },   /* 最左端有物 → 微右转        */
    { 0b1001, OA_BACK_THEN_RIGHT },   /* 两端有中间空 → 先后退再右转  */
    { 0b1010, OA_BACK_THEN_RIGHT },   /* 间隔挡 → 先后退再右转       */
    { 0b1011, OA_BACK_THEN_LEFT  },   /* 仅S2空 → 先后退再左转       */
    { 0b1100, OA_BACK_THEN_RIGHT },   /* 左侧全挡 → 先后退再右转     */
    { 0b1101, OA_BACK_THEN_RIGHT },   /* 仅S3空 → 先后退再右转       */
    { 0b1110, OA_BACK_THEN_RIGHT },   /* 仅右端空 → 先后退再右转     */
    { 0b1111, OA_BACK            },   /* 全堵 → 快速直退800ms后随机转   */
};

/**
 * @brief 将四个距离值转换为标志位
 *
 *         距离 < DANGER_DIST  → flag = 1 (有障碍)
 *         距离 >= DANGER_DIST → flag = 0 (安全)
 *
 * @param dist_cm  距离数组(cm), [0]=S1左端, [1]=S2左中, [2]=S3右中, [3]=S4右端
 * @param flag     输出的标志数组
 */
void obstacle_make_flags(const uint16_t dist_cm[4], uint8_t flag[4])
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
 * 统一由 motion.update() 在 TIM4 中断中输出电机，保证架构一致。
 *
 * @param action  由 obstacle_decide() 返回的动作
 */
void obstacle_execute(oa_action_t action)
{
    int base_speed = 60;  /* 基础速度 */

    /* ── 多阶段状态机 (用于 BACK_THEN_LEFT / BACK_THEN_RIGHT) ── */
    /*     阶段0=空闲, 阶段1=后退中, 阶段2=转向中, 阶段3=前进冷却   */
    static oa_action_t saved_action = OA_GO;
    static uint32_t    phase_tick   = 0;
    static uint32_t    backup_ms    = 500;  /* 后退时长, 根据遮挡程度动态调整 */
    static uint8_t     phase        = 0;

    /* 动作变化 → 重置阶段 (但前进冷却期不被打断, 保证脱困后先走一段) */
    if (action != saved_action && phase != 3) {
        phase = 0;
        saved_action = action;
    }

    /* 记录当前动作供调试 */
    oa_action = action;

    /* ══════════════════════════════════════════════════════════════
     *  先后退再转: OA_BACK_THEN_LEFT / OA_BACK_THEN_RIGHT
     *  阶段:后退 → 转向 → 前进冷却 → 重判
     * ══════════════════════════════════════════════════════════════ */
    if (action == OA_BACK_THEN_LEFT || action == OA_BACK_THEN_RIGHT) {

        if (phase == 0) {
            phase = 1;
            phase_tick = HAL_GetTick();

            /* 统计 oa_pattern 中触发的传感器数量, 决定后退时长 */
            uint8_t bits = oa_pattern;
            int n = (bits & 1) + ((bits >> 1) & 1) + ((bits >> 2) & 1) + ((bits >> 3) & 1);
            /* n=1:轻挡300ms, n=2:中挡500ms, n=3:重挡800ms, n=4:全堵1200ms */
            backup_ms = (n == 1) ? 300 : (n == 2) ? 500 : (n == 3) ? 800 : 1200;
        }

        if (phase == 1) {
            /* ── 第一阶段: 后退 ── */
            motion.set_target_yaw(&motion, angle.yaw);
            motion.set_speed(&motion, -40);
            motion.enable(&motion, 1);

            if (HAL_GetTick() - phase_tick >= backup_ms) {
                phase = 2;                    /* 进入转向阶段 */
                phase_tick = HAL_GetTick();
            }
        } else if (phase == 2) {
            /* ── 第二阶段: 转向（PID 自动产生左右差速）── */
            if (action == OA_BACK_THEN_LEFT) {
                motion.set_target_yaw(&motion, angle.yaw + 45.0f);
            } else {
                motion.set_target_yaw(&motion, angle.yaw - 45.0f);
            }
            motion.set_speed(&motion, base_speed);
            motion.enable(&motion, 1);

            /* 转向完成 → 进入前进冷却期, 避免原地重判 */
            if (HAL_GetTick() - phase_tick >= 1000) {
                phase = 3;
                phase_tick = HAL_GetTick();
                saved_action = action;  /* 锁定动作, 防止冷却期被打断 */
            }
        } else {
            /* ── 第三阶段: 前进冷却 (脱困后强制直行一段时间) ── */
            motion.set_target_yaw(&motion, angle.yaw);
            motion.set_speed(&motion, base_speed);
            motion.enable(&motion, 1);
            if (HAL_GetTick() - phase_tick >= 1500) {
                phase = 0;              /* 冷却结束, 允许下一轮重判 */
            }
        }

    } else {
        /* ══════════════════════════════════════════════════════════
         *  普通动作 (无后退): 统一通过 motion 控制器设置参数，
         *  由 TIM4_ISR → motion.update() 完成 PID 闭环输出
         * ══════════════════════════════════════════════════════════ */
        switch (action) {

        case OA_GO:
            motion.set_target_yaw(&motion, angle.yaw);
            motion.set_speed(&motion, base_speed);
            motion.enable(&motion, 1);
            break;

        case OA_SLIGHT_LEFT:
            motion.set_target_yaw(&motion, angle.yaw + 15.0f);
            motion.set_speed(&motion, base_speed);
            motion.enable(&motion, 1);
            break;

        case OA_SLIGHT_RIGHT:
            motion.set_target_yaw(&motion, angle.yaw - 15.0f);
            motion.set_speed(&motion, base_speed);
            motion.enable(&motion, 1);
            break;

        case OA_LEFT:
            motion.set_target_yaw(&motion, angle.yaw + 35.0f);
            motion.set_speed(&motion, base_speed);
            motion.enable(&motion, 1);
            break;

        case OA_RIGHT:
            motion.set_target_yaw(&motion, angle.yaw - 35.0f);
            motion.set_speed(&motion, base_speed);
            motion.enable(&motion, 1);
            break;

        case OA_HARD_LEFT:
            motion.set_target_yaw(&motion, angle.yaw + 60.0f);
            motion.set_speed(&motion, base_speed);
            motion.enable(&motion, 1);
            break;

        case OA_HARD_RIGHT:
            motion.set_target_yaw(&motion, angle.yaw - 60.0f);
            motion.set_speed(&motion, base_speed);
            motion.enable(&motion, 1);
            break;

        case OA_BACK:
            /* ── 全堵: 快速后退, 先直退800ms再随机转向, 最后前进冷却 ── */
            {
                static oa_action_t last_back_action = OA_GO;
                static uint32_t    back_start = 0;
                static uint8_t     back_phase = 0;  /* 0=直退, 1=转向, 2=前进冷却 */
                static int         back_turn_dir = 0; /* 固定随机方向: +1右, -1左 */
                static float       back_turn_yaw = 0.0f; /* 进入转向时的 yaw */

                /* 从非BACK切过来时重置 (前进冷却期不打断) */
                if (last_back_action != OA_BACK && back_phase != 2) {
                    back_phase = 0;
                    back_start = HAL_GetTick();
                    last_back_action = OA_BACK;
                }

                if (back_phase == 0) {
                    /* ── 直退 800ms ── */
                    motion.set_target_yaw(&motion, angle.yaw);
                    motion.set_speed(&motion, -50);
                    motion.enable(&motion, 1);
                    if (HAL_GetTick() - back_start >= 800) {
                        back_phase = 1;
                        back_turn_yaw = angle.yaw;
                        /* 用 tick 低位决定方向, 一次选定不再变 */
                        back_turn_dir = (HAL_GetTick() & 0x100) ? -1 : +1;
                    }
                } else if (back_phase == 1) {
                    /* ── 固定方向转向 1500ms ── */
                    motion.set_target_yaw(&motion, back_turn_yaw + back_turn_dir * 60.0f);
                    motion.set_speed(&motion, -40);
                    motion.enable(&motion, 1);
                    if (HAL_GetTick() - back_start >= 800 + 1500) {
                        back_phase = 2;
                        back_start = HAL_GetTick();
                    }
                } else {
                    /* ── 前进冷却: 强制直行 1500ms ── */
                    motion.set_target_yaw(&motion, angle.yaw);
                    motion.set_speed(&motion, base_speed);
                    motion.enable(&motion, 1);
                    if (HAL_GetTick() - back_start >= 1500) {
                        back_phase = 0;
                        last_back_action = OA_GO; /* 冷却结束, 允许重判 */
                    }
                }
            }
            break;

        case OA_STOP:
        default:
            motion.enable(&motion, 0);
            break;
        }
    }
}

/**
 * @brief 解析 ESP32 发来的距离数据帧，填充 obs_dist[4]
 *
 * ESP32 UART2 距离帧格式:
 *   帧头:  0xFC 0xFD
 *   数据:  "one:XX two:XX three:XX four:XX"
 *   帧尾:  0xFE
 *
 * Command bytes are 0x00~0x06 and do not overlap frame markers.
 * Example: \xFC\xFDone:45 two:120 three:30 four:80\xFE
 *
 * @param buf   接收缓冲区
 * @param len   缓冲区有效字节数
 * @return      成功解析的数值个数 (应为4), 失败返回 0
 */
int obstacle_parse_frame(const uint8_t *buf, uint8_t len)
{
    if (buf == NULL || len < 12) return 0;

    const uint8_t *p = buf;
    const uint8_t *end = buf + len;

    while (p + 1 < end) {
        if (p[0] == 0xFC && p[1] == 0xFD) break;
        p++;
    }
    if (p + 1 >= end) return 0;
    p += 2;

    const uint8_t *frame_end = p;
    while (frame_end < end && *frame_end != 0xFE) frame_end++;
    if (frame_end >= end) return 0;

    static const char *keys[] = {"one", "two", "three", "four"};
    uint16_t vals[4];
    int parsed = 0;

    for (int i = 0; i < 4 && p < frame_end; i++) {

        while (p < frame_end && *p == ' ') p++;

        const char *key = keys[i];
        while (p < frame_end && *key != '\0') {
            if (*p != *key) return 0;
            p++;
            key++;
        }
		
        if (p < frame_end && *p == ':') p++;
        else return 0;

        uint16_t val = 0;
        int got_digit = 0;
        while (p < frame_end && *p >= '0' && *p <= '9') {
            val = val * 10 + (uint16_t)(*p - '0');
            got_digit = 1;
            p++;
        }
        if (!got_digit) return 0;

        vals[i] = val;
        parsed++;
    }

    while (p < frame_end && *p == ' ') p++;
    if (parsed != 4 || p != frame_end) return 0;

    for (int i = 0; i < 4; i++) {
        obs_dist[i] = vals[i];
    }

    return parsed;
}
