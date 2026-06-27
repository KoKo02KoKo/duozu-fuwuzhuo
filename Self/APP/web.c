#include "web.h"
#include "motion.h"

/* ── 全局变量 ── */
web_frame_t web_frame = {{0}, 0};

/* ===================== 网页串口帧解析 ===================== */

/// @brief 解析网页端发来的数据帧: 'S','t',dat1~4,cmd,'E'
/// @param buf 接收缓冲区
/// @param len 接收长度
void web_parse_frame(const uint8_t *buf, uint8_t len)
{
    // 校验帧长度
    if (len != WEB_FRAME_LEN) return;

    // 校验帧头 'S'、类型 't'、帧尾 'E'
    if (buf[0] != WEB_FRAME_START) return;
    if (buf[1] != WEB_FRAME_TYPE)  return;
    if (buf[7] != WEB_FRAME_END)   return;

    // 提取数据
    web_frame.dat[0] = buf[2];
    web_frame.dat[1] = buf[3];
    web_frame.dat[2] = buf[4];
    web_frame.dat[3] = buf[5];
    web_frame.cmd    = buf[6];

    // 执行运动指令 (0~6 有效)
    if (web_frame.cmd <= MODE_STOP) {
        mode_execute((motor_mode_t)web_frame.cmd);
    }
}

/* ===================== 运动指令执行 ===================== */

/// @brief 根据运动模式执行相应动作
void mode_execute(motor_mode_t mode_param)
{
    static motor_mode_t last_mode = MODE_UNKNOWN;

    // 模式没变 → 不重复设置，避免反复复位 PID
    if (mode_param == last_mode) {
        return;
    }
    last_mode = mode_param;

    switch (mode_param)
    {
        case MODE_FORWARD:
            motion.enable(&motion, 0);
            motion.set_speed(&motion, 80);
            motion.set_target_yaw(&motion, 0);
            motion.enable(&motion, 1);
            break;
        case MODE_BACKWARD:
            motion.enable(&motion, 0);
            motion.set_speed(&motion, -80);
            motion.set_target_yaw(&motion, 0);
            motion.enable(&motion, 1);
            break;
        case MODE_RIGHT:
            motion.enable(&motion, 0);
            motion.set_target_yaw(&motion, 90);
            motion.set_speed(&motion, 0);
            motion.enable(&motion, 1);
            break;
        case MODE_LEFT:
            motion.enable(&motion, 0);
            motion.set_target_yaw(&motion, -90);
            motion.set_speed(&motion, 0);
            motion.enable(&motion, 1);
            break;
        case MODE_TURN:
            motion.enable(&motion, 0);
            motion.set_target_yaw(&motion, 180);
            motion.set_speed(&motion, 0);
            motion.enable(&motion, 1);
            break;
        case MODE_CIRCLE:
            motion.enable(&motion, 0);
            motion.set_target_yaw(&motion, 360);
            motion.set_speed(&motion, 0);
            motion.enable(&motion, 1);
            break;
        case MODE_UNKNOWN:
        default:
            motion.enable(&motion, 0);
            break;
    }
}
