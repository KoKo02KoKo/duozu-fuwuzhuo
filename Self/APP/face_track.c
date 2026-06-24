/**
  ******************************************************************************
  * @file    face_track.c
  * @brief   人脸/视觉追踪模块
  *
  * 解析 ESP32 通过 web_uart 发送的 $23,06,x,y,w,h# 帧，
  * 并通过 PID 控制 sl 舵机使目标保持在画面中心。
  ******************************************************************************
  */
#include "face_track.h"
#include "uart.h"   // debug_uart 用于诊断输出

/* ========================== 全局变量 ========================== */

SERVO sh; // 定义一个 SERVO 实例
SERVO sl; // 定义一个 SERVO 实例
face_track_data_t face_data = {320, 0, 0, 0, 0, 0};  // 默认中心=320
PID_TypeDef       track_x_pid;   // 水平追踪 PID（驱动 sl）
PID_TypeDef       track_y_pid;   // 垂直追踪 PID（驱动 sh）
static uint32_t   last_face_tick = 0;  // 上一帧的系统 tick

/* ========================== 初始化 ========================== */

/// @brief 初始化人脸追踪 PID 控制器（增量式，双轴云台）
///
/// 摄像头固定在云台上，舵机转动 → 画面移动 → 目标位置变化
///   sl 舵机: 水平旋转 (pan),  画面 cx=0~640, 中心=320
///   sh 舵机: 垂直俯仰 (tilt), 画面 cy=0~480, 中心=240
///
/// 假设: 摄像头 HFOV≈60° VFOV≈45°, 1px≈0.094°(水平) / 0.094°(垂直)
void face_track_init(void)
{
    // ── 水平 PID: 驱动 sl 舵机 ──
    PID_Init(&track_x_pid, 0.06f, 0, 0.002f, -20.0f, 20.0f);
    track_x_pid.update = PID_Update_Linear;

    // ── 垂直 PID: 驱动 sh 舵机 ──
    PID_Init(&track_y_pid, 0.03f, 0, 0.001f, -15.0f, 15.0f);
    track_y_pid.update = PID_Update_Linear;

    last_face_tick = HAL_GetTick();
}

/* ========================== 帧解析 ========================== */

/// @brief 解析 $23,06,x,y,w,h# 格式的追踪数据帧
///
/// 帧格式: '$' + cmd_id + ',' + sub_cmd + ',' + x + ',' + y + ',' + w + ',' + h + '#'
/// 容忍 '#' 后附带 \r\n, 容忍 '$' 前有垃圾数据
///
/// @return 1=解析成功, 0=未识别到有效帧
int face_track_parse(const uint8_t *buf, uint8_t len)
{
    const uint8_t *start = buf;
    const uint8_t *end   = buf + len;
    const uint8_t *dollar = NULL;
    const uint8_t *hash  = NULL;

    /* ── 搜索 '$' 起始 ── */
    {
        const uint8_t *p;
        for (p = start; p < end; p++) { if (*p == '$') { dollar = p; break; } }
    }
    if (dollar == NULL) return 0;

    /* ── 搜索 '#' 结尾 ── */
    {
        const uint8_t *q;
        for (q = dollar + 1; q < end; q++) { if (*q == '#') { hash = q; break; } }
    }
    if (hash == NULL) return 0;

    /* ── 有效数据长度检查 ── */
    int data_len = (int)(hash - dollar) + 1;  // 含 '$' 和 '#'
    if (data_len < 14) return 0;  // 最短帧: $23,06,0,0,0,0# = 16字符, 留余量

    /* ── 解析 6 个逗号分隔的整数值 ── */
    int values[6] = {0};
    int vi = 0;
    const uint8_t *r = dollar + 1;  // 跳过 '$'

    while (r < hash && vi < 6)
    {
        while (r < hash && *r == ' ') r++;  // 跳过空格

        int val = 0;
        while (r < hash && *r >= '0' && *r <= '9')
        {
            val = val * 10 + (*r - '0');
            r++;
        }
        values[vi++] = val;

        if (r < hash && *r == ',') r++;
    }

    /* ── 验证指令 ID ── */
    if (vi != 6 || values[0] != 23 || values[1] != 6)
    {
        return 0;
    }

    /* ── 更新追踪数据 ── */
    face_data.cx    = values[2] + values[4] / 2;  // x + w/2 = 中心 x
    face_data.cy    = values[3] + values[5] / 2;  // y + h/2 = 中心 y
    face_data.w     = values[4];
    face_data.h     = values[5];
    face_data.tick  = HAL_GetTick();              // 记录收帧时刻
    face_data.valid = 1;
    return 1;
}

/* ========================== PID 控制 ========================== */

/// @brief 双轴增量式 PID: 同时追踪水平和垂直方向
///
///   水平: sl 舵机, target=320, measurement=face_data.cx
///   垂直: sh 舵机, target=240, measurement=face_data.cy
///
/// cx<320 → sl 右转 → 人脸右移靠近中心
/// cx>320 → sl 左转 → 人脸左移靠近中心
/// cy<240 → sh 上仰 → 人脸上移靠近中心
/// cy>240 → sh 下俯 → 人脸下移靠近中心
void face_track_update(void)
{
    if (!face_data.valid) return;

    /* ── 计算真实 dt (帧间间隔) ── */
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - last_face_tick) * 0.001f;  // ms → s
    if (dt <= 0.0f) dt = 0.033f;   // 最小 33ms (≈30fps)
    if (dt > 0.5f)  dt = 0.1f;     // 最大 100ms, 防止积分爆炸
    last_face_tick = now;

    /* ── 水平追踪: sl 舵机, target=画面水平中心320 ── */
    {
        float x_out = track_x_pid.update(&track_x_pid, 320.0f, (float)face_data.cx, dt);
        sl.angle += (int)x_out;
    }

    /* ── 垂直追踪: sh 舵机, target=画面垂直中心240 (480px高度) ── */
    {
        float y_out = track_y_pid.update(&track_y_pid, 240.0f, (float)face_data.cy, dt);
        sh.angle += (int)y_out;
    }

    face_data.valid = 0;  // 清除标志, 等待下一帧
}
