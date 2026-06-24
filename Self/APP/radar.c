/**
  ******************************************************************************
  * @file    radar.c
  * @brief   雷达追踪模块
  *
  * 解析 radar_uart (UART6) 发送的 T,ang,dist\r\n 帧，
  * 其中 ang 为追踪角度(int)，dist 为追踪距离(unsigned int)。
  ******************************************************************************
  */
#include "radar.h"
#include "uart.h"

/* ========================== 全局变量 ========================== */

radar_data_t radar_data = {0, 0, 0, 0};

/* ========================== 帧解析 ========================== */

/// @brief 解析 T,ang,dist\r\n 格式的雷达追踪数据帧
///
/// 帧格式: 'T' + ',' + track_ang(int) + ',' + track_dist(unsigned int) + '\r\n'
///
/// @return 1=解析成功, 0=未识别到有效帧
int radar_parse(const uint8_t *buf, uint8_t len)
{
    const uint8_t *start = buf;
    const uint8_t *end   = buf + len;

    /* ── 搜索 'T' 起始标识 ── */
    const uint8_t *t_ptr = NULL;
    {
        const uint8_t *p;
        for (p = start; p < end; p++) { if (*p == 'T') { t_ptr = p; break; } }
    }
    if (t_ptr == NULL) return 0;

    /* ── 验证 'T' 后紧跟 ',' ── */
    if (t_ptr + 1 >= end || *(t_ptr + 1) != ',') return 0;

    /* ── 搜索 '\r\n' 结尾 ── */
    const uint8_t *cr = NULL;
    {
        const uint8_t *p;
        for (p = t_ptr + 2; p < end - 1; p++)
        {
            if (*p == '\r' && *(p + 1) == '\n') { cr = p; break; }
        }
    }
    if (cr == NULL) return 0;

    /* ── 解析两个逗号分隔的数值 ── */
    int      ang  = 0;
    uint32_t dist = 0;
    int      vi   = 0;
    const uint8_t *r = t_ptr + 2;  // 跳过 'T,'

    while (r < cr && vi < 2)
    {
        /* 跳过空格 */
        while (r < cr && *r == ' ') r++;

        /* 解析数值（支持负号，仅第一个值 ang 可为负） */
        int neg = 0;
        if (r < cr && *r == '-') { neg = 1; r++; }

        uint32_t val = 0;
        while (r < cr && *r >= '0' && *r <= '9')
        {
            val = val * 10 + (uint32_t)(*r - '0');
            r++;
        }

        if (vi == 0) ang  = neg ? -(int)val : (int)val;
        else         dist = val;

        vi++;

        /* 跳过逗号 */
        if (r < cr && *r == ',') r++;
    }

    /* ── 验证解析了两个值 ── */
    if (vi != 2) return 0;

    /* ── 更新全局数据 ── */
    radar_data.track_ang  = ang;
    radar_data.track_dist = dist;
    radar_data.valid      = 1;
    radar_data.tick       = HAL_GetTick();

    return 1;
}
