#ifndef RADAR_H
#define RADAR_H

#include "main.h"

/* ========================== 数据结构 ========================== */

/// @brief 雷达追踪目标数据（来自 radar_uart (UART6): T,ang,dist\r\n）
typedef struct {
    int      track_ang;    // 追踪角度
    uint32_t track_dist;   // 追踪距离
    uint8_t  valid;        // =1 表示当前帧数据有效
    uint32_t tick;         // 收到此帧时的系统 tick
} radar_data_t;

extern radar_data_t radar_data;   // 雷达追踪数据实例

/* ========================== API ========================== */

/// @brief 解析 web_uart 接收缓冲区中的 T,ang,dist\r\n 帧
/// @param buf  接收缓冲区
/// @param len  缓冲区有效数据长度
/// @return 1=解析成功并更新 radar_data, 0=未识别到有效帧
int radar_parse(const uint8_t *buf, uint8_t len);

#endif /* RADAR_H */
