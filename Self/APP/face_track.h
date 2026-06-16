#ifndef FACE_TRACK_H
#define FACE_TRACK_H

#include "main.h"
#include "pid.h"
#include "servo.h"

/* ========================== 数据结构 ========================== */

/// @brief 视觉追踪目标数据（来自 web_uart: $23,06,x,y,w,h#）
typedef struct {
    int cx;           // 目标中心 x (0~640, 画面中心=320)
    int cy;           // 目标中心 y
    int w;            // 目标宽度
    int h;            // 目标高度
    uint8_t valid;    // =1 表示当前帧数据有效
    uint32_t tick;    // 收到此帧时的系统 tick (用于计算 dt)
} face_track_data_t;

extern face_track_data_t face_data;     // 追踪数据实例
extern PID_TypeDef       track_x_pid;   // 水平追踪 PID（驱动 sl 舵机）
extern PID_TypeDef       track_y_pid;   // 垂直追踪 PID（驱动 sh 舵机）

/* ========================== API ========================== */

/// @brief 初始化人脸追踪模块（PID 参数等）
void face_track_init(void);

/// @brief 解析 web_uart 接收缓冲区中的 $23,06,x,y,w,h# 帧
/// @param buf  接收缓冲区
/// @param len  缓冲区有效数据长度
/// @return 1=解析成功并更新 face_data, 0=未识别到有效帧
int  face_track_parse(const uint8_t *buf, uint8_t len);

/// @brief 根据 face_data 运行 PID 并更新 sl 舵机角度（主循环中调用）
void face_track_update(void);

#endif /* FACE_TRACK_H */
