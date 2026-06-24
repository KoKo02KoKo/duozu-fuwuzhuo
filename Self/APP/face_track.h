#ifndef FACE_TRACK_H
#define FACE_TRACK_H

#include "main.h"
#include "pid.h"
#include "servo.h"

typedef struct FACE_TRACK_DATA_T face_track_data_t;


extern SERVO sh; // 定义一个 SERVO 实例
extern SERVO sl; // 定义一个 SERVO 实例
extern face_track_data_t face_data;     // 追踪数据实例
extern PID_TypeDef       track_x_pid;   // 水平追踪 PID（驱动 sl 舵机）
extern PID_TypeDef       track_y_pid;   // 垂直追踪 PID（驱动 sh 舵机）



/* ========================== 数据结构 ========================== */

/// @brief 视觉追踪目标数据（来自 web_uart: $23,06,x,y,w,h#）
typedef struct FACE_TRACK_DATA_T{
    int cx;           // 目标中心 x (0~640, 画面中心=320)
    int cy;           // 目标中心 y
    int w;            // 目标宽度
    int h;            // 目标高度
    uint8_t valid;    // =1 表示当前帧数据有效
    uint32_t tick;    // 收到此帧时的系统 tick (用于计算 dt)
} face_track_data_t;

/* ========================== API ========================== */

/// @brief 初始化人脸追踪模块（PID 参数等）
void face_track_init(void);
int  face_track_parse(const uint8_t *buf, uint8_t len);// 解析来自 k230_uart 的数据帧，更新 face_data
void face_track_update(void);// 根据 face_data 和 PID 输出，更新 sh/sl 舵机角度

#endif /* FACE_TRACK_H */
