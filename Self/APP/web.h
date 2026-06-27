#ifndef __WEB_H
#define __WEB_H

#include <stdint.h>
#include "main.h"
#include "motion.h"

/* ── 网页串口数据帧 ── */
#define WEB_FRAME_LEN   8           // 'S','t',dat1~4,cmd,'E'
#define WEB_FRAME_START 'S'         // 0x53
#define WEB_FRAME_TYPE  't'         // 0x74
#define WEB_FRAME_END   'E'         // 0x45

typedef struct {
    uint8_t dat[4];                 // dat1~dat4
    uint8_t cmd;                    // 运动指令
} web_frame_t;

extern web_frame_t web_frame;

/* ── 函数声明 ── */
void mode_execute(motor_mode_t mode);
void web_parse_frame(const uint8_t *buf, uint8_t len);

#endif /* __WEB_H */
