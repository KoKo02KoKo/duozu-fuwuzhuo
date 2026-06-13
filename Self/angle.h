#ifndef _ANGLE_H
#define _ANGLE_H

#include <stdint.h>

typedef struct ANGLE_  ANGLE;

extern ANGLE angle; // 定义一个 ANGLE 实例

/// @brief Angle structure definition
typedef struct ANGLE_
{
    //public:
    float roll;
    float pitch;
    float yaw;
    float target_yaw;
    float target_roll;
    float target_pitch;
    
} ANGLE;


void angle_init(ANGLE *a);
uint8_t angle_update_from_jy61p_frame(ANGLE *a, const uint8_t *buf, uint8_t len);

#endif
