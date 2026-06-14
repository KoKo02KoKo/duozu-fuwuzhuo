#ifndef _SERVO_H
#define _SERVO_H

#include "tim.h"

typedef struct SERVO_ SERVO;

extern SERVO sh; // 定义一个 SERVO 实例
extern SERVO sl; // 定义一个 SERVO 实例

/// @brief Servo structure definition
struct SERVO_
{
    //public:
    int angle;
    //private:
    int angle_max;
    int angle_min;
    uint32_t duty;
    uint32_t duty_max;
    uint32_t duty_min;
    unsigned int channel; // PWM通道
    //public:
    void (*set_angle)(struct SERVO_ *s, int angle);
    //private:
    void (*set_duty)(struct SERVO_ *s, uint32_t duty);
};

void servo_init(SERVO *s, unsigned int channel, int angle_max, int angle_min, int duty_max, int duty_min);
void servo_Set_Duty(SERVO *s, uint32_t duty);
void servo_set_angle(SERVO *s, int angle);

#endif
