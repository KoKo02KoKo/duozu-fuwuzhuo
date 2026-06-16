#include "servo.h"

SERVO sh; // 定义一个 SERVO 实例
SERVO sl; // 定义一个 SERVO 实例

/// @brief Initialize the servo
void servo_init(SERVO *s, unsigned int channel, int angle_max,int angle_min,int duty_max,int duty_min)
{
    s->angle = 0;
    s->angle_max = angle_max;
    s->angle_min = angle_min;
    s->duty = 0;
    s->duty_max = duty_max;
    s->duty_min = duty_min;
    s->channel = channel;
    s->set_angle = servo_set_angle;
    s->set_duty = servo_Set_Duty;

    #ifndef SERVO_PWM_STARTED
    #define SERVO_PWM_STARTED
    // 启动PWM输出
    HAL_TIM_PWM_Init(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    #endif
}


/// @brief Set the servo duty cycle
void servo_Set_Duty(SERVO *s, uint32_t duty)
{
    // 钳位到 [min(duty_min,duty_max), max(duty_min,duty_max)] 范围
    uint32_t dmin = s->duty_min < s->duty_max ? s->duty_min : s->duty_max;
    uint32_t dmax = s->duty_min < s->duty_max ? s->duty_max : s->duty_min;
    if(duty > dmax) duty = dmax;
    if(duty < dmin) duty = dmin;
    __HAL_TIM_SET_COMPARE(&htim1, s->channel, duty);
}

/// @brief Set the servo angle
void servo_set_angle(SERVO *s, int angle)
{
    int angle_temp = 0;
    // 将角度映射到-180° ~ 180°范围内
    if(angle > 180)
        angle_temp = angle%360 - 360;
    else if(angle < -180)
        angle_temp = angle%360 + 360;
    else
        angle_temp = angle;
    // 限制角度在angle_min ~ angle_max范围内
    if(angle_temp > s->angle_max)
        angle_temp = s->angle_max;
    else if(angle_temp < s->angle_min)
        angle_temp = s->angle_min;

    // 将角度映射为占空比 (duty_min ~ duty_max 对应 angle_min° ~ angle_max°)
    int32_t duty_range = (int32_t)s->duty_max - (int32_t)s->duty_min;
    int32_t angle_range = s->angle_max - s->angle_min;
    s->duty = (uint32_t)((int32_t)s->duty_min + duty_range * (angle_temp - s->angle_min) / angle_range);
    // 设置舵机占空比
    s->set_duty(s, s->duty);
}
