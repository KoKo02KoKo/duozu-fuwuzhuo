#include "pid.h"

PID_TypeDef yaw_pid; // 定义一个 PID 实例

/// @brief Initialize the PID controller
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float out_min, float out_max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->integeral_max = 10.0f; // 防止积分过大
    pid->output_min = out_min;
    pid->output_max = out_max;
    pid->update = PID_Update;
}

/// @brief 增量式 PID 更新函数
float PID_Update(PID_TypeDef *pid, float target, float measurement, float dt)
{
    float error = target - measurement;
    if (error > 180.0f) error -= 360.0f;// 处理角度环绕
    if (error < -180.0f) error += 360.0f;// 处理角度环绕

    pid->integral += error * dt;
    if (pid->integral > pid->integeral_max) pid->integral = pid->integeral_max;// 防止积分过大
    if (pid->integral < -pid->integeral_max) pid->integral = -pid->integeral_max;// 防止积分过大

    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    if (output > pid->output_max) output = pid->output_max;// 输出限制
    if (output < pid->output_min) output = pid->output_min;// 输出限制

    return output;
}
