#ifndef _PID_H
#define _PID_H

typedef struct PID_TPEDEF  PID_TypeDef;

/// @brief PID controller structure definition
struct PID_TPEDEF
{
    //public:
    float Kp;
    float Ki;
    float Kd;
    //private:
    float integral;
    float integeral_max;
    float prev_error;
    float output_min;
    float output_max;
    //public:
    float (*update)(PID_TypeDef *pid, float target, float measurement, float dt);
};

void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float out_min, float out_max);
float PID_Update(PID_TypeDef *pid, float target, float measurement, float dt);
float PID_Update_Linear(PID_TypeDef *pid, float target, float measurement, float dt);  // 线性 PID，不做角度环绕

#endif
