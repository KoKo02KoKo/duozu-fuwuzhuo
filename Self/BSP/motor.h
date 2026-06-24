#ifndef _MOTOR_H
#define _MOTOR_H

#include "tim.h"

typedef struct MOTOR_ MOTOR;

/// @brief Motor structure definition
struct MOTOR_
{
    //public:
    int speed;
    //private:
    int speed_max;
    uint32_t duty;
    uint32_t duty_max;
    unsigned int channel; // PWM通道
    uint16_t dir_a_pin;// 方向控制引脚
    uint16_t dir_b_pin; // 方向控制引脚
    GPIO_TypeDef* dir_a_port; // 方向控制端口
    GPIO_TypeDef* dir_b_port; // 方向控制端口
    //public:
    void (*set_speed)(struct MOTOR_ *m, int speed);
    //private:
    void (*set_duty)(struct MOTOR_ *m, uint32_t duty);
    void (*set_direction)(struct MOTOR_ *m, char direction);
};

void motor_init(MOTOR *m,unsigned int channel,GPIO_TypeDef* dir_a_port, GPIO_TypeDef* dir_b_port, uint16_t dir_a_pin, uint16_t dir_b_pin);
void motor_Set_Duty(MOTOR *m, uint32_t duty);
void motor_set_direction(MOTOR *m, char direction);
void motor_set_speed(MOTOR *m, int speed);

#endif
