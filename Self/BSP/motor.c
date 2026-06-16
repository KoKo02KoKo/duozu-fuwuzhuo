#include "motor.h"
#include <math.h>
#include <stdlib.h>

MOTOR ml; // 定义一个 MOTOR 实例
MOTOR mr; // 定义一个 MOTOR 实例

/// @brief Initialize the motor
void motor_init(MOTOR *m,unsigned int channel,GPIO_TypeDef* dir_a_port, GPIO_TypeDef* dir_b_port, uint16_t dir_a_pin, uint16_t dir_b_pin)
{
    m->speed = 0;
    m->speed_max = 100;
    m->duty = 0;
    m->duty_max = 100;
    m->channel = channel;
    m->dir_a_port = dir_a_port;
    m->dir_b_port = dir_b_port;
    m->dir_a_pin = dir_a_pin;
    m->dir_b_pin = dir_b_pin;
    m->set_speed = motor_set_speed;
    m->set_duty = motor_Set_Duty;
    m->set_direction = motor_set_direction;

    #ifndef PWM_STARTED
    #define PWM_STARTED
    // 启动PWM输出 //replace
    HAL_TIM_PWM_Init(&htim2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); 
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); 
    HAL_TIM_Base_Start_IT(&htim2);
    #endif
}

/// @brief Set the motor duty cycle
void motor_Set_Duty(MOTOR *m, uint32_t duty)
{
    //replace
    __HAL_TIM_SET_COMPARE(&htim2, m->channel, duty);
}

/// @brief Set the motor direction
void motor_set_direction(MOTOR *m, char direction)
{
    if(direction == 1) // Forward
    {
        //replace
        HAL_GPIO_WritePin(m->dir_a_port, m->dir_a_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(m->dir_b_port, m->dir_b_pin, GPIO_PIN_RESET);
    }
    else if(direction == 0) // Backward
    {
        //replace
        HAL_GPIO_WritePin(m->dir_a_port, m->dir_a_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(m->dir_b_port, m->dir_b_pin, GPIO_PIN_SET);
    }
}

/// @brief Set the motor speed
void motor_set_speed(MOTOR *m, int speed)
{
    // Limit the speed to the maximum allowed
    if(speed > m->speed_max) 
        m->speed = m->speed_max;
    else if(speed < -m->speed_max) 
        m->speed = - m->speed_max;
    else
        m->speed = speed;

    m->duty = abs((m->duty_max * speed) / m->speed_max); 
    //Set the motor duty cycle
    m->set_duty(m, m->duty);
    // Set the motor direction
    if(speed >= 0)
        m->set_direction(m, 1); // Forward
    else
        m->set_direction(m, 0); // Backward
}
