#include "common.h"

/// @brief Motor structure definition
typedef struct MOTOR_
{
    //public:
    int speed;
    //private:
    int speed_max;
    uint32_t period;
    uint32_t period_max;
    //public:
    void (*set_speed)(MOTOR *m, int speed);
} MOTOR;

/// @brief Initialize the motor
void motor_init(MOTOR *m, int speed)
{
    m->speed = speed;
    //replace start
    m->speed_max = 100;
    m->period = 0;
    m->period_max = 1000;
    //replace end
    m->set_speed = motor_set_speed;
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

    m->period = (m->period_max * speed) / m->speed_max; 
    //Set the motor period //replace start
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm_a);
    //Set the motor direction
    if (speed >= 0)
    {
        // Set direction to forward //replace start
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    }
    else
    {
        // Set direction to reverse //replace start
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    }
}

/// @brief Get the motor mode from the input pins
typedef enum {
    MODE_FORWARD   = 0, 
    MODE_BACKWARD  = 1,   
    MODE_RIGHT     = 2,   
    MODE_LEFT      = 3,  
    MODE_TURN      = 4,   
    MODE_CIRCLE    = 5,   
    MODE_STOP      = 6,   
    MODE_LIGHT_ON  = 7,   
    MODE_LIGHT_OFF = 8,  
    MODE_FAN_ON    = 9,   
    MODE_FAN_OFF   = 10,  
    MODE_NIGHT     = 11,  
    MODE_UNKNOWN   = 12   
} motor_mode_t;

/// @brief Get the motor mode based on the state of the input pins
motor_mode_t get_mode_from_pins(void)
{
    //replace start
    GPIO_PinState p01, p02, p04, p05;

    p01 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);   
    p02 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);   
    p04 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);   
    p05 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);   
    //replace end

    if (p01 == GPIO_PIN_SET && p02 == GPIO_PIN_SET && p04 == GPIO_PIN_SET && p05 == GPIO_PIN_RESET) return MODE_FORWARD;
    else if (p01 == GPIO_PIN_SET && p02 == GPIO_PIN_SET && p04 == GPIO_PIN_RESET && p05 == GPIO_PIN_SET) return MODE_BACKWARD;
    else if (p01 == GPIO_PIN_SET && p02 == GPIO_PIN_RESET && p04 == GPIO_PIN_RESET && p05 == GPIO_PIN_RESET) return MODE_RIGHT;
    else if (p01 == GPIO_PIN_RESET && p02 == GPIO_PIN_SET && p04 == GPIO_PIN_RESET && p05 == GPIO_PIN_RESET) return MODE_LEFT;
    else if (p01 == GPIO_PIN_RESET && p02 == GPIO_PIN_RESET && p04 == GPIO_PIN_SET && p05 == GPIO_PIN_RESET) return MODE_TURN;
    else if (p01 == GPIO_PIN_SET && p02 == GPIO_PIN_SET && p04 == GPIO_PIN_RESET && p05 == GPIO_PIN_RESET) return MODE_CIRCLE;
    else if (p01 == GPIO_PIN_RESET && p02 == GPIO_PIN_SET && p04 == GPIO_PIN_SET && p05 == GPIO_PIN_RESET) return MODE_STOP;
    else if (p01 == GPIO_PIN_SET && p02 == GPIO_PIN_SET && p04 == GPIO_PIN_SET && p05 == GPIO_PIN_SET) return MODE_LIGHT_ON;
    else if (p01 == GPIO_PIN_RESET && p02 == GPIO_PIN_RESET && p04 == GPIO_PIN_RESET && p05 == GPIO_PIN_SET) return MODE_LIGHT_OFF;
    else if (p01 == GPIO_PIN_SET && p02 == GPIO_PIN_RESET && p04 == GPIO_PIN_RESET && p05 == GPIO_PIN_SET) return MODE_FAN_ON;
    else if (p01 == GPIO_PIN_RESET && p02 == GPIO_PIN_SET && p04 == GPIO_PIN_SET && p05 == GPIO_PIN_SET) return MODE_FAN_OFF;
    else if (p01 == GPIO_PIN_SET && p02 == GPIO_PIN_RESET && p04 == GPIO_PIN_SET && p05 == GPIO_PIN_SET) return MODE_NIGHT;
    else return MODE_UNKNOWN;
}

void motor_execute(motor_mode_t mode, )
{
    switch (mode)
    {
        case MODE_FORWARD:
            base_speed_left = 80;
            base_speed_right = 80;
            if(angle_control_enabled == 0) target_yaw = angle_yaw;
            angle_control_enabled = 1;              
            break;
        case MODE_BACKWARD:
            base_speed_left = -80;
            base_speed_right = -80;
            if(angle_control_enabled == 0) target_yaw = angle_yaw;
            angle_control_enabled = 1;
            break;
        case MODE_RIGHT:
            base_speed_left = 0;   // 必须为0，完全由PID差速转向
            base_speed_right = 0;
            if(angle_control_enabled == 0) {
                target_yaw = angle_yaw - 90.0f;
                if (target_yaw < -180.0f) target_yaw += 360.0f;
            }
            angle_control_enabled = 1;
            break;
        case MODE_LEFT:
            base_speed_left = 0;   // 必须为0
            base_speed_right = 0;
            if(angle_control_enabled == 0) {
                target_yaw = angle_yaw + 90.0f;
                if (target_yaw > 180.0f) target_yaw -= 360.0f;
            }
            angle_control_enabled = 1;
            break;
        case MODE_TURN:
            base_speed_left = 0;   // 必须为0
            base_speed_right = 0;
            if(angle_control_enabled == 0) {
                target_yaw = angle_yaw + 180.0f;
                if (target_yaw > 180.0f) target_yaw -= 360.0f;
            }
            angle_control_enabled = 1;
            break;
        case MODE_CIRCLE:
            base_speed_left = 80;
            base_speed_right = -80;
            angle_control_enabled = 0;   
            break;
        case MODE_UNKNOWN:
						base_speed_left = 0;
            base_speed_right = 0;
            angle_control_enabled = 0;
				
            break;
				
        default:
            base_speed_left = 0;
            base_speed_right = 0;
            angle_control_enabled = 0;
            break;
    }
}