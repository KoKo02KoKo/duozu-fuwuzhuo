#include "others.h"
#include "obstacle.h"
#include "angle.h"
#include "motion.h"

/* ===================== 内部辅助函数 ===================== */

/// @brief 限幅到 [-100, 100]
static int clamp_speed(int v)
{
    if (v > 100)  return 100;
    if (v < -100) return -100;
    return v;
}


motor_mode_t mode = MODE_UNKNOWN;
int cc = 0;
volatile uint8_t turn_completed_flag = 0; // 转向完成防连发锁
int mode_HMI = 12; // 默认空闲状态
volatile uint8_t mode_HMI_flag = 0; // HMI 模式标志位


//__________识别小智输入信号，输出运动指令————————————//
/// @brief Get the motor mode based on the state of the input pins
motor_mode_t get_mode_from_pins(void)
{
    //replace
    GPIO_PinState p01, p02, p04, p05;
    p01 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);   //PA0
    p02 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);   //PA1
    p04 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);   //PA4
    p05 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);   //PA5
    
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

// ——————————根据运动指令做出响应——————————————//
void mode_execute(motor_mode_t mode)
{
    static motor_mode_t last_mode = MODE_UNKNOWN;  // 记录上一次模式

    // 模式没变 → 不需要重新设置 motion 参数，避免反复复位 PID
    if (mode == last_mode) {
        return;
    }
    last_mode = mode;

    switch (mode)
    {
        case MODE_FORWARD:
            motion.enable(&motion, 0);
            motion.set_speed(&motion, 40);
            motion.set_target_yaw(&motion, angle.yaw);
            motion.enable(&motion, 1);
            break;
        case MODE_BACKWARD:
            motion.enable(&motion, 0);
            motion.set_speed(&motion, -40);
            motion.set_target_yaw(&motion, angle.yaw);
            motion.enable(&motion, 1);
            break;
        case MODE_RIGHT:
            motion.enable(&motion, 0);
            {
                float target = angle.yaw - 90.0f;
                if (target < -180.0f) target += 360.0f;
                motion.set_target_yaw(&motion, target);
            }
            motion.set_speed(&motion, 40);
            motion.enable(&motion, 1);
            break;
        case MODE_LEFT:
            motion.enable(&motion, 0);
            {
                float target = angle.yaw + 90.0f;
                if (target > 180.0f) target -= 360.0f;
                motion.set_target_yaw(&motion, target);
            }
            motion.set_speed(&motion, 40);
            motion.enable(&motion, 1);
            break;
        case MODE_TURN:
            motion.enable(&motion, 0);
            {
                float target = angle.yaw + 180.0f;
                if (target > 180.0f) target -= 360.0f;
                motion.set_target_yaw(&motion, target);
            }
            motion.set_speed(&motion, 40);
            motion.enable(&motion, 1);
            break;
        case MODE_CIRCLE:
            motion.enable(&motion, 0);
            ml.set_speed(&ml, 40);
            mr.set_speed(&mr, -40);
            break;
        case MODE_UNKNOWN:
        default:
            motion.enable(&motion, 0);
            break;
    }
}


