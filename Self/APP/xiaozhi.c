#include "xiaozhi.h"
#include "main.h"

/// @brief 根据小智 GPIO 输入引脚（PA4~PA7）获取运动模式
motor_mode_t get_mode_from_pins(void)
{
    motor_mode_t motor_mode = MODE_UNKNOWN;
    static motor_mode_t last_motor_mode = MODE_UNKNOWN;
    static motor_mode_t return_mode = MODE_UNKNOWN;
    // 一次性读取 4 个引脚，避免分步读导致跨模式混合值
    uint32_t idr = GPIOA->IDR;
    #define XZ_READ(pin) ((idr & (pin)) ? GPIO_PIN_SET : GPIO_PIN_RESET)
    GPIO_PinState p01 = XZ_READ(GPIO_PIN_4);
    GPIO_PinState p02 = XZ_READ(GPIO_PIN_5);
    GPIO_PinState p04 = XZ_READ(GPIO_PIN_6);
    GPIO_PinState p05 = XZ_READ(GPIO_PIN_7);   

    if (p01 == GPIO_PIN_SET && p02 == GPIO_PIN_SET && p04 == GPIO_PIN_SET && p05 == GPIO_PIN_RESET) motor_mode = MODE_FORWARD;//1110
    else if (p01 == GPIO_PIN_SET && p02 == GPIO_PIN_SET && p04 == GPIO_PIN_RESET && p05 == GPIO_PIN_SET) motor_mode = MODE_BACKWARD;//1101
    else if (p01 == GPIO_PIN_SET && p02 == GPIO_PIN_RESET && p04 == GPIO_PIN_RESET && p05 == GPIO_PIN_RESET) motor_mode = MODE_RIGHT;//1000
    else if (p01 == GPIO_PIN_RESET && p02 == GPIO_PIN_SET && p04 == GPIO_PIN_RESET && p05 == GPIO_PIN_RESET) motor_mode = MODE_LEFT;//0100
    else if (p01 == GPIO_PIN_RESET && p02 == GPIO_PIN_RESET && p04 == GPIO_PIN_SET && p05 == GPIO_PIN_RESET) motor_mode = MODE_TURN;//0010
    else if (p01 == GPIO_PIN_SET && p02 == GPIO_PIN_SET && p04 == GPIO_PIN_RESET && p05 == GPIO_PIN_RESET) motor_mode = MODE_CIRCLE;//1100
    else if (p01 == GPIO_PIN_RESET && p02 == GPIO_PIN_SET && p04 == GPIO_PIN_SET && p05 == GPIO_PIN_RESET) motor_mode = MODE_STOP;//0110
    else if (p01 == GPIO_PIN_SET && p02 == GPIO_PIN_SET && p04 == GPIO_PIN_SET && p05 == GPIO_PIN_SET) motor_mode = MODE_LIGHT_ON;//1111
    else if (p01 == GPIO_PIN_RESET && p02 == GPIO_PIN_RESET && p04 == GPIO_PIN_RESET && p05 == GPIO_PIN_SET) motor_mode = MODE_LIGHT_OFF;//0001
    else if (p01 == GPIO_PIN_SET && p02 == GPIO_PIN_RESET && p04 == GPIO_PIN_RESET && p05 == GPIO_PIN_SET) motor_mode = MODE_FAN_ON;//1001
    else if (p01 == GPIO_PIN_RESET && p02 == GPIO_PIN_SET && p04 == GPIO_PIN_SET && p05 == GPIO_PIN_SET) motor_mode = MODE_FAN_OFF;//0111
    else if (p01 == GPIO_PIN_SET && p02 == GPIO_PIN_RESET && p04 == GPIO_PIN_SET && p05 == GPIO_PIN_SET) motor_mode = MODE_NIGHT;//1011
    else motor_mode = MODE_UNKNOWN;
    // 检测到新模式（非UNKNOWN且与上次不同）→ 更新
    if (motor_mode != MODE_UNKNOWN && motor_mode != last_motor_mode)
        return_mode = motor_mode;
    //
    last_motor_mode = motor_mode;
    return return_mode;
}
