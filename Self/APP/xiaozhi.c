#include "xiaozhi.h"
#include "main.h"

/// @brief 根据小智 GPIO 输入引脚（PA4~PA7）获取运动模式
///        p01(PA4)=bit3, p02(PA5)=bit2, p03(PA6)=bit1, p04(PA7)=bit0
///        0000=UNKNOWN  0001=FORWARD  0010=BACKWARD  ...  0110=STOP
motor_mode_t get_mode_from_pins(void)
{
    static motor_mode_t last_motor_mode = MODE_UNKNOWN;
    static motor_mode_t return_mode     = MODE_UNKNOWN;

    /* 一次性读取 4 个引脚 */
    uint32_t idr = GPIOA->IDR;
    uint8_t p01 = (idr & GPIO_PIN_4) ? 1 : 0;
    uint8_t p02 = (idr & GPIO_PIN_5) ? 1 : 0;
    uint8_t p03 = (idr & GPIO_PIN_6) ? 1 : 0;
    uint8_t p04 = (idr & GPIO_PIN_7) ? 1 : 0;

    /* 拼成 4-bit: p01=bit3, p02=bit2, p03=bit1, p04=bit0 */
    uint8_t pin_val = (p01 << 3) | (p02 << 2) | (p03 << 1) | (p04 << 0);

    motor_mode_t motor_mode;
    switch(pin_val){
        case 0: motor_mode = MODE_UNKNOWN;
            break;
        case 1: motor_mode = MODE_BACKWARD;
            break;
        case 2: motor_mode = MODE_RIGHT;
            break;
        case 3: motor_mode = MODE_LEFT;
            break;
        case 4: motor_mode = MODE_TURN;
            break;
        case 5: motor_mode = MODE_CIRCLE;
            break;
        case 6: motor_mode = MODE_STOP;
            break;
        case 7: motor_mode = MODE_FORWARD;
            break;
        default: motor_mode = MODE_UNKNOWN;
    }
    /* 检测到新模式（非UNKNOWN且与上次不同）→ 更新 */
    if (motor_mode != MODE_UNKNOWN && motor_mode != last_motor_mode)
        return_mode = motor_mode;

    last_motor_mode = motor_mode;
    return return_mode;
}
