#ifndef __XIAOZHI_H
#define __XIAOZHI_H

#include "motion.h"        // motor_mode_t

/// @brief 根据小智 GPIO 输入引脚（PA4~PA7）获取运动模式
motor_mode_t get_mode_from_pins(void);

#endif /* __XIAOZHI_H */
