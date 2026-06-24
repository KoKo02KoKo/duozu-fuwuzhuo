#ifndef __OTHERS_H
#define __OTHERS_H

#include "main.h"
#include "usart.h"
#include "motor.h"
#include "angle.h"
#include "pid.h"

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

extern motor_mode_t mode;
extern MOTOR ml, mr; // 电机实例（定义在 motion.c）
extern ANGLE angle; // 角度实例
extern PID_TypeDef yaw_pid; // PID实例
extern int cc;
extern volatile uint8_t turn_completed_flag; // 转向完成防连发锁


extern int mode_HMI;

motor_mode_t get_motor_mode(void);





#endif /* __OTHERS_H */
