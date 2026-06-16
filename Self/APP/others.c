#include "others.h"

motor_mode_t mode = MODE_UNKNOWN;
MOTOR motor_a, motor_b; // 电机实例
ANGLE angle; // 角度实例
PID_TypeDef yaw_pid; // PID实例
int cc = 0;
volatile uint8_t turn_completed_flag = 0; // 转向完成防连发锁
uint8_t gyro_rx_byte = 0;// 陀螺仪串口单字节接收缓存与解析状态机
uint8_t screen_rx_byte = 0;// 串口屏单字节接收缓存与解析状态机
uint8_t web_rx_byte = 0;// ESP32 wifi单字节接收缓存与解析状态机
int mode_HMI = 12; // 默认空闲状态






/// @brief Get the motor mode based on the state of the input pins
motor_mode_t get_mode_from_pins(void)
{
    //replace
    GPIO_PinState p01, p02, p04, p05;
    p01 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);   
    p02 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);   
    p04 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);   
    p05 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);   
    
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

void mode_execute(motor_mode_t mode, MOTOR *ma, MOTOR *mb, ANGLE *angle)
{
    switch (mode)
    {
        case MODE_FORWARD:
            ma->speed = 80;
            mb->speed = 80;
            if(angle->control_enable == 0) angle->target_yaw = angle->yaw;
            angle->control_enable = 1;              
            break;
        case MODE_BACKWARD:
            ma->speed = -80;
            mb->speed = -80;
            if(angle->control_enable == 0) angle->target_yaw = angle->yaw;
            angle->control_enable = 1;
            break;
        case MODE_RIGHT:
            ma->speed = 0;
            mb->speed = 80;
            if(angle->control_enable == 0) {
                angle->target_yaw = angle->yaw - 90.0f;
                if (angle->target_yaw < -180.0f) angle->target_yaw += 360.0f;
            }
            angle->control_enable = 1;
            break;
        case MODE_LEFT:
            ma->speed = 80;
            mb->speed = 0;
            if(angle->control_enable == 0) {
                angle->target_yaw = angle->yaw + 90.0f;
                if (angle->target_yaw > 180.0f) angle->target_yaw -= 360.0f;
            }
            angle->control_enable = 1;
            break;
        case MODE_TURN:
            ma->speed = 0;
            mb->speed = 0;
            if(angle->control_enable == 0) {
                angle->target_yaw = angle->yaw + 180.0f;
                if (angle->target_yaw > 180.0f) angle->target_yaw -= 360.0f;
            }
            angle->control_enable = 1;
            break;
        case MODE_CIRCLE:
            ma->speed = 80;
            mb->speed = -80;
            angle->control_enable = 0;   
            break;
        case MODE_UNKNOWN:
			ma->speed = 0;
            mb->speed = 0;
            angle->control_enable = 0;
				
            break;
				
        default:
            ma->speed = 0;
            mb->speed = 0;
            angle->control_enable = 0;
            break;
    }
    ma->set_speed(ma, ma->speed);
    mb->set_speed(mb, mb->speed);
}

/// @brief Screen UART receive interrupt service routine (处理串口屏发送的控制命令)
void UART_Screen_Rx_ISR()
{    
    uint8_t data = screen_rx_byte;   // 获取单字节数据
    // 1. 基础控制模式 (0-10)
    if (data >= 0 && data <= 10  ) 
    {
        mode_HMI = data; 
    }
	else if (data > 10) 
    {
        // --- 核心：13-16 转发给 ESP32音乐播放器 ---
        uint8_t music_cmd = 0;
        if (data == 13) music_cmd = 'P';      // 播放
        else if (data == 14) music_cmd = 'O'; // 暂停
        else if (data == 15) music_cmd = '+'; // 下一首
        else if (data == 16) music_cmd = '-'; // 上一首
        if (music_cmd != 0) 
        {
            // 通过 USART2 发送给 ESP32音乐播放器 //replace
            HAL_UART_Transmit(&huart2, &music_cmd, 1, 10);
        } 
    }
}

/// @brief ESP32 WiFi UART receive interrupt service routine (处理来自ESP32的WiFi命令)
void UART_Web_Rx_ISR()
{
    // 这里可以处理来自ESP32的WiFi命令，例如接收新的控制模式或参数
    uint8_t data = web_rx_byte;   // 获取单字节数据
  
    // 1. 基础控制模式 (0-10)
    if (data >= 0 && data <= 10  ) 
    {
        mode_HMI = data; 
    }
    else if (rx_esp_byte > 10) 
    {
       // --- 核心：13-16 转发给 ESP32音乐播放器 ---
        uint8_t music_cmd = 0;
        if (data == 13) music_cmd = 'P';      // 播放
        else if (data == 14) music_cmd = 'O'; // 暂停
        else if (data == 15) music_cmd = '+'; // 下一首
        else if (data == 16) music_cmd = '-'; // 上一首
        if (music_cmd != 0) 
        {
            // 通过 USART2 发送给 ESP32音乐播放器 //replace
            HAL_UART_Transmit(&huart2, &music_cmd, 1, 10);
        } 
    }
}



/// @brief LED control procedure (根据当前模式控制LED灯效)
void LED_Proc()
{
    switch (mode)
      {
          case MODE_LIGHT_ON:
              RGB_All_Off();   // 先关闭
              Effect_RainbowFlow(5);
			//Effect_Breathing(RGBColor_TypeDef c, uint8_t speed)
				// 耗时任务安全执行
              break;
          case MODE_LIGHT_OFF:
              RGB_All_Off();
              break;
          case MODE_FAN_ON:
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
              break;
          case MODE_FAN_OFF:
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
              break;
          case MODE_NIGHT:
              RGB_All_Off();
              break;
          default:
              break;
      }
     
}

/// @brief TIM4 interrupt service routine (主控制循环，执行当前模式的动作，并调用角度控制环)
void TIM4_ISR()
{
    int a, c = 0;
    a = get_mode_from_pins();
    if(a != 12)
	{
        if(a == 0)mode = 0;
        if(a == 1)mode = 1;
        if(a == 2)mode = 2;
        if(a == 3)mode = 3;
        if(a == 4)mode = 4;
        if(a == 5)mode = 5;
        if(a == 6)mode = 6;
        if(a == 7)mode = 7;
        if(a == 8)mode = 8;
        if(a == 9)mode = 9;
        if(a == 10)mode = 10;
        if(a == 11)mode = 11;
        a = 12;
    }
	if(mode_HMI != 12)
	{
        if(mode_HMI == 0)mode = 0;
        if(mode_HMI == 1)mode = 1;
        if(mode_HMI == 2)mode = 2;
        if(mode_HMI == 3)mode = 3;
        if(mode_HMI == 4)mode = 4;
        if(mode_HMI == 5)mode = 5;
        if(mode_HMI == 6)mode = 6;
        if(mode_HMI == 7)mode = 7;
        if(mode_HMI == 8)mode = 8;
        if(mode_HMI == 9)mode = 9;
        if(mode_HMI == 10)mode = 10;
        if(mode_HMI == 11)mode = 11;
        mode_HMI = 12;
	}
	
    if(mode == 12)  mode_HMI_flag = 1;
	  c++;
	if(c > 200000){
    c = 0;
		
		if(mode  == 0 || mode  == 1 || mode  == 5  ){
		mode = 12;
		a= 12;
		mode_HMI = 12;
		}
	}
	
	mode_execute((motor_mode_t)mode);
    float dt = 0.000125f;   //TIM4中断周期为125us
    AngleControl_Loop(dt);
}