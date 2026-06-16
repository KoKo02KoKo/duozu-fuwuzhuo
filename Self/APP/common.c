#include "common.h"

uint8_t DEBUG = 1;  // =1 开机即输出调试信息
uint8_t debug_flag = 0; // 用于控制调试信息输出频率
uint32_t debug_last_tick = 0;
uint8_t servo_open =1; // =1 允许操作舵机
uint8_t motor_open = 0; // =1 允许操作电机


void Init_Common(void)
{
  uart_init(&debug_uart, &huart1, 10, 20); // UART1 设置超时 10ms, 最大接收 20 字节
  uart_init(&k230_uart, &huart2, 10, 0); // UART2 设置超时 10ms, 不固定长度（k230数据帧长度可变）
  uart_init(&screen_uart, &huart3, 10, 0); // UART3 设置超时 10ms, 不固定长度
  uart_init(&gyro_uart, &huart4, 0, 11); // UART4 不使用超时, 最大接收 11 字节
  uart_init(&radar_uart, &huart6, 10, 0); // UART6 设置超时 10ms, 不固定长度
  uart_init(&tx_radar_uart, &huart8, 10, 0); // UART8 设置超时 10ms, 不固定长度
 
if(motor_open == 1)
  {
    motor_init(&ml, TIM_CHANNEL_3, GPIOD, GPIOA, GPIO_PIN_11, GPIO_PIN_1); // 初始化电机1
    motor_init(&mr, TIM_CHANNEL_4, GPIOD, GPIOD, GPIO_PIN_12, GPIO_PIN_13); // 初始化电机2

  }

  if(servo_open == 1)
  {
    servo_init(&sh, TIM_CHANNEL_3, 90, -50, 730, 200); // 初始化舵机1
    servo_init(&sl, TIM_CHANNEL_2, 90, -90, 1100, 400); // 初始化舵机2
    
    face_track_init();  // 初始化人脸追踪 PID
  }

  if(motor_open == 1)
  {
    angle_init(&angle);// 初始化角度结构体

    PID_Init(&yaw_pid, 2.0f, 0.005f, 0.5f, -50.0f, 50.0f); // 初始化 PID 控制器，参数需要根据实际情况调整
  }


  // 初始化陀螺仪，发送解锁、设置输出、校准和保存命令
  if(motor_open == 1)
  {
    const uint8_t gyro_unlock_cmd[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    const uint8_t gyro_angle_output_cmd[5] = {0xFF, 0xAA, 0x02, 0x08, 0x00};
    const uint8_t gyro_calibrate_cmd[5] = {0xFF, 0xAA, 0x01, 0x08, 0x00};
    const uint8_t gyro_save_cmd[5] = {0xFF, 0xAA, 0x00, 0x00, 0x00};

    gyro_uart.send_string(&gyro_uart, gyro_unlock_cmd, sizeof(gyro_unlock_cmd));
    HAL_Delay(200);
    gyro_uart.send_string(&gyro_uart, gyro_angle_output_cmd, sizeof(gyro_angle_output_cmd));
    HAL_Delay(200);
    gyro_uart.send_string(&gyro_uart, gyro_calibrate_cmd, sizeof(gyro_calibrate_cmd));
    HAL_Delay(3000);
    gyro_uart.send_string(&gyro_uart, gyro_save_cmd, sizeof(gyro_save_cmd));
  }
}

void main_task()
{
    // 这里是主循环的代码，可以放一些周期性执行的任务
    // 0. 每10ms输出一次调试信息
    if(DEBUG && (HAL_GetTick() - debug_last_tick >= 100))
    {
      debug_last_tick = HAL_GetTick();
      printf("DEBUG=%d SERVO=%d\r\n", DEBUG, servo_open);
      printf("track_x_pid: Kp=%.3f Ki=%.5f Kd=%.3f\r\n", track_x_pid.Kp, track_x_pid.Ki, track_x_pid.Kd);
      printf("track_y_pid: Kp=%.3f Ki=%.5f Kd=%.3f\r\n", track_y_pid.Kp, track_y_pid.Ki, track_y_pid.Kd);

    }
    // 1.debug_uart 接收 "DEBUG=1" 或 "DEBUG=0" 来控制 DEBUG 模式开关
    if(debug_uart.is_received(&debug_uart))
    {
      debug_uart.rx_flag = 0;
      strcmp((char*)debug_uart.rx_buf, "DEBUG=1") == 0 ? (DEBUG = 1) : 0; // 收到 "DEBUG=1" 则开启 DEBUG 模式
      strcmp((char*)debug_uart.rx_buf, "DEBUG=0") == 0 ? (DEBUG = 0) : 0; // 收到 "DEBUG=0" 则关闭 DEBUG 模式
      strcmp((char*)debug_uart.rx_buf, "SERVO=1") == 0 ? (servo_open = 1) : 0; // 收到 "SERVO=1" 则开启舵机控制
      strcmp((char*)debug_uart.rx_buf, "SERVO=0") == 0 ? (servo_open = 0) : 0; // 收到 "SERVO=0" 则关闭舵机控制
        strcmp((char*)debug_uart.rx_buf, "p+") == 0 ? (track_y_pid.Kp += 0.01f) : 0; // 收到 "p+" 则增加垂直 PID 比例增益
        strcmp((char*)debug_uart.rx_buf, "p-") == 0 ? (track_y_pid.Kp -= 0.01f) : 0; // 收到 "p-" 则减少垂直 PID 比例增益
        strcmp((char*)debug_uart.rx_buf, "i+") == 0 ? (track_y_pid.Ki += 0.001f) : 0; // 收到 "i+" 则增加垂直 PID 积分增益
        strcmp((char*)debug_uart.rx_buf, "i-") == 0 ? (track_y_pid.Ki -= 0.001f) : 0; // 收到 "i-" 则减少垂直 PID 积分增益
        strcmp((char*)debug_uart.rx_buf, "d+") == 0 ? (track_y_pid.Kd += 0.001f) : 0; // 收到 "d+" 则增加垂直 PID 微分增益
        strcmp((char*)debug_uart.rx_buf, "d-") == 0 ? (track_y_pid.Kd -= 0.001f) : 0; // 收到 "d-" 则减少垂直 PID 微分增益

    }
    // 2.k230_uart 接收人脸追踪数据帧，解析并更新 face_data 结构体
    if( servo_open == 1 && k230_uart.is_received(&k230_uart))
    {
      k230_uart.rx_flag = 0;
      face_track_parse(k230_uart.rx_buf, k230_uart.len);  // 解析人脸追踪帧
    }
    // 3. screen_uart 接收数据帧，解析并更新相关状态（如果有屏幕交互需求的话）
    if(motor_open == 1 && screen_uart.is_received(&screen_uart))
    {
      screen_uart.rx_flag = 0; 
    }
    //4. 陀螺仪串口的处理逻辑是解析数据帧并更新 angle 结构体，具体解析函数在 angle.c 中实现
    if(motor_open == 1 && gyro_uart.is_received(&gyro_uart))
    {
        // 解析陀螺仪数据帧，更新 angle 结构体
      angle_update_from_jy61p_frame(&angle, gyro_uart.rx_buf, gyro_uart.len);
      gyro_uart.rx_flag = 0;
    }
    //5. 接受到雷达数据帧，解析并更新相关状态（如果有雷达交互需求的话）
    if(motor_open == 1 && radar_uart.is_received(&radar_uart))
    {
      radar_uart.rx_flag = 0; 
      // 这里可以添加解析雷达数据帧的代码，更新相关状态变量

      //
    }
    //6. 接受到发送雷达数据帧，解析并更新相关状态（如果有雷达交互需求的话）
    if(motor_open == 1 && tx_radar_uart.is_received(&tx_radar_uart))
    {
      tx_radar_uart.rx_flag = 0;
      // 这里可以添加解析发送雷达数据帧的代码，更新相关状态变量

      //
    }
    // 7. 设置舵机角度
    if(servo_open == 1)
    {
      face_track_update();  // 人脸追踪 PID 驱动 sl sh舵机
      
      sh.set_angle(&sh, sh.angle); // 设置舵机1角度为90度
      sl.set_angle(&sl, sl.angle); // 设置舵机2角度为90度
    }
    // 8. 设置电机速度
    if(motor_open == 1)
    {
      ml.set_speed(&ml, 10); // 设置电机1速度为10 
      mr.set_speed(&mr, -10); // 设置电机2速度为-10
    }
}
