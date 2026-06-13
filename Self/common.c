#include "common.h"


void Init_Common(void)
{
  uart_init(&debug_uart, &huart1, 10, 20); // 设置超时 10ms, 最大接收 20 字节
  uart_init(&web_uart, &huart2, 10, 10); // 设置超时 10ms, 最大接收 10 字节
  uart_init(&screen_uart, &huart3, 10, 0); // 设置超时 10ms, 不固定长度
  uart_init(&gyro_uart, &huart4, 0, 11); // 不使用超时, 最大接收 11 字节

  motor_init(&ml, TIM_CHANNEL_3, GPIOD, GPIOA, GPIO_PIN_11, GPIO_PIN_1); // 初始化电机1
  motor_init(&mr, TIM_CHANNEL_4, GPIOD, GPIOD, GPIO_PIN_12, GPIO_PIN_13); // 初始化电机2

  angle_init(&angle);// 初始化角度结构体

  PID_Init(&yaw_pid, 2.0f, 0.005f, 0.5f, -50.0f, 50.0f); // 初始化 PID 控制器，参数需要根据实际情况调整

  // 初始化陀螺仪，发送解锁、设置输出、校准和保存命令
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

void main_task()
{
    // 这里是主循环的代码，可以放一些周期性执行的任务
    // 0. 每10ms输出一次调试信息
    if(DEBUG && (HAL_GetTick() - debug_last_tick >= 10))
    {
      debug_last_tick = HAL_GetTick();
      debug_uart.send_string(&debug_uart, (uint8_t*)"DEBUG MODE\r\n", 11);
      printf("Angle: roll=%.2f, pitch=%.2f, yaw=%.2f\r\n", angle.roll, angle.pitch, angle.yaw);
    }
    // 1. 处理串口数据
    // debug_uart 接收 "DEBUG=1" 或 "DEBUG=0" 来控制 DEBUG 模式开关
    if(debug_uart.is_received(&debug_uart))
    {
      debug_uart.rx_flag = 0;
      strcmp((char*)debug_uart.rx_buf, "DEBUG=1") == 0 ? (DEBUG = 1) : 0; // 收到 "DEBUG=1" 则开启 DEBUG 模式
      strcmp((char*)debug_uart.rx_buf, "DEBUG=0") == 0 ? (DEBUG = 0) : 0; // 收到 "DEBUG=0" 则关闭 DEBUG 模式
    }
    // web_uart 和 screen_uart 的数据处理逻辑可以根据实际需求来写，这里暂时只是清除接收标志
    if(web_uart.is_received(&web_uart))
    {
      web_uart.rx_flag = 0;
    }
    if(screen_uart.is_received(&screen_uart))
    {
      screen_uart.rx_flag = 0; 
    }
    // 陀螺仪串口的处理逻辑是解析数据帧并更新 angle 结构体，具体解析函数在 angle.c 中实现
    if(gyro_uart.is_received(&gyro_uart))
    {
        // 解析陀螺仪数据帧，更新 angle 结构体
      angle_update_from_jy61p_frame(&angle, gyro_uart.rx_buf, gyro_uart.len);
      gyro_uart.rx_flag = 0;
    }

    ml.set_speed(&ml, 10); // 设置电机1速度为10 
    mr.set_speed(&mr, -10); // 设置电机2速度为90
}
