/**
  ******************************************************************************
  * @file    uart.c
  * @brief   UART 极简驱动实现
  ******************************************************************************
  */
#include "uart.h"
#include "usart.h" // 包含 HAL_UART_Transmit 和 HAL_GetTick 的声明
#include <string.h> // 包含 memset 和 strcmp 的声明
#include <stdio.h>  // 包含 printf 的声明
#include <stdlib.h>

/* ===== printf 重定向 (ARMCC Compiler 5) ===== */
#pragma import(__use_no_semihosting)   // 禁用半主机, 否则 printf 会卡死

struct __FILE { int handle; };
FILE __stdout;

// 禁用半主机后需要自己提供 _sys_exit
void _sys_exit(int return_code)
{
    (void)return_code;
    while (1);
}
/* ============================================ */

uart_t debug_uart; // 用于调试打印UART1    115200
uart_t k230_uart;   // 用于 K230 模块UART2 115200
uart_t screen_uart; // 用于串口屏UART3 115200
uart_t gyro_uart;   // 用于陀螺仪UART4 115200
uart_t web_uart;    // 用于 Web 模块UART5 115200
uart_t radar_uart;   // 用于雷达UART6 230400
uart_t tx_radar_uart;   // 用于发送雷达数据UART8 

/* ========================== 默认 receive_byte 实现 ========================== */

/**
 * @brief 默认接收处理 — 放在串口中断里调用
 *
 * 每收到一个字节:
 *   1. 存入 rx_buf, rx_pos++
 *   2. 记录 last_tick
 *   3. 如果 rx_pos >= max_len → rx_flag = 1 (收够了)
 */
static void default_receive_byte(uart_t *uart, uint8_t byte)
{
    uint32_t now = HAL_GetTick();

    /* ── 检测用户是否清过 rx_flag ── */
    /*    上次 rx_flag=1 这次=0 → 用户已读完数据, 重置接收位置 */
    if (uart->rx_flag_last == 1 && uart->rx_flag == 0)
    {
        uart->rx_pos = 0;
    }
    uart->rx_flag_last = uart->rx_flag;

    /* ── 超时检测: 距离上一字节超过 timeout_ms → 包完成 ── */
    uart->is_received(uart);

    /* ── 存入新字节 ── */
    if (uart->rx_pos < sizeof(uart->rx_buf))
        uart->rx_buf[uart->rx_pos++] = byte;

    uart->last_tick = now;

    /* 收够了指定数量 → 置标志（固定长度模式） */
    if (uart->max_len > 0 && uart->rx_pos >= uart->max_len)
    {
        uart->len = uart->max_len;
        uart->rx_flag = 1;
        uart->rx_flag_last = 1;                /* 让下个字节触发重置，开始新包 */
    }
}

/* ==================== 默认 is_received ==================== */

/**
 * @brief 默认 is_received 实现 - 检测超时后返回 rx_flag
 *
 * 每次调用时检查帧间超时:
 *   如果 timeout_ms>0 且 rx_pos>0 且距离 last_tick >= timeout_ms,
 *   则认为帧完成, 自动置 rx_flag=1.
 *
 * @param uart  UART 实例指针
 * @return uint8_t  rx_flag 的值 (1=收到一包, 0=未收完)
 */
static uint8_t default_is_received(uart_t *uart)
{
    uint32_t now = HAL_GetTick();

    /* ── 超时检测: 距离上一字节超过 timeout_ms → 包完成 ── */
    if (uart->timeout_ms > 0 && uart->rx_pos > 0 
        && (now - uart->last_tick >= uart->timeout_ms))
    {
        if (uart->rx_flag == 0)
        {
            uart->len = uart->rx_pos;          /* 保存完成包长度 */
            uart->rx_flag = 1;                 /* 通知主循环 */
            uart->rx_flag_last = 1;            /* 让下个字节触发重置，开始新包 */
        }
        uart->rx_pos = 0;                      /* 重置，此字节作为新包的第一个字节 */
    }
    if (uart->timeout_ms == 0 && uart->rx_pos > 0 
        && (now - uart->last_tick >= 10))
    {
        if (uart->rx_flag == 0)
        {
            uart->len = uart->rx_pos;          /* 保存完成包长度 */
            uart->rx_flag = 1;                 /* 通知主循环 */
            uart->rx_flag_last = 1;            /* 让下个字节触发重置，开始新包 */
        }
        uart->rx_pos = 0;                      /* 重置，此字节作为新包的第一个字节 */
    }

    return uart->rx_flag;
}

/* ========================== 默认 send_char 实现 ========================== */

static void default_send_char(uart_t *uart, uint8_t byte)
{
    //replace: 实际项目中这里调 HAL_UART_Transmit, 用户可自己替换
    if (uart->huart)
        HAL_UART_Transmit(uart->huart, &byte, 1, 100);
}

static void default_send_string(uart_t *uart, const uint8_t *data, uint16_t len)
{
    //replace: 实际项目中这里调 HAL_UART_Transmit, 用户可自己替换
    if (uart->huart && data && len > 0)
        HAL_UART_Transmit(uart->huart, (uint8_t *)data, len, 1000);
}

/* ========================== API ========================== */

void uart_init(uart_t *uart, UART_HandleTypeDef *huart, uint32_t timeout_ms, uint8_t max_len)
{
    memset(uart, 0, sizeof(uart_t));
    uart->huart = huart;
    uart->rx_flag = 0;
    uart->rx_pos = 0;
    uart->max_len = max_len;
    uart->timeout_ms = timeout_ms;
    uart->last_tick = 0;
    uart->rx_flag_last = 0;
    uart->send_char    = default_send_char;
    uart->send_string  = default_send_string;
    uart->receive_byte = default_receive_byte;
    uart->is_received  = default_is_received;

    if (huart)
    {
        //replace: 实际项目中这里调用 HAL_UART_Receive_IT 使能接收中断, 用户可自己替换
        HAL_UART_Receive_IT(huart, &uart->rx_byte, 1);
    }
}

/// @brief UART receive complete callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   //UART 1 用于调试打印
    if (huart->Instance == USART1)
    {
        debug_uart.receive_byte(&debug_uart, debug_uart.rx_byte);
        //replace: 实际项目中这里调用 HAL_UART_Receive_IT 使能接收中断, 用户可自己替换
        HAL_UART_Receive_IT(&huart1, &debug_uart.rx_byte, 1);
    }
    //UART 2 用于 K230 模块
    if (huart->Instance == USART2)
    {
        k230_uart.receive_byte(&k230_uart, k230_uart.rx_byte);
        //replace: 实际项目中这里调用 HAL_UART_Receive_IT 使能接收中断, 用户可自己替换
        HAL_UART_Receive_IT(&huart2, &k230_uart.rx_byte, 1);
    }
    //UART 3 用于串口屏
    if (huart->Instance == USART3)
    {
        screen_uart.receive_byte(&screen_uart, screen_uart.rx_byte);
        //replace: 实际项目中这里调用 HAL_UART_Receive_IT 使能接收中断, 用户可自己替换
        HAL_UART_Receive_IT(&huart3, &screen_uart.rx_byte, 1);
    }
    //UART 4 用于陀螺仪
    if (huart->Instance == UART4)
    {
        gyro_uart.receive_byte(&gyro_uart, gyro_uart.rx_byte);
        //replace: 实际项目中这里调用 HAL_UART_Receive_IT 使能接收中断, 用户可自己替换
        HAL_UART_Receive_IT(&huart4, &gyro_uart.rx_byte, 1);
    }
    //UART 5 用于 Web 模块
    if (huart->Instance == UART5)
    {
        web_uart.receive_byte(&web_uart, web_uart.rx_byte);
        //replace: 实际项目中这里调用 HAL_UART_Receive_IT 使能接收中断, 用户可自己替换
        HAL_UART_Receive_IT(&huart5, &web_uart.rx_byte, 1);
    }   
    //UART 6 用于雷达
    if (huart->Instance == USART6)
    {
        radar_uart.receive_byte(&radar_uart, radar_uart.rx_byte);
        //replace: 实际项目中这里调用 HAL_UART_Receive_IT 使能接收中断, 用户可自己替换
        HAL_UART_Receive_IT(&huart6, &radar_uart.rx_byte, 1);
    }
    //UART 8 用于发送雷达数据
    if (huart->Instance == UART8)
    {
        tx_radar_uart.receive_byte(&tx_radar_uart, tx_radar_uart.rx_byte);
        //replace: 实际项目中这里调用 HAL_UART_Receive_IT 使能接收中断, 用户可自己替换
        HAL_UART_Receive_IT(&huart8, &tx_radar_uart.rx_byte, 1);
    }
}

// 重定向 printf 到 debug_uart (不受 DEBUG 控制, 调用 printf 即输出)
int fputc(int ch, FILE *f)
{
    debug_uart.send_char(&debug_uart, (uint8_t)ch);
    return ch;
}
