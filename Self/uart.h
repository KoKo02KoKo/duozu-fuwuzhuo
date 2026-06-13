/**
  ******************************************************************************
  * @file    uart.h
  * @brief   UART 极简驱动
  ******************************************************************************
  * 用法:
  *   1. uart_init(&uart, &huart, timeout_ms, max_len);
  *   2. 设置 uart.max_len 和 uart.timeout_ms
  *   3. 串口中断里调 uart.receive_byte(&uart, byte)
  *   4. rx_flag == 1 说明收到一包, 处理后清0
  ******************************************************************************
  */
#ifndef __UART_H__
#define __UART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <string.h>

#define UART_RX_BUF_SIZE    64

typedef struct uart_t uart_t;

extern uint8_t DEBUG;
extern uint32_t debug_last_tick;
extern uart_t debug_uart; // 用于调试打印
extern uart_t web_uart;   // 用于 ESP32 WiFi 模块
extern uart_t screen_uart; // 用于串口屏
extern uart_t gyro_uart;   // 用于陀螺仪

/* 函数指针类型 */
typedef void (*uart_send_char_fn)(uart_t *uart, uint8_t byte);
typedef void (*uart_send_str_fn)(uart_t *uart, const uint8_t *data, uint16_t len);
typedef void (*uart_recv_byte_fn)(uart_t *uart, uint8_t byte);
typedef uint8_t (*uart_is_received_fn)(uart_t *uart);

struct uart_t {
    //public:
    UART_HandleTypeDef *huart;              /* HAL UART 句柄 */
    uint8_t  rx_flag;                        /* =1 时表示收到一包, 用户处理后清0 */
    uint8_t  rx_buf[UART_RX_BUF_SIZE];       /* 接收缓冲区 */
    uint8_t  len;                            /* 完成包长度（超时时保存，供主循环使用） */
    //private:
    uint8_t  rx_pos;                         /* 当前已收字节数 */
    uint8_t  max_len;                        /* 期望接收字节数 (收满此数量即置 rx_flag) */
    uint8_t  rx_flag_last;                   /* 上次 rx_flag 的值 */
    uint8_t  rx_byte;                        /* HAL 单字节接收缓存 */
    uint32_t timeout_ms;                     /* 超时时间 (0=不超时) */
    uint32_t last_tick;                      /* 上次收到字节的时刻 */

    //public:
    uart_send_char_fn   send_char;             /* 发送单字节 */
    uart_send_str_fn    send_string;           /* 发送字符串 */
    uart_recv_byte_fn   receive_byte;          /* 接收字节 (放在串口中断里) */
    uart_is_received_fn is_received;            /* 查询是否收到一包 (内含超时判断) */
};

/* ========================== API ========================== */

void uart_init(uart_t *uart, UART_HandleTypeDef *huart, uint32_t timeout_ms, uint8_t max_len);

#ifdef __cplusplus
}
#endif

#endif /* __UART_H__ */
