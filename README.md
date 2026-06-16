# 🦿 多足服务桌 (Duozu Service Desk)

> 基于 STM32H750 的多足服务机器人控制平台

| 项目 | 信息 |
|------|------|
| **版本** | v1.2_0616 |
| **主控** | STM32H750VBTx (Cortex-M7, 480MHz) |
| **IDE** | Keil MDK-ARM |
| **生成工具** | STM32CubeMX |
| **仓库** | [KoKo02KoKo/duozu-fuwuzhuo](https://github.com/KoKo02KoKo/duozu-fuwuzhuo) |

---

## 📁 项目结构

```
h750/
├── test.ioc                    # CubeMX 工程配置
├── Core/                       # HAL 层核心代码
│   ├── Inc/                    # main.h, gpio.h, tim.h, usart.h, stm32h7xx_hal_conf.h
│   └── Src/                    # main.c, gpio.c, tim.c, usart.c, stm32h7xx_it.c
├── Drivers/                    # STM32 官方驱动
│   ├── CMSIS/
│   └── STM32H7xx_HAL_Driver/
├── Self/                       # 👈 用户业务代码（本层）
│   ├── APP/                    # 应用层
│   │   ├── angle.c/h           # 角度控制
│   │   ├── common.c/h          # 公共初始化 & 主任务调度
│   │   ├── face_track.c/h      # 人脸追踪（PID + 舵机）
│   │   ├── obstacle.c/h        # 避障逻辑
│   │   ├── others.c/h          # 其他功能
│   │   ├── radar.c/h           # 雷达数据处理
│   │   └── tx_radar.c/h        # 雷达数据转发
│   └── BSP/                    # 板级支持包
│       ├── motor.c/h           # 电机驱动（PWM + 方向控制）
│       ├── pid.c/h             # PID 控制器
│       ├── servo.c/h           # 舵机驱动
│       └── uart.c/h            # 串口通信封装（环形缓冲 + 超时）
└── MDK-ARM/                    # Keil 工程文件
    ├── test.uvprojx            # 项目文件
    ├── common.c/h              # MDK 公共定义
    └── startup_stm32h750xx.s   # 启动文件
```

---

## 🔌 硬件接口

| 外设 | 连接设备 | 功能 |
|------|---------|------|
| USART1 | 调试串口 | 日志输出 (`debug_uart`) |
| USART2 | K230 AI 处理器 | 视觉数据通信 (`k230_uart`) |
| USART3 | 屏幕 | 人机交互 (`screen_uart`) |
| UART4 | 陀螺仪 (WT931) | 姿态角度读取 (`gyro_uart`) |
| UART5 | 预留 | — |
| USART6 | 雷达模块 | 障碍物检测 (`radar_uart`) |
| UART8 | 雷达转发 | 雷达数据透传 (`tx_radar_uart`) |
| TIM1 | 舵机 PWM (CH2/CH3) | `sh` (水平), `sl` (垂直) 舵机 |
| TIM4 | 电机 PWM (CH1/CH3/CH4) | `ml` (左电机), `mr` (右电机) |

---

## 🧠 功能模块

### 人脸追踪 (`face_track`)
- 解析 K230 返回的 `$23,06,x,y,w,h#` 协议帧
- 双轴 PID 控制舵机，实现目标居中追踪

### 避障 (`obstacle`)
- 雷达距离数据处理与避障决策

### 电机控制 (`motor`)
- PWM 调速 + GPIO 方向控制
- 带速度限幅保护

### 舵机控制 (`servo`)
- 角度范围限幅，脉冲宽度换算

### PID 控制 (`pid`)
- 位置式 PID，支持积分分离

### 陀螺仪 (`gyro`)
- WT931 协议：解锁 → 设置角度输出 → 校准 → 保存
- 用于机器人角度闭环控制

---

## 🚀 快速开始

1. 用 **Keil MDK-ARM** 打开 `MDK-ARM/test.uvprojx`
2. 编译（Project → Build）
3. 通过 ST-Link / J-Link 下载到 STM32H750VBTx
4. 系统启动后：
   - 自动初始化所有 UART、舵机、电机、PID
   - 主循环执行 `main_task()` 调度各模块

---

## 📝 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.2_0616 | 2026-06-16 | 目录重构为 APP/BSP 分层；新增 face_track、obstacle、radar、tx_radar 模块 |
| v1.1 | — | 基础功能：电机、舵机、PID、角度、UART |
| v1.0 | — | 初始 STM32CubeMX 工程搭建 |

---

## 📄 License

STM32CubeMX 生成代码遵循 STMicroelectronics 许可。用户代码部分保留所有权利。
