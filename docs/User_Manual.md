# STM32F334 + DRV8870 电机控制与电流采样用户手册

本手册详细说明了基于 STM32F334 的四通道 DRV8870 直流电机控制系统的功能、使用方法与接口说明。项目使用 CubeMX 生成的基础代码，并增加了用户自定义驱动层，适用于 Keil MDK 开发环境。

## 1. 系统概述

### 1.1 硬件组成
- MCU: STM32F334C8T6 (72 MHz, ARM Cortex-M4F)
- 电机驱动: 4× DRV8870 (双输入控制)
- 电流采样: 4× INA180A2 (增益 50 V/V) + 10 mΩ 分流电阻
- 接口: USART1 (日志), I2C1 (可选扩展)

### 1.2 功能特性
- 四路 DRV8870 电机 PWM 控制 (1 kHz)
- 四通道同步电流采样 (TIM1 触发 + DMA)
- 可配置一阶低通滤波
- 串口日志功能
- 系统状态 LED 指示

### 1.3 文件结构
项目组织为以下结构：
```
Project/
├── Core/                          # CubeMX 生成的核心文件
│   ├── Inc/                       # 系统头文件
│   └── Src/                       # 系统源文件 (含 main.c)
├── Drivers/                       # HAL 库与芯片驱动
├── User/                          # 用户自定义代码
│   ├── Drivers/                   # 自定义驱动模块
│   │   ├── drv8870/               # 电机驱动
│   │   └── current_sense/         # 电流采样
│   └── Utils/                     # 工具函数
└── MDK-ARM/                       # Keil 项目文件
```

## 2. 快速入门

### 2.1 基本连接
1. **电源**
   - 接入 3.3V 给 MCU 和信号系统
   - 接入电机电源 (通常 12V) 给 DRV8870

2. **串口调试**
   - USART1: PB6 (TX), PB7 (RX)
   - 连接 USB-UART 转换器，波特率 115200, 8N1

3. **调试接口**
   - SWD: SWDIO, SWCLK (标准 SWD 接口)

### 2.2 默认配置
- 系统时钟: 72 MHz (HSE 8 MHz → PLL×9)
- PWM 频率: 1 kHz (可调整至更高值避免噪声)
- 电流采样: 1 kHz 同步采样 (与 PWM 周期同步)
- 低通滤波: 20 Hz 截止频率 (默认开启)

### 2.3 程序启动流程
1. 系统初始化 (HAL, 时钟, 外设)
2. 配置并启动四路 PWM (TIM1, TIM2)
3. 初始化电机 (默认速度设置)
4. 配置并启动 ADC2+DMA 电流采样
5. 进入主循环，定期打印电流信息

## 3. 编程接口参考

### 3.1 电机控制 (drv8870.h)

```c
// 初始化 (指定使用的定时器)
void MTD1_Init(TIM_HandleTypeDef* htim);
void MTD2_Init(TIM_HandleTypeDef* htim);
void MTD3_Init(TIM_HandleTypeDef* htim);
void MTD4_Init(TIM_HandleTypeDef* htim);

// 设置速度 (-100~+100 百分比, 正/负控制方向)
void MTD1_SetSpeedPercent(int8_t percent);
void MTD2_SetSpeedPercent(int8_t percent);
void MTD3_SetSpeedPercent(int8_t percent);
void MTD4_SetSpeedPercent(int8_t percent);

// 悬空/滑行 (两路输入都置 0)
void MTD1_Coast(void);
void MTD2_Coast(void);
void MTD3_Coast(void);
void MTD4_Coast(void);
```

#### 使用示例
```c
// 初始化电机 (在 main.c USER CODE 区域)
MTD1_Init(&htim2);  // MTD1/2 使用 TIM2
MTD2_Init(&htim2);
MTD3_Init(&htim1);  // MTD3/4 使用 TIM1
MTD4_Init(&htim1);

// 设置速度 (可在任意地方调用)
MTD1_SetSpeedPercent(+50);  // 正向 50% 占空比
MTD2_SetSpeedPercent(-30);  // 反向 30% 占空比
MTD3_Coast();               // 滑行/悬空
```

### 3.2 电流采样 (currentsense.h)

```c
// 初始化 (指定 ADC 句柄)
void CurrentSense_InitDMA(ADC_HandleTypeDef* hadc);

// 设置校准参数 (VREF, 分流电阻, 增益)
void CurrentSense_SetCalibration(CurrCalib_t c);

// 低通滤波配置
void CurrentSense_EnableLPF(uint8_t enable);
void CurrentSense_SetLPF_ByAlpha(float alpha);
void CurrentSense_SetLPF_ByCutoff(float fc_hz, float fs_hz);

// 数据访问 (全局变量)
extern volatile uint16_t g_curr_raw[4];     // 原始 ADC 值
extern volatile float    g_curr_amp[4];     // 计算后电流值 (A)
extern volatile float    g_curr_amp_filt[4]; // 滤波后电流值 (A)
```

#### 使用示例
```c
// 设置校准参数 (适配 INA180A2 + 10mΩ)
CurrentSense_SetCalibration((CurrCalib_t){
    .vref = 3.3f,    // 参考电压 (V)
    .rsense = 0.01f, // 分流电阻 (Ω)
    .gain = 50.0f    // INA180A2 增益
});

// 初始化 ADC2 DMA 采样
CurrentSense_InitDMA(&hadc2);

// 配置低通滤波 (20Hz 截止, 1kHz 采样)
CurrentSense_SetLPF_ByCutoff(20.0f, 1000.0f);
CurrentSense_EnableLPF(1);

// 读取电流值 (在任意地方)
float current1 = g_curr_amp_filt[0]; // 第一路滤波后电流
float current2 = g_curr_amp[1];      // 第二路原始电流
```

### 3.3 日志工具 (log.h)

```c
// 初始化 (指定使用的 UART)
void Log_Init(UART_HandleTypeDef* huart);

// 不同级别的日志输出
#define LOG_DEBUG(fmt, ...)
#define LOG_INFO(fmt, ...)
#define LOG_WARN(fmt, ...)
#define LOG_ERROR(fmt, ...)

// 格式化输出函数 (按日志级别)
void Log_Print(LogLevel_t level, const char* fmt, ...);
```

#### 使用示例
```c
// 初始化日志 (使用 UART1)
Log_Init(&huart1);

// 输出不同级别日志
LOG_INFO("系统启动, PWM=%dHz\r\n", 1000);
LOG_DEBUG("调试信息: %d, %f\r\n", value, fvalue);
LOG_WARN("电流过高: %.2fA\r\n", current);
LOG_ERROR("错误代码: %04X\r\n", error_code);
```

## 4. 实际应用指南

### 4.1 速度控制与换向
- 使用 `MTDx_SetSpeedPercent(+/-percent)` 控制速度和方向
- 速度范围: -100 (全速反向) 到 +100 (全速正向)
- 0 值会将电机设置为 COAST 模式 (两输入为低)
- 方向与实际旋转可能有差异，需根据接线调整正负号

### 4.2 电流监控与保护
建议实现的保护措施:
1. **过流检测**
   ```c
   for (int i = 0; i < 4; i++) {
     if (g_curr_amp_filt[i] > OVERCURRENT_THRESHOLD) {
       // 对应电机减速或停止
       MTDx_SetSpeedPercent(0); // 或降低占空比
       LOG_WARN("Motor %d overcurrent: %.2fA\r\n", i+1, g_curr_amp_filt[i]);
     }
   }
   ```

2. **堵转保护**
   ```c
   // 检测高电流 + 低转速状态持续时间
   if (g_curr_amp_filt[motor_idx] > STALL_CURRENT && stall_time > STALL_TIMEOUT) {
     MTDx_Coast(); // 停止输出
     LOG_ERROR("Motor %d stall detected\r\n", motor_idx+1);
   }
   ```

### 4.3 PWM 频率调整
若需改变 PWM 频率 (如提高到 20kHz 避免噪声):
1. 修改 CubeMX 中 TIM1 和 TIM2 的预分频器与周期
2. 重新生成代码
3. 更新低通滤波采样频率:
   ```c
   CurrentSense_SetLPF_ByCutoff(20.0f, 20000.0f); // 采样频率改为 20kHz
   ```

### 4.4 ADC 通道适配
若电路板 ADC 通道与默认不同:
1. 修改 `currentsense.h` 中的通道定义:
   ```c
   #define CUR1_ADC2_CHANNEL     ADC_CHANNEL_x  // 实际通道号
   #define CUR1_GPIO_Port        GPIOx
   #define CUR1_GPIO_Pin         GPIO_PIN_y
   ```
2. 确保 CubeMX 中 ADC2 的转换序列与上述定义一致

## 5. 常见问题与解决方案

### 5.1 电机相关

| 问题 | 可能原因 | 解决方案 |
|------|---------|---------|
| 电机不转 | PWM 未输出<br>驱动芯片未供电<br>接线错误 | 检查 PWM 波形<br>确认驱动电源<br>检查 IN1/IN2 连接 |
| 电机方向相反 | 定义与接线不符 | 取反速度参数<br>交换 IN1/IN2 接线 |
| 电机噪声大 | PWM 频率在可听范围 | 提高 PWM 频率到 20kHz 以上 |

### 5.2 电流采样相关

| 问题 | 可能原因 | 解决方案 |
|------|---------|---------|
| 电流值异常 | 校准参数错误<br>通道映射不正确<br>ADC 触发问题 | 检查 vref/gain/rsense<br>确认 ADC 通道定义<br>检查 TIM1 TRGO 设置 |
| 电流波动大 | 电源纹波<br>滤波不足<br>采样时机不合适 | 增加电源滤波<br>降低 LPF 截止频率<br>调整采样为 PWM 中点 |
| DMA 中断不触发 | 中断函数错误<br>NVIC 未配置 | 检查 it.c 中的处理函数<br>确认中断启用并配置优先级 |

### 5.3 通信与调试

| 问题 | 可能原因 | 解决方案 |
|------|---------|---------|
| 无日志输出 | 串口参数错误<br>接线问题<br>日志未初始化 | 确认波特率 115200<br>检查 TX/RX 连接<br>检查 Log_Init 调用 |
| 程序跑飞 | 栈溢出<br>中断嵌套问题<br>数组越界 | 增加堆栈大小<br>优化中断优先级<br>检查数组访问边界 |

## 6. 附录

### 6.1 PWM 通道映射
- MTD1: TIM2_CH1/CH2 (PA0/PA1)
- MTD2: TIM2_CH3/CH4 (PA2/PA3)
- MTD3: TIM1_CH1/CH2 (PA8/PA9)
- MTD4: TIM1_CH3/CH4 (PA10/PA11)

### 6.2 电流采样通道
- MTD1_CUR: ADC2_IN1 (PA4)
- MTD2_CUR: ADC2_IN2 (PA5)
- MTD3_CUR: ADC2_IN3 (PA6)
- MTD4_CUR: ADC2_IN13 (PB12)

### 6.3 参考资料
- STM32F334 参考手册 (RM0364)
- DRV8870 数据手册: https://www.ti.com/product/DRV8870
- INA180 数据手册: https://www.ti.com/product/INA180

### 6.4 更新日志
- v1.0.0 (2025-08-13): 初始版本
  - 基于 CubeMX 生成的 Keil 工程
  - 四路 DRV8870 驱动
  - 四通道电流采样与低通滤波

---

文档版本：1.0.0  
作者：GAARAHK  
最后更新：2025-08-13