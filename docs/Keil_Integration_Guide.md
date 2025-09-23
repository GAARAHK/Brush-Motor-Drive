# STM32F334 + DRV8870 项目 Keil MDK 集成指南

本文档提供了详细的步骤，指导如何使用 STM32CubeMX 在 Keil MDK 环境下创建项目，并集成我们的自定义驱动文件，实现四路 DRV8870 电机控制和电流采样。

## 1. 环境准备

- Keil MDK 5.x（推荐 5.36 或更高版本）
- STM32CubeMX（2.9.0 或更高版本）
- STM32CubeF3 HAL 库（1.11.x 或更高版本）
- STM32F334C8T6 开发板

## 2. 使用 CubeMX 创建项目

### 2.1 基本配置
1. 打开 STM32CubeMX，新建项目
2. 选择芯片型号 STM32F334C8Tx
3. 系统配置
   - RCC: HSE = Crystal/Ceramic Resonator (8 MHz)
   - SYS: Debug = Serial Wire

### 2.2 时钟配置
1. 切换到"Clock Configuration"选项卡
2. 配置系统时钟：
   - HSE = 8 MHz
   - PLLCLK = HSE × 9 = 72 MHz
   - SYSCLK = PLLCLK
   - HCLK = 72 MHz
   - APB1 = 36 MHz
   - APB2 = 72 MHz
   - Flash Latency = 2 WS

### 2.3 外设配置

#### GPIO
- PC13: 配置为"GPIO_Output" (LED)
- PB0/PB1: 配置为"GPIO_Input" (按键SW1/SW2)，上拉

#### USART1（日志）
- Mode: Asynchronous
- 引脚: PB6 (TX)、PB7 (RX)
- Baud Rate: 115200
- Word Length: 8 Bits
- Parity: None
- Stop Bits: 1

#### I2C1（备用）
- 引脚: PB8 (SCL)、PB9 (SDA)
- I2C Speed Mode: Fast Mode
- 时钟速率: 400 kHz

#### TIM1（PWM + TRGO）
- Clock Source: Internal Clock
- Channel 1/2/3/4: PWM Generation CH1/2/3/4
- Prescaler: 71
- Counter Period: 999
- 引脚: PA8/PA9/PA10/PA11
- TRGO: Update Event

#### TIM2（PWM）
- Clock Source: Internal Clock
- Channel 1/2/3/4: PWM Generation CH1/2/3/4
- Prescaler: 71
- Counter Period: 999
- 引脚: PA0/PA1/PA2/PA3

#### ADC2（电流采样）
- 通道: IN1/IN2/IN3/IN13（默认引脚 PA4/PA5/PA6 和 PB12）
- Resolution: 12 bits
- Data Alignment: Right alignment
- Scan Conversion Mode: Enabled
- Continuous Conversion Mode: Disabled
- External Trigger Conversion Source: Timer 1 Trigger
- External Trigger Conversion Edge: Rising Edge
- DMA Continuous Requests: Enabled
- Regular Channels
  - Rank 1: Channel 1, Sampling Time 61.5 Cycles
  - Rank 2: Channel 2, Sampling Time 61.5 Cycles
  - Rank 3: Channel 3, Sampling Time 61.5 Cycles
  - Rank 4: Channel 13, Sampling Time 61.5 Cycles
- 注：请确保 PB12 设为 Analog 模式

#### DMA
- ADC2: DMA1 Channel2 (或根据实际芯片映射选择)
  - Mode: Circular
  - Peripheral to Memory
  - Peripheral/Memory Data Width: Half Word
  - Memory Increment Mode: Enable
  - Priority: High

#### NVIC
- 启用 DMA1_Channel2_IRQn (或你使用的通道中断)

### 2.4 项目设置与代码生成
1. Project Manager 选项卡
   - Project Name: 设置项目名称
   - Project Location: 选择保存位置
   - Toolchain / IDE: MDK-ARM V5
2. Advanced Settings
   - 设置堆栈大小: Stack Size ≥ 0x400, Heap Size ≥ 0x400
3. 点击 "GENERATE CODE"，生成 Keil 项目

## 3. 集成自定义驱动文件

### 3.1 创建文件夹结构
1. 打开 Keil MDK，加载生成的项目
2. 在项目中创建新的文件夹结构：
   - 在项目树中右击 "Target"
   - 添加新组 (Add Group)："User"
   - 在 "User" 下添加子组：
     - "Drivers"（在其下再添加"drv8870"和"current_sense"子组）
     - "Utils"

### 3.2 添加源文件
1. 将提供的源文件复制到相应目录：
   ```
   User/Drivers/drv8870/drv8870.h
   User/Drivers/drv8870/drv8870.c
   User/Drivers/current_sense/currentsense.h
   User/Drivers/current_sense/currentsense.c
   User/Utils/log.h
   User/Utils/log.c
   ```
2. 将文件添加到项目中（右击相应组 → Add Existing Files...）

### 3.3 配置包含路径
1. 打开项目选项（右击 "Target" → Options）
2. 切换到 "C/C++" 选项卡
3. 在 "Include Paths" 中添加：
   ```
   ./User
   ./User/Drivers
   ./User/Utils
   ```
4. 如使用数学库函数（如 math.h），确保在 "Target" 选项卡的 "Linker" 设置中包含了数学库（勾选 "Use Math Libraries"）

### 3.4 修改生成的中断文件
1. 打开 `Core/Src/stm32f3xx_it.c`
2. 在文件顶部用户代码区域添加：
   ```c
   /* USER CODE BEGIN 1 */
   #include "User/Drivers/current_sense/currentsense.h"
   /* USER CODE END 1 */
   ```
3. 在 DMA 中断处理函数中调用我们的处理程序：
   ```c
   void DMA1_Channel2_IRQHandler(void)
   {
     /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

     /* USER CODE END DMA1_Channel2_IRQn 0 */
     CUR_ADC_DMA_IRQHandler();
     /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

     /* USER CODE END DMA1_Channel2_IRQn 1 */
   }
   ```
   注：如使用不同通道，请修改相应的中断函数名

### 3.5 修改主函数
1. 打开 `Core/Inc/main.h`，在用户代码区添加：
   ```c
   /* USER CODE BEGIN Includes */
   #include "User/Drivers/drv8870/drv8870.h"  
   #include "User/Drivers/current_sense/currentsense.h"
   #include "User/Utils/log.h"
   /* USER CODE END Includes */
   ```

2. 修改 `Core/Src/main.c`，在 main 函数合适位置添加初始化代码：
   ```c
   /* USER CODE BEGIN 2 */
   // 使用日志功能初始化（基于 UART1）
   Log_Init(&huart1);
   LOG_INFO("\r\nF334 + DRV8870, PWM=1kHz, ADC2 DMA (TIM1 TRGO), IN13=PB12\r\n");

   // 初始化四路电机控制
   MTD1_Init(&htim2);
   MTD2_Init(&htim2);
   MTD3_Init(&htim1);
   MTD4_Init(&htim1);

   // 设置初始速度
   MTD1_SetSpeedPercent(+20);
   MTD2_SetSpeedPercent(-30);
   MTD3_SetSpeedPercent(+40);
   MTD4_SetSpeedPercent(-50);

   // 启动电流采样（ADC2+DMA，TIM1 触发）
   CurrentSense_SetCalibration((CurrCalib_t){3.3f, 0.01f, 50.0f});
   CurrentSense_InitDMA(&hadc2);

   // 启用一阶低通滤波，截止频率 20Hz，采样 1kHz
   CurrentSense_SetLPF_ByCutoff(20.0f, 1000.0f);
   CurrentSense_EnableLPF(1);
   /* USER CODE END 2 */
   ```

3. 在主循环中添加：
   ```c
   /* USER CODE BEGIN 3 */
   // 每 500ms 打印一次电流值
   if (HAL_GetTick() - t_last_print >= 500) {
     t_last_print = HAL_GetTick();
     LOG_INFO("I[A]: M1=%.3f M2=%.3f M3=%.3f M4=%.3f | RAW: %u %u %u %u\r\n",
             g_curr_amp_filt[0], g_curr_amp_filt[1], g_curr_amp_filt[2], g_curr_amp_filt[3],
             g_curr_raw[0], g_curr_raw[1], g_curr_raw[2], g_curr_raw[3]);
   }
    
   // 闪烁 LED 表示系统运行
   LED_Blink();
   /* USER CODE END 3 */
   ```

4. 添加 LED 闪烁函数：
   ```c
   /* USER CODE BEGIN 4 */
   static void LED_Blink(void)
   {
     static uint32_t last_toggle = 0;
     if (HAL_GetTick() - last_toggle >= 200) {
       last_toggle = HAL_GetTick();
       HAL_GPIO_TogglePin(LED_PC13_GPIO_Port, LED_PC13_Pin);
     }
   }
   /* USER CODE END 4 */
   ```

## 4. 编译与下载

1. 编译项目（F7 或点击 "Build"）
   - 确保无编译错误或警告
2. 通过 ST-Link 连接开发板
3. 下载程序（F8 或点击 "Load"）
4. 运行程序（点击 "Run"）

## 5. 故障排除

### 5.1 编译错误
- **DMA 中断函数重复定义**：检查 stm32f3xx_it.c 中是否正确转调 CUR_ADC_DMA_IRQHandler()
- **找不到头文件**：检查包含路径是否正确
- **浮点运算错误**：确认链接了数学库 (-lm)
- **ADC 通道错误**：确保 currentsense.h 中的通道号与引脚映射与你的板子一致

### 5.2 运行错误
- **电机不转**：检查 GPIO 复用配置、TIM PWM 通道输出
- **ADC 采样值不变**：检查 TIM1 TRGO 配置、ADC 触发源、DMA 配置
- **串口无输出**：检查波特率、连接、USART 配置

## 6. 注意事项

1. ADC 通道与引脚映射可能因芯片/封装不同而有差异，请根据实际情况调整 currentsense.h 中的定义
2. 确保 DMA 中断处理函数名与你所用的 DMA 通道匹配
3. 如果将 PWM 频率从 1kHz 改为其他值，记得同时更新 LPF 采样频率设置

## 7. 参考资料

- STM32F334 参考手册 (RM0364)
- STM32F334 数据手册
- STM32CubeF3 HAL 文档
- DRV8870 数据手册
- INA180 数据手册

---

本指南最后更新于 2025 年 8 月 13 日
作者：GAARAHK