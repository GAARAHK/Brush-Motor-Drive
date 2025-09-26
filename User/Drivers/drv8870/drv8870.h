#ifndef DRV8870_H
#define DRV8870_H

#include "stm32f3xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 软启动配置参数
#define SOFT_START_ENABLE           1       // 启用软启动
#define SOFT_START_RAMP_TIME_MS     1000    // 软启动时间
#define LOAD_DELAY_TIME_MS          1000    // 负载延迟时间  
#define MAX_MOTOR_PAIR_PHASES       8       // 最大阶段数

// DRV8870 双输入控制：-100..100 速度百分比
// >0: IN1 = PWM, IN2 = 0；<0: IN1 = 0, IN2 = PWM；0: 两路 0（COAST）

// MTD1: TIM2_CH1/CH2
void MTD1_Init(TIM_HandleTypeDef* htim);
void MTD1_SetSpeedPercent(int8_t percent);
void MTD1_Coast(void);

// MTD2: TIM2_CH3/CH4
void MTD2_Init(TIM_HandleTypeDef* htim);
void MTD2_SetSpeedPercent(int8_t percent);
void MTD2_Coast(void);

// MTD3: TIM1_CH1/CH2
void MTD3_Init(TIM_HandleTypeDef* htim);
void MTD3_SetSpeedPercent(int8_t percent);
void MTD3_Coast(void);

// MTD4: TIM1_CH3/CH4
void MTD4_Init(TIM_HandleTypeDef* htim);
void MTD4_SetSpeedPercent(int8_t percent);
void MTD4_Coast(void);

// 软启动功能
void DRV8870_SoftStartUpdate(void);

#ifdef __cplusplus
}
#endif
#endif // DRV8870_H