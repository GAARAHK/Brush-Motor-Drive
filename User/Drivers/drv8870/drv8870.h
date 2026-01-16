#ifndef DRV8870_H
#define DRV8870_H

#include "stm32f3xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 软启动配置结构体
typedef struct {
    uint16_t ramp_time_ms;      // 软启动时间(毫秒)
    uint8_t step_count;         // 启动步数
    uint8_t enabled;            // 是否启用软启动
} SoftStart_Config_t;

// 电机软启动状态
typedef struct {
    int8_t  target_speed;        // 目标速度(-100 to +100)
    int8_t  current_speed;       // 当前实际输出速度
    uint32_t last_update_time;   // 上次更新时间
    uint32_t step_interval_ms;   // 每步时间间隔
    uint8_t  is_ramping;         // 是否正在软启动
    SoftStart_Config_t config;   // 软启动配置

    // 直换相保护新增
    int8_t   last_direction;       // 上一次方向(-1/0/+1)
    uint8_t  coasting;             // 是否处于滑行阶段
    uint32_t coast_start_time;     // 滑行开始时间
    uint32_t coast_duration_ms;    // 滑行时长(可调, ms)
} SoftStart_State_t;

// 默认软启动配置
#define DEFAULT_SOFT_START_CONFIG { \
    .ramp_time_ms = 500,  \
    .step_count    = 20,  \
    .enabled       = 1    \
}

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

// 软启动配置函数
void MTD_SetSoftStartConfig(uint8_t motor_idx, SoftStart_Config_t config);
void MTD_EnableSoftStart(uint8_t motor_idx, uint8_t enable);
void MTD_SetSoftStartTime(uint8_t motor_idx, uint16_t ramp_time_ms);

// 软启动处理函数（需要在主循环中定期调用）
void MTD_ProcessSoftStart(void);

// 获取电机实际输出速度
int8_t MTD_GetActualSpeed(uint8_t motor_idx);

// 新增：设置直换相滑行时间（ms，0=关闭保护）
void MTD_SetDirectionChangeDelay(uint8_t motor_idx, uint32_t delay_ms);

#ifdef __cplusplus
}
#endif
#endif // DRV8870_H