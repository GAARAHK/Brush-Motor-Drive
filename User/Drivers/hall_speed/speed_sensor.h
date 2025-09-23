#ifndef SPEED_SENSOR_H
#define SPEED_SENSOR_H

#include "stm32f3xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// 速度传感器配置
typedef struct {
    uint8_t magnet_poles;    // 磁环上的磁极对数
    uint32_t timeout_ms;     // 超时时间(毫秒)，用于检测电机停止
    float filter_alpha;      // 速度滤波系数(0-1),推荐使用小值如0.1
    uint32_t min_period;     // 最小有效周期(防止噪声)
	uint32_t max_period;     // 最大有效周期(低速限制)
    float max_rpm;           // 最大合理转速
    uint8_t direction_pin;   // 可选：用于确定方向的引脚
	
} SpeedSensorConfig_t;

// 速度传感器状态
typedef struct {
    TIM_HandleTypeDef *htim; // 定时器句柄
    uint32_t channel;        // 输入捕获通道
    uint32_t last_capture;   // 上次捕获值
    uint32_t period;         // 捕获周期
    uint32_t last_update;    // 上次更新时间
    float rpm;               // 计算的RPM值
    float filtered_rpm;      // 滤波后的RPM值
    bool valid;              // 速度值是否有效
    SpeedSensorConfig_t config; // 配置参数
	
	float raw_rpm;           // 原始RPM，无方向修正
    uint32_t pulse_count;    // 脉冲计数，用于诊断
    uint32_t last_pulse_time; // 上次脉冲时间，用于超时检测
	
} SpeedSensor_t;

// 初始化速度传感器
void SpeedSensor_Init(SpeedSensor_t *sensor, TIM_HandleTypeDef *htim, 
                     uint32_t channel, SpeedSensorConfig_t config);

// 处理输入捕获事件
void SpeedSensor_CaptureCallback(SpeedSensor_t *sensor, uint32_t capture);

// 更新速度传感器状态
void SpeedSensor_Update(SpeedSensor_t *sensor);

// 获取当前RPM
float SpeedSensor_GetRPM(SpeedSensor_t *sensor);

// 检查速度值是否有效
bool SpeedSensor_IsValid(SpeedSensor_t *sensor);

#endif // SPEED_SENSOR_H