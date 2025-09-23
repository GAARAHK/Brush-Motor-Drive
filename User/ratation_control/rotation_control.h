#ifndef ROTATION_CONTROL_H
#define ROTATION_CONTROL_H

#include "main.h"
#include <stdint.h>

// 系统位置定义
typedef enum {
    POSITION_ZERO = 0,            // 0位 (光1=1, 光2=1)
    POSITION_PUMP_PREP = 1,       // 基站抽水预备位 -50.5° (光1:101, 光2:0)
    POSITION_PUMP_FORMAL = 2,     // 基站抽水正式位 -101° (光1:101, 光2:0)
    POSITION_DRAIN_PREP = 3,      // 基站排水预备位 125.5° (光1:0, 光2:10101)
    POSITION_DRAIN_FORMAL = 4,    // 基站排水正式位 79° (光1:0, 光2:1010)
    POSITION_UNKNOWN = 255        // 未知位置
} SystemPosition_t;

// 流程步骤定义
typedef enum {
    STEP_INIT = 0,                // 初始位置(0位)
    STEP_PUMP_PREP = 1,           // 抽水预备位(-50.5°)
    STEP_PUMP_FORMAL = 2,         // 抽水正式位(-101°)
    STEP_RETURN_ZERO_1 = 3,       // 第一次回零
    STEP_DRAIN_PREP = 4,          // 排水预备位(125.5°)
    STEP_DRAIN_FORMAL = 5,        // 排水正式位(79°)
    STEP_RETURN_ZERO_2 = 6,       // 第二次回零
    STEP_COMPLETE = 7,            // 流程完成
    STEP_MAX_COUNT = 8            // 步骤总数
} FlowStep_t;

// 系统运行状态
typedef enum {
    STATE_IDLE = 0,               // 空闲状态
    STATE_ROTATING = 1,           // 正在旋转
    STATE_HOLDING = 2,            // 位置保持(停留)
    STATE_ERROR = 3               // 错误状态
} SystemState_t;

// 函数声明
void RotationControl_Init(void);
void RotationControl_Process(void);
void RotationControl_Start(uint16_t cycles);
void RotationControl_Stop(void);
void RotationControl_SetStepHoldTime(FlowStep_t step, uint32_t hold_time_ms);
void RotationControl_SetMotorSpeed(uint8_t speed_percent);
uint8_t RotationControl_IsComplete(void);
void RotationControl_ReportPosition(SystemPosition_t position);
void RotationControl_SetMotor2Speed(uint8_t speed_percent); // 新增：设置MTD2速度

#endif /* ROTATION_CONTROL_H */


