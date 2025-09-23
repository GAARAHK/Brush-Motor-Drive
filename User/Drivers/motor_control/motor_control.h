#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "stm32f3xx_hal.h"
#include <stdint.h>
#include "drv8870.h"
#include "currentsense.h"
#include "speed_sensor.h"


// 控制模式定义
typedef enum {
    CONTROL_MODE_DUTY = 0,    // 普通PWM占空比控制模式
    CONTROL_MODE_TORQUE = 1,  // 扭矩(电流)控制模式
    CONTROL_MODE_SPEED = 2    // 速度控制模式
} MotorControlMode_t;


// PID控制器结构体
typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    float setpoint;     // 设定值
    float integral;     // 积分项
    float prev_error;   // 上一次误差
    float output_min;   // 输出最小值
    float output_max;   // 输出最大值
    float sample_time;  // 采样时间(秒)
} PID_Controller_t;

// 电机控制状态结构体
typedef struct {
    uint8_t motor_idx;              // 电机索引(0-3)
    MotorControlMode_t mode;        // 控制模式
    float duty_setpoint;            // 普通模式PWM设定值(-100 ~ +100)
    float current_setpoint;         // 扭矩模式电流设定值(A)
    float speed_setpoint;           // 速度模式速度设定值(rpm)
    float actual_current;           // 实际电流值(A)
    float actual_speed;             // 实际速度值(rpm)
    PID_Controller_t current_pid;   // 电流环PID控制器
    PID_Controller_t speed_pid;     // 速度环PID控制器
    uint32_t last_update_time;      // 上次更新时间
    int8_t direction;               // 方向：1=正向，-1=反向，0=停止
    uint8_t enabled;                // 使能标志
	
	SpeedSensor_t *speed_sensor;    // 速度传感器
} MotorControl_t;

// 初始化电机控制系统
void MotorControl_Init(void);


// 关联电机与霍尔传感器
void MotorControl_AssignSpeedSensor(uint8_t motor_idx, SpeedSensor_t *sensor);



// 设置电机控制模式
void MotorControl_SetMode(uint8_t motor_idx, MotorControlMode_t mode);

// 设置占空比(普通模式)
void MotorControl_SetDuty(uint8_t motor_idx, float duty_percent);

// 设置电流(扭矩模式)
void MotorControl_SetCurrent(uint8_t motor_idx, float current_amps);

// 设置速度(速度模式)
void MotorControl_SetSpeed(uint8_t motor_idx, float speed_rpm);

// 启用/禁用电机
void MotorControl_Enable(uint8_t motor_idx, uint8_t enable);

// 电流环控制执行函数(在定时器中断中调用，5kHz频率)
void MotorControl_CurrentLoopUpdate(void);

// 速度环控制执行函数(在中断或主循环中调用，100-500Hz频率)
void MotorControl_SpeedLoopUpdate(void);

// 配置电流环PID参数
void MotorControl_SetCurrentPID(uint8_t motor_idx, float kp, float ki, float kd);

// 配置速度环PID参数
void MotorControl_SetSpeedPID(uint8_t motor_idx, float kp, float ki, float kd);

// 获取电机控制状态
MotorControl_t* MotorControl_GetStatus(uint8_t motor_idx);

#endif // MOTOR_CONTROL_H