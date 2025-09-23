#ifndef CMD_MOTOR_H
#define CMD_MOTOR_H

#include "at_command.h"
#include <stdint.h>




// 最大支持的自定义阶段数
#define MAX_CUSTOM_PHASES 20



// 电机方向定义
typedef enum {
    MOTOR_DIR_STOP = 0,  // 停止
    MOTOR_DIR_CW = 1,    // 正转
    MOTOR_DIR_CCW = 2    // 反转
} MotorDirection_t;

// 电机错误代码
typedef enum {
    MOTOR_ERR_NONE = 0,     // 无错误
    MOTOR_ERR_OVERVOLTAGE,  // 过压
    MOTOR_ERR_UNDERVOLTAGE, // 欠压
    MOTOR_ERR_OVERCURRENT,  // 过流（堵转）
    MOTOR_ERR_OTHER         // 其他错误
} MotorErrorCode_t;

// 电机类型
typedef enum {
    MOTOR_TYPE_BRUSHED = 1,   // 有刷直流电机
    MOTOR_TYPE_BLDC = 2,      // 无刷无感电机
    MOTOR_TYPE_FOC = 3,       // 无刷有感电机
    MOTOR_TYPE_STEPPER = 4    // 步进电机
} MotorType_t;

// 电机状态结构体
typedef struct {
    uint16_t id;               // 设备ID
    uint8_t motor_num;         // 电机编号 (1-4)
    uint8_t enabled;           // 是否启用
    uint16_t voltage_mv;       // 电压 (mV)
    uint16_t current_ma;       // 电流 (mA)
    MotorDirection_t direction;// 方向状态
    uint16_t temperature;      // 温度
    uint16_t speed;            // 速度
    MotorErrorCode_t error;    // 错误状态
    char hw_version[16];       // 硬件版本
} MotorStatus_t;

// 电机报警阈值
typedef struct {
    uint16_t overvoltage_mv;   // 过压阈值 (mV)
    uint16_t undervoltage_mv;  // 欠压阈值 (mV)
    uint16_t overcurrent_ma;   // 过流阈值 (mA)
} MotorAlarm_t;


// 单个阶段定义
typedef struct {
    uint32_t start_time_ms;     // 阶段开始时间(相对于周期开始)
    MotorDirection_t direction; // 运行方向
    uint16_t pwm;               // PWM值
} MotorPhase_t;


// 添加初始化函数声明
void MotorCmd_Init(void);


// AT+MotorRun 命令处理 (设备ID,电机号,方向,PWM,运行时间)
AtCmdStatus_t MotorCmd_Run(const char *params);

// AT+MotorStop 命令处理 (设备ID,电机号)
AtCmdStatus_t MotorCmd_Stop(const char *params);

// AT+MotorStatusQuery 命令处理 (设备ID,电机号)
AtCmdStatus_t MotorCmd_StatusQuery(const char *params);

// AT+MotorStatusUpLoad 命令处理 (设备ID,电机号,上报周期)
AtCmdStatus_t MotorCmd_StatusUpLoad(const char *params);

// AT+MotorStatusUpLoadStop 命令处理 (设备ID,电机号)
AtCmdStatus_t MotorCmd_StatusUpLoadStop(const char *params);

// AT+MotorSetDevID 命令处理 (设备ID)
AtCmdStatus_t MotorCmd_SetDevID(const char *params);

// AT+MotorSetType 命令处理 (设备ID,电机号,类型)
AtCmdStatus_t MotorCmd_SetType(const char *params);

// AT+MotorAlarmValue 命令处理 (设备ID,电机号,过压,欠压,过流)
AtCmdStatus_t MotorCmd_AlarmValue(const char *params);

// AT+MotorRunRepeat 命令处理 (设备ID,电机号,PWM,正转时间,正转停止时间,反转时间,反转停止时间,循环总次数)
AtCmdStatus_t MotorCmd_RunRepeat(const char *params);

// AT+MotorStopRepeat 命令处理 (设备ID,电机号)
AtCmdStatus_t MotorCmd_StopRepeat(const char *params);

// 查询设备ID命令
AtCmdStatus_t MotorCmd_QueryDevID(void);

// 查询版本命令
AtCmdStatus_t MotorCmd_QueryVersion(void);

// 自定义运行状态
AtCmdStatus_t MotorCmd_RunCustom(const char *params);

// 停止自定义运行
AtCmdStatus_t MotorCmd_StopCustom(const char *params);

// 在cmd_motor.h中声明
void MotorCmd_DisableAllStatusUploads(void);

//换气阀启动循环命令
AtCmdStatus_t HuanQi_Start(const char *params);
//换气阀停止循环命令
AtCmdStatus_t HuanQi_Stop(const char *params);

//************************************
// 控制模式切换命令
AtCmdStatus_t MotorCmd_SetControlMode(const char *params);

// 扭矩控制命令
AtCmdStatus_t MotorCmd_SetTorque(const char *params);

// 速度控制命令
AtCmdStatus_t MotorCmd_SetTargetSpeed(const char *params);



//配置传感器设置
AtCmdStatus_t MotorCmd_SpeedConfig(const char *params);

// PID参数设置命令
AtCmdStatus_t MotorCmd_SetCurrentPID(const char *params);
AtCmdStatus_t MotorCmd_SetSpeedPID(const char *params);

// 电机控制状态查询命令
AtCmdStatus_t MotorCmd_QueryControlStatus(const char *params);
// 电机使能/禁用命令: AT+MotorEnable=设备ID,电机号,使能状态
AtCmdStatus_t MotorCmd_SetEnable(const char *params);


// 周期性处理函数 (在主循环中调用)
void MotorCmd_PeriodicHandler(void);

#endif // CMD_MOTOR_H