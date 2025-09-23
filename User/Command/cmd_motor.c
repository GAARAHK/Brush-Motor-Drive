#include "cmd_motor.h"
#include "cmd_parser.h"
#include "log.h"
#include "drv8870.h"
#include "currentsense.h"
#include <string.h>
#include <stdlib.h>
#include "at_command.h"
#include "rotation_control.h"
#include "motor_control.h"
#include "tim.h"
#include "flash_storage.h"

// 添加在文件开头的全局定义部分
#define FIRMWARE_VERSION "V1.1.0"  // 固件版本号

// 在全局变量区


// 全局速度传感器实例
SpeedSensor_t g_speed_sensors[4];

// 设备ID变量定义 (现在是从Flash读取)
 uint16_t g_device_id;


static uint8_t g_motor_enabled[4] = {1, 1, 1, 1};
static MotorType_t g_motor_type = MOTOR_TYPE_BRUSHED;
static char g_hw_version[16] = FIRMWARE_VERSION;

// 报警阈值
static MotorAlarm_t g_motor_alarm = {
    .overvoltage_mv = 16000,
    .undervoltage_mv = 9000,
    .overcurrent_ma = 800
};


// 自定义运行状态数据结构
static struct {
    uint16_t device_id;
    uint8_t motor_num;
    uint32_t total_period_ms;    // 总周期时间
    uint32_t max_cycles;         // 最大循环次数
    uint32_t current_cycle;      // 当前已完成循环数
    uint8_t phase_count;         // 阶段数量
    MotorPhase_t phases[MAX_CUSTOM_PHASES]; // 阶段数组
    uint8_t current_phase;       // 当前阶段索引
    uint32_t cycle_start_time;   // 当前周期开始时间
    uint8_t is_running;          // 是否运行中
} g_motor_custom[4] = {0};



// 电机运行状态
static struct {
    MotorDirection_t direction;
    uint16_t pwm;
    uint32_t runtime_ms;
    uint32_t start_time;
    uint8_t is_running;
} g_motor_run[4] = {0};

// 电机循环运行状态
static struct {
    uint16_t pwm;                   // PWM值
    uint32_t cw_time_ms;            // 正转持续时间
    uint32_t cw_stop_time_ms;       // 正转后停止时间
    uint32_t ccw_time_ms;           // 反转持续时间
    uint32_t ccw_stop_time_ms;      // 反转后停止时间
    uint32_t max_cycles;            // 最大循环次数(0=无限循环)
    uint32_t current_cycle;         // 当前已完成循环数
    uint32_t last_switch_time;      // 最后一次状态切换时间
    MotorDirection_t current_dir;   // 当前方向
    uint8_t current_state;          // 当前状态(0=正转, 1=正转停止, 2=反转, 3=反转停止)
    uint8_t is_running;             // 是否正在循环运行
} g_motor_repeat[4] = {0};

// 状态上报配置
typedef struct {
    uint16_t device_id;
    uint8_t motor_num;
    uint32_t period_ms;
    uint32_t last_time;
    uint8_t enabled;
} StatusUpload_t;

static StatusUpload_t g_status_upload[4] = {0};

// 将毫秒转换为微秒
static inline uint32_t ms_to_us(uint32_t ms) {
    return ms * 1000;
}

// 将微秒转换为毫秒
static inline uint32_t us_to_ms(uint32_t us) {
    return us / 1000;
}

// 初始化电机命令模块
void MotorCmd_Init(void)
{
    // 初始化Flash存储
    Flash_Init();
    
    // 从Flash读取设备ID
    g_device_id = Flash_ReadDeviceID();
    
    LOG_INFO("Motor command module initialized with Device ID: %d\r\n", g_device_id);
}

// 检查设备ID和电机号是否匹配
static bool CheckDeviceAndMotor(uint16_t dev_id, uint8_t motor_num) {
    // 验证设备ID
    if (dev_id != g_device_id) {
        LOG_WARN("Device ID mismatch: %d != %d\r\n", dev_id, g_device_id);
        return false;
    }
    
    // 验证电机号 (1-4)
    if (motor_num < 1 || motor_num > 4) {
        LOG_WARN("Invalid motor number: %d\r\n", motor_num);
        return false;
    }
    
    // 检查电机是否启用
    if (!g_motor_enabled[motor_num-1]) {
        LOG_WARN("Motor %d is disabled\r\n", motor_num);
        return false;
    }
    
    return true;
}

// 电机控制函数
// 增强Motor_SetState函数的电机换向保护
static void Motor_SetState(uint8_t motor_idx, MotorDirection_t dir, uint16_t pwm) {
    int8_t speed = 0;
    
    // 将 PWM 值 (0-10000) 转换为百分比 (-100 到 +100)
    if (dir == MOTOR_DIR_CW) {
        speed = (pwm * 100) / 10000;
    } else if (dir == MOTOR_DIR_CCW) {
        speed = -((pwm * 100) / 10000);
    }
    
    // 调用对应电机控制函数
    switch (motor_idx) {
        case 0: // 电机 1
            MTD1_SetSpeedPercent(speed);
            break;
        case 1: // 电机 2
            MTD2_SetSpeedPercent(speed);
            break;
        case 2: // 电机 3
            MTD3_SetSpeedPercent(speed);
            break;
        case 3: // 电机 4
            MTD4_SetSpeedPercent(speed);
            break;
    }
    
    // 更新电机状态
    g_motor_run[motor_idx].direction = dir;
    g_motor_run[motor_idx].pwm = pwm;
}

// 获取电机状态
static void Motor_GetStatus(uint8_t motor_idx, MotorStatus_t *status) {
    status->id = g_device_id;
    status->motor_num = motor_idx + 1;
    status->enabled = g_motor_enabled[motor_idx];
    status->direction = g_motor_run[motor_idx].direction;
    
    // 获取电流值 (mA)
    status->current_ma = (uint16_t)(g_curr_amp_filt[motor_idx] * 1000.0f);
    
    // 模拟电压值 (实际项目中应从硬件读取)
    status->voltage_mv = 12000;
    
    // 模拟温度和速度 (实际项目中应从硬件读取)
    status->temperature = 25;
    status->speed = g_motor_run[motor_idx].pwm;
    
    // 错误检测
    if (status->voltage_mv > g_motor_alarm.overvoltage_mv) {
        status->error = MOTOR_ERR_OVERVOLTAGE;
    } else if (status->voltage_mv < g_motor_alarm.undervoltage_mv) {
        status->error = MOTOR_ERR_UNDERVOLTAGE;
    } else if (status->current_ma > g_motor_alarm.overcurrent_ma) {
        status->error = MOTOR_ERR_OVERCURRENT;
    } else {
        status->error = MOTOR_ERR_NONE;
    }
    
    // 硬件版本
    strcpy(status->hw_version, g_hw_version);
}

// 发送电机状态
static void Motor_SendStatus(uint8_t motor_idx) {
    MotorStatus_t status;
    Motor_GetStatus(motor_idx, &status);
    
    AT_SendResponse("AT+MotorStatusQuery=%d,%d,%d,%d,%d,%d,%d,%d,%d,%s",
                   status.id,
                   status.motor_num,
                   status.enabled,
                   status.voltage_mv,
                   status.current_ma,
                   status.direction,
                   status.temperature,
                   status.speed,
                   status.error,
                   status.hw_version);
}

// AT+MotorRun 命令处理
AtCmdStatus_t MotorCmd_Run(const char *params) {
    if (params == NULL) {
        return AT_PARAM_ERROR;
    }
    
    // 解析参数: 设备ID,电机号,方向,PWM,运行时间
    int dev_id, motor_num, direction, pwm, runtime_us;
    if (sscanf(params, "%d,%d,%d,%d,%d", &dev_id, &motor_num, &direction, &pwm, &runtime_us) != 5) {
        return AT_PARAM_ERROR;
    }
    
    // 验证设备ID和电机号
    if (!CheckDeviceAndMotor(dev_id, motor_num)) {
        return AT_PARAM_ERROR;
    }
    
    // 验证方向参数
    if (direction != MOTOR_DIR_CW && direction != MOTOR_DIR_CCW) {
        return AT_PARAM_ERROR;
    }
    
    // 限制PWM范围
    if (pwm < 0 || pwm > 10000) {
        pwm = pwm < 0 ? 0 : 10000;  // 限制在有效范围内
    }
    
    // 获取电机索引 (0-3)
    uint8_t motor_idx = motor_num - 1;
    
    // 停止之前的循环运行
    g_motor_repeat[motor_idx].is_running = 0;
    
    // 设置电机运行状态
    g_motor_run[motor_idx].direction = direction;
    g_motor_run[motor_idx].pwm = pwm;
    g_motor_run[motor_idx].runtime_ms = us_to_ms(runtime_us);
    g_motor_run[motor_idx].start_time = HAL_GetTick();
    g_motor_run[motor_idx].is_running = 1;
    
    // 设置电机状态
    Motor_SetState(motor_idx, direction, pwm);
    
    // 返回确认信息
    AT_SendResponse("AT+MotorRun=%d,%d,%d,%d,%d,0",
                   dev_id,
                   motor_num,
                   g_motor_enabled[motor_idx],
                   pwm,
                   runtime_us);
    
    return AT_OK;
}

// AT+MotorStop 命令处理
AtCmdStatus_t MotorCmd_Stop(const char *params) {
    if (params == NULL) {
        return AT_PARAM_ERROR;
    }
    
    // 解析参数: 设备ID,电机号
    int dev_id, motor_num;
    if (sscanf(params, "%d,%d", &dev_id, &motor_num) != 2) {
        return AT_PARAM_ERROR;
    }
    
    // 验证设备ID和电机号
    if (!CheckDeviceAndMotor(dev_id, motor_num)) {
        return AT_PARAM_ERROR;
    }
    
    // 获取电机索引 (0-3)
    uint8_t motor_idx = motor_num - 1;
    
    // 停止电机
    Motor_SetState(motor_idx, MOTOR_DIR_STOP, 0);
    
    // 清除运行状态
    g_motor_run[motor_idx].is_running = 0;
    g_motor_repeat[motor_idx].is_running = 0;
    
    // 返回确认信息
    AT_SendResponse("AT+MotorStop=%d,%d,%d",
                   dev_id,
                   motor_num,
                   g_motor_enabled[motor_idx]);
    
    return AT_OK;
}

// AT+MotorStatusQuery 命令处理
AtCmdStatus_t MotorCmd_StatusQuery(const char *params) {
    if (params == NULL) {
        return AT_PARAM_ERROR;
    }
    
    // 解析参数: 设备ID,电机号
    int dev_id, motor_num;
    if (sscanf(params, "%d,%d", &dev_id, &motor_num) != 2) {
        return AT_PARAM_ERROR;
    }
    
    // 验证设备ID和电机号
    if (!CheckDeviceAndMotor(dev_id, motor_num)) {
        return AT_PARAM_ERROR;
    }
    
    // 获取电机索引 (0-3)
    uint8_t motor_idx = motor_num - 1;
    
    // 发送电机状态
    Motor_SendStatus(motor_idx);
    
    return AT_OK;
}

// AT+MotorStatusUpLoad 命令处理
AtCmdStatus_t MotorCmd_StatusUpLoad(const char *params) {
    if (params == NULL) {
        return AT_PARAM_ERROR;
    }
    
	
	
    // 解析参数: 设备ID,电机号,上报周期
    int dev_id, motor_num, period_ms;
    if (sscanf(params, "%d,%d,%d", &dev_id, &motor_num, &period_ms) != 3) {
        return AT_PARAM_ERROR;
    }
    
    // 验证设备ID和电机号
    if (!CheckDeviceAndMotor(dev_id, motor_num)) {
        return AT_PARAM_ERROR;
    }
    
    // 获取电机索引 (0-3)
    uint8_t motor_idx = motor_num - 1;
    
    // 设置状态上报
    g_status_upload[motor_idx].device_id = dev_id;
    g_status_upload[motor_idx].motor_num = motor_num;
	
	// 限制最小上传间隔为10ms，避免系统过载
    if (period_ms < 10) {
        period_ms = 10;
    }
	
    g_status_upload[motor_idx].period_ms = period_ms;
    // 修改初始时间，确保首次检查时就能触发上报
    g_status_upload[motor_idx].last_time = HAL_GetTick() - period_ms;
    g_status_upload[motor_idx].enabled = 1;
    
    return AT_OK;
}

// AT+MotorStatusUpLoadStop 命令处理
AtCmdStatus_t MotorCmd_StatusUpLoadStop(const char *params) {
    if (params == NULL) {
        return AT_PARAM_ERROR;
    }
    
    // 解析参数: 设备ID,电机号
    int dev_id, motor_num;
    if (sscanf(params, "%d,%d", &dev_id, &motor_num) != 2) {
        return AT_PARAM_ERROR;
    }
    
    // 验证设备ID和电机号
    if (!CheckDeviceAndMotor(dev_id, motor_num)) {
        return AT_PARAM_ERROR;
    }
    
    // 获取电机索引 (0-3)
    uint8_t motor_idx = motor_num - 1;
    
    // 禁用状态上报
    g_status_upload[motor_idx].enabled = 0;
    
    // 返回确认信息
    AT_SendResponse("AT+MotorStatusUpLoadStop=%d,%d,%d",
                   dev_id,
                   motor_num,
                   g_motor_enabled[motor_idx]);
    
    return AT_OK;
}

// AT+MotorSetDevID 命令处理
AtCmdStatus_t MotorCmd_SetDevID(const char *params) {
    if (params == NULL) {
        return AT_PARAM_ERROR;
    }
    
    // 解析参数: 设备ID
    int dev_id;
    if (sscanf(params, "%d", &dev_id) != 1) {
        return AT_PARAM_ERROR;
    }
    
    // 设置新的设备ID并写入Flash
    HAL_StatusTypeDef status = Flash_WriteDeviceID((uint16_t)dev_id);
    if (status != HAL_OK) {
        LOG_ERROR("Failed to save Device ID to Flash: %d\r\n", status);
        return AT_EXECUTION_ERROR;
    }
    
    // 更新当前使用的设备ID
    g_device_id = (uint16_t)dev_id;
    
    // 返回确认信息
    AT_SendResponse("AT+MotorSetDevID=%d", g_device_id);
    
    return AT_OK;
}

// AT+MotorSetType 命令处理
AtCmdStatus_t MotorCmd_SetType(const char *params) {
    if (params == NULL) {
        return AT_PARAM_ERROR;
    }
    
    // 解析参数: 设备ID,电机号,类型
    int dev_id, motor_num, type;
    if (sscanf(params, "%d,%d,%d", &dev_id, &motor_num, &type) != 3) {
        return AT_PARAM_ERROR;
    }
    
    // 验证设备ID和电机号
    if (!CheckDeviceAndMotor(dev_id, motor_num)) {
        return AT_PARAM_ERROR;
    }
    
    // 验证参数
    if (type < MOTOR_TYPE_BRUSHED || type > MOTOR_TYPE_STEPPER) {
        return AT_PARAM_ERROR;
    }
    
    // 获取电机索引 (0-3)
    uint8_t motor_idx = motor_num - 1;
    
    // 设置电机类型
    g_motor_type = (MotorType_t)type;
    
    // 返回确认信息
    AT_SendResponse("AT+MotorSetType=%d,%d,%d,%d",
                   dev_id,
                   motor_num,
                   g_motor_enabled[motor_idx],
                   g_motor_type);
    
    return AT_OK;
}

// AT+MotorAlarmValue 命令处理
AtCmdStatus_t MotorCmd_AlarmValue(const char *params) {
    if (params == NULL) {
        return AT_PARAM_ERROR;
    }
    
    // 解析参数: 设备ID,电机号,过压,欠压,过流
    int dev_id, motor_num, overvoltage, undervoltage, overcurrent;
    if (sscanf(params, "%d,%d,%d,%d,%d", &dev_id, &motor_num, &overvoltage, &undervoltage, &overcurrent) != 5) {
        return AT_PARAM_ERROR;
    }
    
    // 验证设备ID和电机号
    if (!CheckDeviceAndMotor(dev_id, motor_num)) {
        return AT_PARAM_ERROR;
    }
    
    // 获取电机索引 (0-3)
    uint8_t motor_idx = motor_num - 1;
    
    // 设置报警阈值
    g_motor_alarm.overvoltage_mv = overvoltage;
    g_motor_alarm.undervoltage_mv = undervoltage;
    g_motor_alarm.overcurrent_ma = overcurrent;
    
    // 返回确认信息
    AT_SendResponse("AT+MotorAlarmValue=%d,%d,%d,%d,%d,%d",
                   dev_id,
                   motor_num,
                   g_motor_enabled[motor_idx],
                   g_motor_alarm.overvoltage_mv,
                   g_motor_alarm.undervoltage_mv,
                   g_motor_alarm.overcurrent_ma);
    
    return AT_OK;
}

// AT+MotorRunRepeat 命令处理
AtCmdStatus_t MotorCmd_RunRepeat(const char *params) {
    if (params == NULL) {
        return AT_PARAM_ERROR;
    }
    
    // 解析参数: 设备ID,电机号,PWM,正转时间,正转停止时间,反转时间,反转停止时间,循环总次数
    int dev_id, motor_num, pwm, cw_time_us, cw_stop_time_us, ccw_time_us, ccw_stop_time_us, total_cycles;
    if (sscanf(params, "%d,%d,%d,%d,%d,%d,%d,%d", &dev_id, &motor_num, &pwm, &cw_time_us, 
               &cw_stop_time_us, &ccw_time_us, &ccw_stop_time_us, &total_cycles) != 8) {
        return AT_PARAM_ERROR;
    }
    
    // 验证设备ID和电机号
    if (!CheckDeviceAndMotor(dev_id, motor_num)) {
        return AT_PARAM_ERROR;
    }
    
    // 验证参数
    if (pwm < 0 || pwm > 10000) {
        pwm = pwm < 0 ? 0 : 10000;  // 限制在有效范围内
    }
    
    // 获取电机索引 (0-3)
    uint8_t motor_idx = motor_num - 1;
    
    // 停止之前的运行
    g_motor_run[motor_idx].is_running = 0;
    
    // 设置循环运行参数
    g_motor_repeat[motor_idx].pwm = pwm;
    g_motor_repeat[motor_idx].cw_time_ms = us_to_ms(cw_time_us);
    g_motor_repeat[motor_idx].cw_stop_time_ms = us_to_ms(cw_stop_time_us);
    g_motor_repeat[motor_idx].ccw_time_ms = us_to_ms(ccw_time_us);
    g_motor_repeat[motor_idx].ccw_stop_time_ms = us_to_ms(ccw_stop_time_us);
    g_motor_repeat[motor_idx].max_cycles = total_cycles;
    g_motor_repeat[motor_idx].current_cycle = 0;
    g_motor_repeat[motor_idx].last_switch_time = HAL_GetTick();
    g_motor_repeat[motor_idx].current_dir = MOTOR_DIR_CW;
    g_motor_repeat[motor_idx].current_state = 0; // 开始于正转状态
    g_motor_repeat[motor_idx].is_running = 1;
    
    // 开始正向运行
    Motor_SetState(motor_idx, MOTOR_DIR_CW, pwm);
    
    // 返回确认信息
    AT_SendResponse("AT+MotorRunRepeat=%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                   dev_id,
                   motor_num,
                   g_motor_enabled[motor_idx],
                   pwm,
                   cw_time_us,
                   cw_stop_time_us,
                   ccw_time_us,
                   ccw_stop_time_us,
                   total_cycles,
                   0); // 当前循环次数为0
    
    return AT_OK;
}

// AT+MotorStopRepeat 命令处理
AtCmdStatus_t MotorCmd_StopRepeat(const char *params) {
    if (params == NULL) {
        return AT_PARAM_ERROR;
    }
    
    // 解析参数: 设备ID,电机号
    int dev_id, motor_num;
    if (sscanf(params, "%d,%d", &dev_id, &motor_num) != 2) {
        return AT_PARAM_ERROR;
    }
    
    // 验证设备ID和电机号
    if (!CheckDeviceAndMotor(dev_id, motor_num)) {
        return AT_PARAM_ERROR;
    }
    
    // 获取电机索引 (0-3)
    uint8_t motor_idx = motor_num - 1;
    
    if (g_motor_repeat[motor_idx].is_running) {
        // 停止电机
        Motor_SetState(motor_idx, MOTOR_DIR_STOP, 0);
        
        // 返回确认信息
        AT_SendResponse("AT+MotorStopRepeat=%d,%d,%d,%d",
                       dev_id,
                       motor_num,
                       g_motor_enabled[motor_idx],
                       g_motor_repeat[motor_idx].current_cycle);
        
        // 清除循环运行状态
        g_motor_repeat[motor_idx].is_running = 0;
    } else {
        // 电机不在循环运行状态
        AT_SendResponse("AT+MotorStopRepeat=%d,%d,%d,0",
                       dev_id,
                       motor_num,
                       g_motor_enabled[motor_idx]);
    }
    
    return AT_OK;
}


// 查询设备ID命令处理
AtCmdStatus_t MotorCmd_QueryDevID(void) {
    // 返回当前设备ID
    AT_SendResponse("AT+MotorSetDevID=%d", g_device_id);
    return AT_OK;
}

// 查询版本命令处理
AtCmdStatus_t MotorCmd_QueryVersion(void) {
    // 返回固件版本号
    AT_SendResponse("AT+QueryVersion=%s", FIRMWARE_VERSION);
    return AT_OK;
}



// 自定义电机运行命令处理
AtCmdStatus_t MotorCmd_RunCustom(const char *params) {
    if (params == NULL) {
		LOG_ERROR("MotorRunCustom: NULL params\r\n");
        return AT_PARAM_ERROR;
    }
    
    // 用于存储解析结果的变量
    int dev_id = 0, motor_num = 0, total_period = 0, repeat_count = 0, phase_count = 0;
    
    // 解析基础参数
    int result = sscanf(params, "%d,%d,%d,%d,%d", 
                        &dev_id, &motor_num, &total_period, &repeat_count, &phase_count);
                        
    LOG_DEBUG("RunCustom parsing: %d params, dev=%d, motor=%d, period=%d, repeat=%d, phases=%d\r\n",
             result, dev_id, motor_num, total_period, repeat_count, phase_count);
    
    if (result != 5 || phase_count <= 0 || phase_count > MAX_CUSTOM_PHASES) {
        LOG_ERROR("MotorRunCustom: Invalid base params\r\n");
        return AT_PARAM_ERROR;
    }
    
    // 验证设备ID和电机号
    if (!CheckDeviceAndMotor(dev_id, motor_num)) {
        LOG_ERROR("MotorRunCustom: Invalid device or motor\r\n");
        return AT_PARAM_ERROR;
    }
    
    // 获取电机索引 (0-3)
    uint8_t motor_idx = motor_num - 1;
    
    // 停止其他运行状态
    g_motor_run[motor_idx].is_running = 0;
    g_motor_repeat[motor_idx].is_running = 0;
    
    // 设置自定义运行参数
    g_motor_custom[motor_idx].device_id = dev_id;
    g_motor_custom[motor_idx].motor_num = motor_num;
    g_motor_custom[motor_idx].total_period_ms = total_period;
    g_motor_custom[motor_idx].max_cycles = repeat_count;
    g_motor_custom[motor_idx].current_cycle = 0;
    g_motor_custom[motor_idx].phase_count = phase_count;
    
    // 查找阶段参数的起始位置
    const char *phase_params = params;
    int comma_count = 0;
    while (*phase_params && comma_count < 5) {
        if (*phase_params == ',') comma_count++;
        phase_params++;
    }
    
    if (comma_count < 5 || !*phase_params) {
        LOG_ERROR("MotorRunCustom: Can't find phase params\r\n");
        return AT_PARAM_ERROR;
    }
    
    // 解析所有阶段参数
    for (int i = 0; i < phase_count; i++) {
        int start_time, direction, pwm;
        
        // 解析一个阶段的三个参数
        result = sscanf(phase_params, "%d,%d,%d", &start_time, &direction, &pwm);
        
        if (result != 3) {
            LOG_ERROR("MotorRunCustom: Failed to parse phase %d\r\n", i);
            return AT_PARAM_ERROR;
        }
        
        // 验证参数
        if (direction < 0 || direction > 2) {
            LOG_ERROR("MotorRunCustom: Invalid direction %d\r\n", direction);
            return AT_PARAM_ERROR;
        }
        
        if (pwm < 0 || pwm > 10000) {
            pwm = pwm < 0 ? 0 : 10000;
        }
        
        // 保存阶段参数
        g_motor_custom[motor_idx].phases[i].start_time_ms = start_time;
        g_motor_custom[motor_idx].phases[i].direction = (MotorDirection_t)direction;
        g_motor_custom[motor_idx].phases[i].pwm = pwm;
        
        //LOG_DEBUG("Phase %d: time=%d, dir=%d, pwm=%d\r\n", 
                 //i, start_time, direction, pwm);
        
        // 移动到下一组参数
        comma_count = 0;
        while (*phase_params && comma_count < 3) {
            if (*phase_params == ',') comma_count++;
            phase_params++;
        }
        
        if (comma_count < 3 && i < phase_count - 1) {
            LOG_ERROR("MotorRunCustom: Missing params for phase %d\r\n", i+1);
            return AT_PARAM_ERROR;
        }
    }
    
    // 验证阶段时间是否按顺序递增
    for (int i = 1; i < phase_count; i++) {
        if (g_motor_custom[motor_idx].phases[i].start_time_ms <= 
            g_motor_custom[motor_idx].phases[i-1].start_time_ms) {
            LOG_ERROR("MotorRunCustom: Phase times not increasing\r\n");
            return AT_PARAM_ERROR;  // 时间必须递增
        }
    }
    
    // 验证最后一个阶段不超过总周期
    if (g_motor_custom[motor_idx].phases[phase_count-1].start_time_ms >= total_period) {
        LOG_ERROR("MotorRunCustom: Last phase exceeds total period\r\n");
        return AT_PARAM_ERROR;
    }
    
    // 启动自定义运行
    g_motor_custom[motor_idx].current_phase = 0;
    g_motor_custom[motor_idx].cycle_start_time = HAL_GetTick();
    g_motor_custom[motor_idx].is_running = 1;
    
    // 设置初始阶段的电机状态
    MotorPhase_t *first_phase = &g_motor_custom[motor_idx].phases[0];
    Motor_SetState(motor_idx, first_phase->direction, first_phase->pwm);
    
    LOG_INFO("MotorRunCustom: Started for motor %d with %d phases\r\n", 
             motor_num, phase_count);
    
    // 构建响应消息
    AT_SendResponse("AT+MotorRunCustom=%d,%d,%d,%d,%d,%d,%d",
                   dev_id, motor_num, g_motor_enabled[motor_idx],
                   total_period, repeat_count, phase_count, 0);
    
    return AT_OK;
}

// 停止自定义运行
AtCmdStatus_t MotorCmd_StopCustom(const char *params) {
    if (params == NULL) {
        return AT_PARAM_ERROR;
    }
    
    // 解析参数: 设备ID,电机号
    int dev_id, motor_num;
    if (sscanf(params, "%d,%d", &dev_id, &motor_num) != 2) {
        return AT_PARAM_ERROR;
    }
    
    // 验证设备ID和电机号
    if (!CheckDeviceAndMotor(dev_id, motor_num)) {
        return AT_PARAM_ERROR;
    }
    
    // 获取电机索引 (0-3)
    uint8_t motor_idx = motor_num - 1;
    
    if (g_motor_custom[motor_idx].is_running) {
        // 停止电机
        Motor_SetState(motor_idx, MOTOR_DIR_STOP, 0);
        
        // 返回确认信息
        AT_SendResponse("AT+MotorStopCustom=%d,%d,%d,%d",
                       dev_id,
                       motor_num,
                       g_motor_enabled[motor_idx],
                       g_motor_custom[motor_idx].current_cycle);
        
        // 清除自定义运行状态
        g_motor_custom[motor_idx].is_running = 0;
    } else {
        // 电机不在自定义运行状态
        AT_SendResponse("AT+MotorStopCustom=%d,%d,%d,0",
                       dev_id,
                       motor_num,
                       g_motor_enabled[motor_idx]);
    }
    
    return AT_OK;
}	


//开启换气阀循环运行
AtCmdStatus_t HuanQi_Start(const char *params) {
    if (params == NULL) {
        return AT_PARAM_ERROR;
    }
    
    // 解析参数: 设备ID,电机号
    int cycles;
    if (sscanf(params, "%d", &cycles) != 1) {
        return AT_PARAM_ERROR;
    }
	if (cycles <= 0) {
            return AT_PARAM_ERROR;
        }
	
	// 启动旋转流程
    RotationControl_Start(cycles);
	
	// 电机不在自定义运行状态
    AT_SendResponse("Starting rotation process with %d cycles\r\n", cycles);

    return AT_OK;
}	



//停止换气阀循环运行
AtCmdStatus_t HuanQi_Stop(const char *params) {
	
	
    // 停止旋转流程
    RotationControl_Stop();
        
    AT_SendResponse("Rotation process stopped\r\n");

    return AT_OK;
}	



// 在cmd_motor.c中添加，停止所有电机
void MotorCmd_DisableAllStatusUploads(void) {
    for (int i = 0; i < 4; i++) {
        g_status_upload[i].enabled = 0;
    }
}



//**************************************以下是速度环控制**********************//




// 设置速度控制参数命令: AT+MotorSpeedConfig=设备ID,电机号,磁极对数,滤波系数
AtCmdStatus_t MotorCmd_SpeedConfig(const char *params) {
    if (params == NULL) {
        return AT_PARAM_ERROR;
    }
    
    // 解析参数
    int dev_id, motor_num, poles;
    float filter_alpha;
    
    if (sscanf(params, "%d,%d,%d,%f", &dev_id, &motor_num, &poles, &filter_alpha) != 4) {
        return AT_PARAM_ERROR;
    }
    
    // 验证设备ID和电机号
    if (!CheckDeviceAndMotor(dev_id, motor_num)) {
        return AT_PARAM_ERROR;
    }
    
    // 验证参数
    if (poles <= 0 || poles > 20) {
        return AT_PARAM_ERROR;
    }
    
    if (filter_alpha < 0.0f || filter_alpha > 1.0f) {
        return AT_PARAM_ERROR;
    }
    
    // 获取电机索引 (0-3)
    uint8_t motor_idx = motor_num - 1;
    
    // 更新配置
    g_speed_sensors[motor_idx].config.magnet_poles = (uint8_t)poles;
    g_speed_sensors[motor_idx].config.filter_alpha = filter_alpha;
    
    // 响应
    AT_SendResponse("AT+MotorSpeedConfig=%d,%d,%d,%.2f",
                   dev_id, motor_num, poles, filter_alpha);
    
    return AT_OK;
}




// 控制模式切换命令: AT+MotorControlMode=设备ID,电机号,模式
// 模式: 0=普通PWM, 1=扭矩控制, 2=速度控制
AtCmdStatus_t MotorCmd_SetControlMode(const char *params) {
    if (params == NULL) {
        return AT_PARAM_ERROR;
    }
    
    // 解析参数
    int dev_id, motor_num, mode;
    if (sscanf(params, "%d,%d,%d", &dev_id, &motor_num, &mode) != 3) {
        return AT_PARAM_ERROR;
    }
    
    // 验证电机号
    if (motor_num < 1 || motor_num > 4) {
        return AT_PARAM_ERROR;
    }
    
    // 验证模式
    if (mode < 0 || mode > 2) {
        return AT_PARAM_ERROR;
    }
    
    // 设置控制模式
    uint8_t motor_idx = motor_num - 1;
    MotorControl_SetMode(motor_idx, (MotorControlMode_t)mode);
    
    // 响应
    AT_SendResponse("AT+MotorControlMode=%d,%d,%d", dev_id, motor_num, mode);
    
    return AT_OK;
}

// 扭矩控制命令: AT+MotorSetTorque=设备ID,电机号,电流值(mA)
AtCmdStatus_t MotorCmd_SetTorque(const char *params) {
    if (params == NULL) {
        return AT_PARAM_ERROR;
    }
    
    // 解析参数
    int dev_id, motor_num, current_ma;
    if (sscanf(params, "%d,%d,%d", &dev_id, &motor_num, &current_ma) != 3) {
        return AT_PARAM_ERROR;
    }
    
    // 验证电机号
    if (motor_num < 1 || motor_num > 4) {
        return AT_PARAM_ERROR;
    }
    
    // 设置电流(转换为安培)
    uint8_t motor_idx = motor_num - 1;
    float current_amps = (float)current_ma / 1000.0f;
    MotorControl_SetCurrent(motor_idx, current_amps);
    
    // 确保电机启用
    MotorControl_Enable(motor_idx, 1);
    
    // 响应
    AT_SendResponse("AT+MotorSetTorque=%d,%d,%d", dev_id, motor_num, current_ma);
    
    return AT_OK;
}

// 速度控制命令: AT+MotorSetSpeed=设备ID,电机号,速度(RPM)
AtCmdStatus_t MotorCmd_SetTargetSpeed(const char *params) {
    if (params == NULL) {
        return AT_PARAM_ERROR;
    }
    
    // 解析参数
    int dev_id, motor_num, speed_rpm;
    if (sscanf(params, "%d,%d,%d", &dev_id, &motor_num, &speed_rpm) != 3) {
        return AT_PARAM_ERROR;
    }
    
    // 验证电机号
    if (motor_num < 1 || motor_num > 4) {
        return AT_PARAM_ERROR;
    }
    
    // 设置目标速度
    uint8_t motor_idx = motor_num - 1;
    MotorControl_SetSpeed(motor_idx, (float)speed_rpm);
    
    // 确保电机启用
    MotorControl_Enable(motor_idx, 1);
    
    // 响应
    AT_SendResponse("AT+MotorSetSpeed=%d,%d,%d", dev_id, motor_num, speed_rpm);
    
    return AT_OK;
}

// 电流环PID参数设置: AT+MotorCurrentPID=设备ID,电机号,Kp,Ki,Kd
AtCmdStatus_t MotorCmd_SetCurrentPID(const char *params) {
    if (params == NULL) {
        return AT_PARAM_ERROR;
    }
    
    // 解析参数
    int dev_id, motor_num;
    float kp, ki, kd;
    if (sscanf(params, "%d,%d,%f,%f,%f", &dev_id, &motor_num, &kp, &ki, &kd) != 5) {
        return AT_PARAM_ERROR;
    }
    
    // 验证电机号
    if (motor_num < 1 || motor_num > 4) {
        return AT_PARAM_ERROR;
    }
    
    // 设置PID参数
    uint8_t motor_idx = motor_num - 1;
    MotorControl_SetCurrentPID(motor_idx, kp, ki, kd);
    
    // 响应
    AT_SendResponse("AT+MotorCurrentPID=%d,%d,%.2f,%.2f,%.2f", 
                   dev_id, motor_num, kp, ki, kd);
    
    return AT_OK;
}

// 速度环PID参数设置: AT+MotorSpeedPID=设备ID,电机号,Kp,Ki,Kd
AtCmdStatus_t MotorCmd_SetSpeedPID(const char *params) {
    if (params == NULL) {
        return AT_PARAM_ERROR;
    }
    
    // 解析参数
    int dev_id, motor_num;
    float kp, ki, kd;
    if (sscanf(params, "%d,%d,%f,%f,%f", &dev_id, &motor_num, &kp, &ki, &kd) != 5) {
        return AT_PARAM_ERROR;
    }
    
    // 验证电机号
    if (motor_num < 1 || motor_num > 4) {
        return AT_PARAM_ERROR;
    }
    
    // 设置PID参数
    uint8_t motor_idx = motor_num - 1;
    MotorControl_SetSpeedPID(motor_idx, kp, ki, kd);
    
    // 响应
    AT_SendResponse("AT+MotorSpeedPID=%d,%d,%.2f,%.2f,%.2f", 
                   dev_id, motor_num, kp, ki, kd);
    
    return AT_OK;
}

// 电机控制状态查询: AT+MotorControlQuery=设备ID,电机号
AtCmdStatus_t MotorCmd_QueryControlStatus(const char *params) {
    if (params == NULL) {
        return AT_PARAM_ERROR;
    }
    
    // 解析参数
    int dev_id, motor_num;
    if (sscanf(params, "%d,%d", &dev_id, &motor_num) != 2) {
        return AT_PARAM_ERROR;
    }
    
    // 验证电机号
    if (motor_num < 1 || motor_num > 4) {
        return AT_PARAM_ERROR;
    }
    
    // 获取电机状态
    uint8_t motor_idx = motor_num - 1;
    MotorControl_t* status = MotorControl_GetStatus(motor_idx);
    
    if (status == NULL) {
        return AT_EXECUTION_ERROR;
    }
    
    // 响应
    AT_SendResponse("AT+MotorControlQuery=%d,%d,%d,%.3f,%.1f", 
                   dev_id, motor_num, status->mode, 
                   status->actual_current, status->actual_speed);
    
    return AT_OK;
}


// 电机使能/禁用命令: AT+MotorEnable=设备ID,电机号,使能状态
AtCmdStatus_t MotorCmd_SetEnable(const char *params) {
    if (params == NULL) {
        return AT_PARAM_ERROR;
    }
    
    // 解析参数
    int dev_id, motor_num, enable;
    if (sscanf(params, "%d,%d,%d", &dev_id, &motor_num, &enable) != 3) {
        return AT_PARAM_ERROR;
    }
    
    // 验证电机号
    if (motor_num < 1 || motor_num > 4) {
        return AT_PARAM_ERROR;
    }
    
    // 验证设备ID (可以从你的现有代码中获取设备ID)
    if (dev_id != g_device_id) {
        return AT_PARAM_ERROR;
    }
    
    // 设置使能状态
    uint8_t motor_idx = motor_num - 1;
    MotorControl_Enable(motor_idx, enable ? 1 : 0);
    
    // 响应
    AT_SendResponse("AT+MotorEnable=%d,%d,%d", dev_id, motor_num, enable);
    
    return AT_OK;
}


// 周期性处理函数 (在主循环中调用)
// 在MotorCmd_PeriodicHandler函数中添加自定义运行处理逻辑
// 添加在处理完循环运行后
// 处理自定义运行
void MotorCmd_PeriodicHandler(void) {
    uint32_t current_time = HAL_GetTick();
    
    // 第一部分：处理电机定时运行
    for (uint8_t i = 0; i < 4; i++) {
        if (g_motor_enabled[i]) {
            // 处理单次运行
            if (g_motor_run[i].is_running && g_motor_run[i].runtime_ms > 0) {
                if (current_time - g_motor_run[i].start_time >= g_motor_run[i].runtime_ms) {
                    // 运行时间到，停止电机
                    Motor_SetState(i, MOTOR_DIR_STOP, 0);
                    g_motor_run[i].is_running = 0;
                    
                    // 发送运行完成消息
                    AT_SendResponse("AT+MotorRun=%d,%d,%d,%d,%d,1",
                                   g_device_id,
                                   i + 1,
                                   g_motor_enabled[i],
                                   g_motor_run[i].pwm,
                                   ms_to_us(g_motor_run[i].runtime_ms));
                }
            }
            
            // 处理循环运行
            if (g_motor_repeat[i].is_running) {
                uint32_t elapsed = current_time - g_motor_repeat[i].last_switch_time;
                uint8_t state_changed = 0;
                
                // 状态机处理
                switch(g_motor_repeat[i].current_state) {
                    case 0: // 正在正转
                        if (elapsed >= g_motor_repeat[i].cw_time_ms) {
                            // 切换到正转停止状态
                            Motor_SetState(i, MOTOR_DIR_STOP, 0);
                            g_motor_repeat[i].current_state = 1;
                            g_motor_repeat[i].last_switch_time = current_time;
                            state_changed = 1;
                        }
                        break;
                        
                    case 1: // 正转停止状态
                        if (elapsed >= g_motor_repeat[i].cw_stop_time_ms) {
                            // 切换到反转状态
                            Motor_SetState(i, MOTOR_DIR_CCW, g_motor_repeat[i].pwm);
                            g_motor_repeat[i].current_state = 2;
                            g_motor_repeat[i].last_switch_time = current_time;
                            state_changed = 1;
                        }
                        break;
                        
                    case 2: // 正在反转
                        if (elapsed >= g_motor_repeat[i].ccw_time_ms) {
                            // 切换到反转停止状态
                            Motor_SetState(i, MOTOR_DIR_STOP, 0);
                            g_motor_repeat[i].current_state = 3;
                            g_motor_repeat[i].last_switch_time = current_time;
                            state_changed = 1;
                        }
                        break;
                        
                    case 3: // 反转停止状态
                        if (elapsed >= g_motor_repeat[i].ccw_stop_time_ms) {
                            // 完成一个完整循环
                            g_motor_repeat[i].current_cycle++;
                            
                            // 发送完成一个循环的消息
                            AT_SendResponse("AT+MotorRunRepeat=%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                                           g_device_id,
                                           i + 1,
                                           g_motor_enabled[i],
                                           g_motor_repeat[i].pwm,
                                           ms_to_us(g_motor_repeat[i].cw_time_ms),
                                           ms_to_us(g_motor_repeat[i].cw_stop_time_ms),
                                           ms_to_us(g_motor_repeat[i].ccw_time_ms),
                                           ms_to_us(g_motor_repeat[i].ccw_stop_time_ms),
                                           g_motor_repeat[i].max_cycles,
                                           g_motor_repeat[i].current_cycle);
                            
                            // 检查是否达到最大循环次数
                            if (g_motor_repeat[i].max_cycles > 0 && 
                                g_motor_repeat[i].current_cycle >= g_motor_repeat[i].max_cycles) {
                                // 已完成所有循环，停止运行
                                g_motor_repeat[i].is_running = 0;
                            } else {
                                // 回到正转状态，开始新的循环
                                Motor_SetState(i, MOTOR_DIR_CW, g_motor_repeat[i].pwm);
                                g_motor_repeat[i].current_state = 0;
                                g_motor_repeat[i].last_switch_time = current_time;
                                state_changed = 1;
                            }
                        }
                        break;
                }
            }
            
            // 处理自定义运行
            if (g_motor_custom[i].is_running) {
                uint32_t elapsed = current_time - g_motor_custom[i].cycle_start_time;
    
                // 检查是否完成一个周期
                if (elapsed >= g_motor_custom[i].total_period_ms) {
                    // 完成一个周期
                    g_motor_custom[i].current_cycle++;
        
                    // 发送完成一个周期的消息
                    AT_SendResponse("AT+MotorRunCustom=%d,%d,%d,%d,%d,%d,%d",
                            g_motor_custom[i].device_id,
                            g_motor_custom[i].motor_num,
                            g_motor_enabled[i],
                            g_motor_custom[i].total_period_ms,
                            g_motor_custom[i].max_cycles,
                            g_motor_custom[i].phase_count,
                            g_motor_custom[i].current_cycle);
        
                    // 检查是否达到最大循环次数
                    if (g_motor_custom[i].max_cycles > 0 && 
                        g_motor_custom[i].current_cycle >= g_motor_custom[i].max_cycles) {
                        // 已完成所有循环，停止运行
                        g_motor_custom[i].is_running = 0;
                        Motor_SetState(i, MOTOR_DIR_STOP, 0);
                    } 
                    else {
                        // 开始新周期
                        g_motor_custom[i].cycle_start_time = current_time;
                        g_motor_custom[i].current_phase = 0;
            
                        // 设置初始阶段的电机状态
                        MotorPhase_t *first_phase = &g_motor_custom[i].phases[0];
                        Motor_SetState(i, first_phase->direction, first_phase->pwm);
                    }
                } 
                else {
                    // 检查是否需要切换到下一个阶段
                    uint8_t current_phase = g_motor_custom[i].current_phase;
                    uint8_t next_phase = current_phase + 1;
        
                    if (next_phase < g_motor_custom[i].phase_count) {
                        uint32_t next_phase_start = g_motor_custom[i].phases[next_phase].start_time_ms;
            
                        if (elapsed >= next_phase_start) {
                            // 切换到下一个阶段
                            g_motor_custom[i].current_phase = next_phase;
                            MotorPhase_t *phase = &g_motor_custom[i].phases[next_phase];
                            Motor_SetState(i, phase->direction, phase->pwm);
                        }
                    }
                }
				
				
				
            }
			
			
			
			// 处理状态上报
            if (g_status_upload[i].enabled) {
                if (current_time - g_status_upload[i].last_time >= g_status_upload[i].period_ms) {
                    g_status_upload[i].last_time = current_time;
                    
                    // 上报电机状态
                    MotorStatus_t status;
                    Motor_GetStatus(i, &status);
                    
                    AT_SendResponse("AT+MotorStatusUpLoad=%d,%d,%d,%d,%d,%d,%d,%d,%d,%s",
                                   status.id,
                                   status.motor_num,
                                   status.enabled,
                                   status.voltage_mv,
                                   status.current_ma,
                                   status.direction,
                                   status.temperature,
                                   status.speed,
                                   status.error,
                                   status.hw_version);
                }
            }
        }
		
		
		
    } // 结束第一部分的电机控制循环

    
}

// 函数结束