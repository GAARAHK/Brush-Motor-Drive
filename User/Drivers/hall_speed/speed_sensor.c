#include "speed_sensor.h"
#include "log.h"
#include "motor_control.h"
#include "tim.h"


// 全局速度传感器实例
extern SpeedSensor_t g_speed_sensors[4];


// 初始化速度传感器
void SpeedSensor_Init(SpeedSensor_t *sensor, TIM_HandleTypeDef *htim, 
                     uint32_t channel, SpeedSensorConfig_t config) {
						 
    sensor->htim = htim;
    sensor->channel = channel;
    sensor->last_capture = 0;
    sensor->period = 0;
    sensor->last_update = HAL_GetTick();
    sensor->rpm = 0.0f;
    sensor->filtered_rpm = 0.0f;
    sensor->valid = false;
    sensor->config = config;
    
    // 启动输入捕获
    HAL_TIM_IC_Start_IT(htim, channel);
    
    LOG_INFO("Speed sensor initialized for TIM%d CH%d\r\n", 
             (int)htim->Instance == (int)TIM1 ? 1 : 
             (int)htim->Instance == (int)TIM2 ? 2 : 
             (int)htim->Instance == (int)TIM3 ? 3 : 17,
             channel == TIM_CHANNEL_1 ? 1 : 
             channel == TIM_CHANNEL_2 ? 2 : 
             channel == TIM_CHANNEL_3 ? 3 : 4);
}


// 速度传感器初始化
void SpeedSensors_Init(void) {
    // 配置4个速度传感器
    SpeedSensorConfig_t config = {
        .magnet_poles = 4,       // 默认1对磁极，根据实际磁环调整
        .timeout_ms = 500,       // 500ms超时时间
        .filter_alpha = 0.1f,    // 低通滤波系数
        .min_period = 100,        // 最小有效周期(防噪声)
     	.max_period = 60000,     // 最大周期(约1RPM下限)
        .max_rpm = 10000.0f      // 最大合理转速
		
    };
	
	// 注意：这里使用TIM17而非TIM4
    extern TIM_HandleTypeDef htim17;
	
	
    
    // 初始化4个速度传感器
    SpeedSensor_Init(&g_speed_sensors[1], &htim17, TIM_CHANNEL_1, config);  //配置定时器17的1通道为传感器0
//    SpeedSensor_Init(&g_speed_sensors[1], &htim17, TIM_CHANNEL_2, config);
//    SpeedSensor_Init(&g_speed_sensors[2], &htim17, TIM_CHANNEL_3, config);
//    SpeedSensor_Init(&g_speed_sensors[3], &htim17, TIM_CHANNEL_4, config);
    
    // 关联电机与速度传感器
    MotorControl_AssignSpeedSensor(1, &g_speed_sensors[1]);  //然后关联0传感器与1号电机
//    MotorControl_AssignSpeedSensor(1, &g_speed_sensors[1]);
//    MotorControl_AssignSpeedSensor(2, &g_speed_sensors[2]);
//    MotorControl_AssignSpeedSensor(3, &g_speed_sensors[3]);
    
    LOG_INFO("Speed sensors initialized and assigned to motors\r\n");
}


// 初始化速度传感器和电机控制
void Init_SpeedControl(void) {
    
	
	// 初始化多模式电机控制
    MotorControl_Init();
    
    // 初始化速度传感器
    SpeedSensors_Init();
    
    
    
    // 启动电流环定时器(5kHz)
    HAL_TIM_Base_Start_IT(&htim6);
    
    LOG_INFO("Speed control system initialized\r\n");
}



// 处理输入捕获事件
void SpeedSensor_CaptureCallback(SpeedSensor_t *sensor, uint32_t capture) {
    uint32_t period;
    
     // 计算周期，处理计数器溢出情况
    if (capture >= sensor->last_capture) {
        period = capture - sensor->last_capture;
    } else {
        // 溢出情况
        period = (0xFFFF - sensor->last_capture) + capture + 1;
    }
    
    // 检查最小有效周期，过滤噪声
    if (period < sensor->config.min_period) {
        // 周期过短，可能是噪声，忽略此次测量
        sensor->last_capture = capture;
        return;
    }
    
    // 检查最大有效周期，防止低速下测量值异常
    if (period > sensor->config.max_period) {
        // 周期过长，限制到最大值
        period = sensor->config.max_period;
    }
	
	 // 更新有效状态和时间戳
    sensor->period = period;
    sensor->last_update = HAL_GetTick();
    sensor->valid = true;
	

    // 计算RPM (假设定时器时钟为1MHz)
    float timer_freq = 1000000.0f; // 1MHz (根据实际定时器配置调整)
    float period_seconds = (float)period / timer_freq;
    
    // 注意：这里使用霍尔传感器每转触发的脉冲数
    float pulses_per_revolution = (float)sensor->config.magnet_poles * 1.0f; // 每对磁极产生2个脉冲,霍尔元件触发有全极、双极,目前一对磁极只产生一个脉冲
    float rpm = 60.0f / (period_seconds * pulses_per_revolution);
    
    // 检查速度的合理性
    if (rpm > sensor->config.max_rpm) {
        rpm = sensor->config.max_rpm;
    }
    
    // 应用方向因子
    sensor->raw_rpm = rpm;
    
    // 应用滤波 - 使用低通滤波增强稳定性
    if (sensor->filtered_rpm == 0.0f) {
        sensor->filtered_rpm = rpm; // 首次初始化
    } else {
        // 使用较小的滤波系数确保更平滑的速度变化
        sensor->filtered_rpm = sensor->filtered_rpm * (1.0f - sensor->config.filter_alpha) + 
                           rpm * sensor->config.filter_alpha;
    }
    
    // 记录此次脉冲时间用于下次计算
    sensor->last_capture = capture;
    
    // 更新脉冲计数，用于诊断
    sensor->pulse_count++;
}

// 更新速度传感器状态
void SpeedSensor_Update(SpeedSensor_t *sensor) {
    // 检查是否超时（电机可能已停止）
    uint32_t current_time = HAL_GetTick();
    if (current_time - sensor->last_update > sensor->config.timeout_ms) {
        // 超时，认为电机已停止
        sensor->rpm = 0.0f;
        sensor->filtered_rpm = 0.0f;
        sensor->valid = false;
    }
}

// 获取当前RPM
float SpeedSensor_GetRPM(SpeedSensor_t *sensor) {
    return sensor->filtered_rpm;
}

// 检查速度值是否有效
bool SpeedSensor_IsValid(SpeedSensor_t *sensor) {
    return sensor->valid;
}