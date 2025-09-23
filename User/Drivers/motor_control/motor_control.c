#include "motor_control.h"
#include "log.h"
#include <math.h>

// 全局电机控制状态数组
MotorControl_t motor_control[4];


// 记录每个电机当前的PWM设置
float mtd_current_pwm[4] = {0};


// 电流环控制频率(Hz)
#define CURRENT_LOOP_FREQ     5000.0f  // 5kHz
#define SPEED_LOOP_FREQ       200.0f   // 200Hz

// 电机常数
#define MOTOR_KT              0.02f    // 转矩常数，牛米/安培
#define MOTOR_J               0.0001f  // 转动惯量，kg*m^2
#define MOTOR_B               0.0005f  // 阻尼系数
#define MOTOR_RATED_CURRENT   1.0f     // 额定电流，安培
#define MOTOR_MAX_SPEED       8000.0f  // 最大速度，RPM


// 获取当前PWM设置函数
float GetMTD1CurrentPWM(void) { return mtd_current_pwm[0]; }
float GetMTD2CurrentPWM(void) { return mtd_current_pwm[1]; }
float GetMTD3CurrentPWM(void) { return mtd_current_pwm[2]; }
float GetMTD4CurrentPWM(void) { return mtd_current_pwm[3]; }



// 更新PID控制器
static float PID_Update(PID_Controller_t *pid, float measured_value) {
    // 计算误差
    float error = pid->setpoint - measured_value;
    
    // 计算微分项
    float derivative = (error - pid->prev_error) / pid->sample_time;
    
    // 更新积分项
    pid->integral += error * pid->sample_time;
    
    // 积分限幅
    if (pid->integral > pid->output_max)
        pid->integral = pid->output_max;
    else if (pid->integral < pid->output_min)
        pid->integral = pid->output_min;
    
    // 计算PID输出
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    
    // 输出限幅
    if (output > pid->output_max)
        output = pid->output_max;
    else if (output < pid->output_min)
        output = pid->output_min;
    
    // 保存当前误差
    pid->prev_error = error;
    
    return output;
}

// 根据输出调节电机
static void ApplyMotorOutput(uint8_t motor_idx, float output) {
    // 输出范围转换为占空比百分比
    int8_t duty_percent = (int8_t)output;
    
	
	// 	
    mtd_current_pwm[motor_idx] = (float)duty_percent;
	
	
    // 调用DRV8870驱动函数控制电机
    switch (motor_idx) {
        case 0: MTD1_SetSpeedPercent(duty_percent); break;
        case 1: MTD2_SetSpeedPercent(duty_percent); break;
        case 2: MTD3_SetSpeedPercent(duty_percent); break;
        case 3: MTD4_SetSpeedPercent(duty_percent); break;
    }
    
    // 更新方向信息
    if (duty_percent > 0)
        motor_control[motor_idx].direction = 1;
    else if (duty_percent < 0)
        motor_control[motor_idx].direction = -1;
    else
        motor_control[motor_idx].direction = 0;
}

// 初始化电机控制系统
void MotorControl_Init(void) {
    // 初始化每个电机的控制结构
    for (uint8_t i = 0; i < 4; i++) {
        motor_control[i].motor_idx = i;
        motor_control[i].mode = CONTROL_MODE_DUTY;
        motor_control[i].duty_setpoint = 0.0f;
        motor_control[i].current_setpoint = 0.0f;
        motor_control[i].speed_setpoint = 0.0f;
        motor_control[i].actual_current = 0.0f;
        motor_control[i].actual_speed = 0.0f;
        motor_control[i].last_update_time = 0;
        motor_control[i].direction = 0;
        motor_control[i].enabled = 0;
        motor_control[i].speed_sensor = NULL;
		
		
        // 配置电流环PID(默认值)
        motor_control[i].current_pid.kp = 30.0f;
        motor_control[i].current_pid.ki = 200.0f;
        motor_control[i].current_pid.kd = 0.0f;
        motor_control[i].current_pid.setpoint = 0.0f;
        motor_control[i].current_pid.integral = 0.0f;
        motor_control[i].current_pid.prev_error = 0.0f;
        motor_control[i].current_pid.output_min = -100.0f;
        motor_control[i].current_pid.output_max = 100.0f;
        motor_control[i].current_pid.sample_time = 1.0f / CURRENT_LOOP_FREQ;
        
        // 配置速度环PID(默认值)
        motor_control[i].speed_pid.kp = 0.02f;
        motor_control[i].speed_pid.ki = 0.05f;
        motor_control[i].speed_pid.kd = 0.000f;
        motor_control[i].speed_pid.setpoint = 0.0f;
        motor_control[i].speed_pid.integral = 0.0f;
        motor_control[i].speed_pid.prev_error = 0.0f;
        motor_control[i].speed_pid.output_min = -MOTOR_RATED_CURRENT;
        motor_control[i].speed_pid.output_max = MOTOR_RATED_CURRENT;
        motor_control[i].speed_pid.sample_time = 1.0f / SPEED_LOOP_FREQ;
    }
    
    LOG_INFO("Motor control system initialized with current loop at %dHz\r\n", 
             (int)CURRENT_LOOP_FREQ);
}


// 关联电机与霍尔传感器
void MotorControl_AssignSpeedSensor(uint8_t motor_idx, SpeedSensor_t *sensor) {
    if (motor_idx >= 4)
        return;
	
        
    motor_control[motor_idx].speed_sensor = sensor;
    LOG_INFO("Speed sensor assigned to motor %d\r\n", motor_idx + 1);
}



// 设置电机控制模式
void MotorControl_SetMode(uint8_t motor_idx, MotorControlMode_t mode) {
    if (motor_idx >= 4)
        return;
	
	LOG_INFO("Setting motor %d to mode %d, sensor=%p\r\n", 
         motor_idx+1, mode, motor_control[motor_idx].speed_sensor);
	
	 // 检查是否可以切换到速度模式
    if (mode == CONTROL_MODE_SPEED && motor_control[motor_idx].speed_sensor == NULL) {
        LOG_WARN("Cannot switch motor %d to speed mode: no speed sensor assigned\r\n", motor_idx + 1);
        return;
    }
	
	
    
    // 切换模式前重置积分器
    motor_control[motor_idx].current_pid.integral = 0.0f;
    motor_control[motor_idx].speed_pid.integral = 0.0f;
    
    // 设置新模式
    motor_control[motor_idx].mode = mode;
    
    const char* mode_names[] = {"PWM Duty", "Torque", "Speed"};
    LOG_INFO("Motor %d mode changed to %s control\r\n", 
             motor_idx + 1, mode_names[mode]);
}

// 设置占空比(普通模式)
void MotorControl_SetDuty(uint8_t motor_idx, float duty_percent) {
    if (motor_idx >= 4)
        return;
    
    // 限制占空比范围
    if (duty_percent > 100.0f)
        duty_percent = 100.0f;
    else if (duty_percent < -100.0f)
        duty_percent = -100.0f;
    
    motor_control[motor_idx].duty_setpoint = duty_percent;
    
    // 在普通模式下直接设置输出
    if (motor_control[motor_idx].mode == CONTROL_MODE_DUTY && motor_control[motor_idx].enabled) {
        ApplyMotorOutput(motor_idx, duty_percent);
    }
}

// 设置电流(扭矩模式)
void MotorControl_SetCurrent(uint8_t motor_idx, float current_amps) {
    if (motor_idx >= 4)
        return;
    
    // 限制电流范围
    if (current_amps > MOTOR_RATED_CURRENT)
        current_amps = MOTOR_RATED_CURRENT;
    else if (current_amps < -MOTOR_RATED_CURRENT)
        current_amps = -MOTOR_RATED_CURRENT;
    
    motor_control[motor_idx].current_setpoint = current_amps;
    motor_control[motor_idx].current_pid.setpoint = current_amps;
}

// 设置速度(速度模式)
void MotorControl_SetSpeed(uint8_t motor_idx, float speed_rpm) {
    if (motor_idx >= 4)
        return;
    
	// 检查速度传感器
    if (motor_control[motor_idx].speed_sensor == NULL) {
        LOG_WARN("Cannot set speed for motor %d: no speed sensor assigned\r\n", motor_idx + 1);
        return;
    }
    
	
	
    // 限制速度范围
    if (speed_rpm > MOTOR_MAX_SPEED)
        speed_rpm = MOTOR_MAX_SPEED;
    else if (speed_rpm < -MOTOR_MAX_SPEED)
        speed_rpm = -MOTOR_MAX_SPEED;
    
    motor_control[motor_idx].speed_setpoint = speed_rpm;
    motor_control[motor_idx].speed_pid.setpoint = speed_rpm;
	
	LOG_INFO("Motor %d speed setpoint: %.1f RPM\r\n", motor_idx + 1, speed_rpm);
}

// 启用/禁用电机
void MotorControl_Enable(uint8_t motor_idx, uint8_t enable) {
    if (motor_idx >= 4)
        return;
    
    if (enable) {
        motor_control[motor_idx].enabled = 1;
    } else {
        motor_control[motor_idx].enabled = 0;
        // 禁用电机时停止输出
        switch (motor_idx) {
            case 0: MTD1_Coast(); break;
            case 1: MTD2_Coast(); break;
            case 2: MTD3_Coast(); break;
            case 3: MTD4_Coast(); break;
        }
        motor_control[motor_idx].direction = 0;
    }
}

// 电流环控制执行函数(在定时器中断中调用，5kHz频率)
void MotorControl_CurrentLoopUpdate(void) {
    // 处理每个电机的电流控制
    for (uint8_t i = 0; i < 4; i++) {
        if (!motor_control[i].enabled)
            continue;
            
        // 读取当前电流值(使用滤波后的值)
        motor_control[i].actual_current = g_curr_amp_filt[i];
        
        // 根据不同模式处理控制逻辑
        switch (motor_control[i].mode) {
            case CONTROL_MODE_DUTY:
                // 占空比模式，无需电流环，直接设置占空比
                ApplyMotorOutput(i, motor_control[i].duty_setpoint);
                break;
                
            case CONTROL_MODE_TORQUE:
                // 扭矩模式，使用电流环控制
                {
                    float output = PID_Update(&motor_control[i].current_pid, motor_control[i].actual_current);
                    ApplyMotorOutput(i, output);
                }
                break;
                
            case CONTROL_MODE_SPEED:
                // 速度模式，电流环为内环，由速度环设定电流
                {
                    float output = PID_Update(&motor_control[i].current_pid, motor_control[i].actual_current);
                    ApplyMotorOutput(i, output);
                }
                break;
        }
    }
}

// 速度估算(无编码器情况下，基于电流和电机特性估算速度)
static float EstimateSpeed(uint8_t motor_idx) {
    // 这里是一个简化的速度估算模型
    // 实际应用中，可以使用更复杂的模型或添加编码器反馈
    float current = motor_control[motor_idx].actual_current;
    float direction = (float)motor_control[motor_idx].direction;
    
    // 简化模型：假设速度与电流成正比，但考虑摩擦和负载
    // 速度 = (转矩常数 * 电流 - 静摩擦转矩) / 阻尼系数
    float static_friction = 0.1f; // 静摩擦转矩(Nm)
    float torque = fabs(current) * MOTOR_KT;
    float friction_effect = (torque > static_friction) ? static_friction : torque;
    float net_torque = torque - friction_effect;
    
    // 简化计算，将净转矩转换为rpm
    float estimated_rpm = net_torque / MOTOR_B * 9.55f; // 9.55 = 60/(2*pi)转换为rpm
    
    // 考虑方向
    if (direction != 0) {
        estimated_rpm *= direction;
    } else {
        estimated_rpm = 0;
    }
    
    return estimated_rpm;
}

// 速度环控制执行函数(在中断或主循环中调用，100-500Hz频率)
void MotorControl_SpeedLoopUpdate(void) {
    // 处理每个电机的速度控制
    for (uint8_t i = 0; i < 4; i++) {
        if (!motor_control[i].enabled || motor_control[i].mode != CONTROL_MODE_SPEED)
            continue;
        
       // 检查速度传感器
        if (motor_control[i].speed_sensor == NULL)
            continue;
            
        // 更新速度传感器状态
        SpeedSensor_Update(motor_control[i].speed_sensor);
        
        // 获取实际速度
        if (SpeedSensor_IsValid(motor_control[i].speed_sensor)) {
            // 有效速度值
            float sensor_rpm = SpeedSensor_GetRPM(motor_control[i].speed_sensor);
            
            // 根据电机当前PWM方向确定速度符号，而不是根据之前的direction变量
            // 这样可以确保速度符号与电机实际运动方向一致
            float current_pwm = 0;
            switch (i) {
                case 0: current_pwm = GetMTD1CurrentPWM(); break;
                case 1: current_pwm = GetMTD2CurrentPWM(); break;
                case 2: current_pwm = GetMTD3CurrentPWM(); break;
                case 3: current_pwm = GetMTD4CurrentPWM(); break;
            }
            
            // 根据PWM值确定方向，而不是direction变量
            if (current_pwm != 0) {
                motor_control[i].actual_speed = (current_pwm > 0) ? 
                                                sensor_rpm : -sensor_rpm;
            } else {
                // 如果PWM为0，则速度也应该为0
                motor_control[i].actual_speed = 0;
            }
            
           
        } else {
            // 无效速度值，可能电机已停止
            motor_control[i].actual_speed = 0.0f;
        }
        
        // 添加死区逻辑，防止小信号抖动
        float speed_error = motor_control[i].speed_pid.setpoint - motor_control[i].actual_speed;
        if (fabsf(speed_error) < 50.0f) { // 50 RPM的死区
            // 微小误差，可以认为已经达到目标
            speed_error = 0;
        }
        
        // 使用修正后的误差计算
        motor_control[i].speed_pid.prev_error = speed_error;
        
        // 速度环计算新的电流设定值
        float current_setpoint = PID_Update(&motor_control[i].speed_pid, motor_control[i].actual_speed);
        
        // 更新电流环设定值
        motor_control[i].current_setpoint = current_setpoint;
        motor_control[i].current_pid.setpoint = current_setpoint;
		
		// 调试输出 (降低输出频率以避免日志过多)
        static uint32_t last_debug_time = 0;
        uint32_t current_time = HAL_GetTick();
        if (current_time - last_debug_time > 1000) { // 每500ms输出一次
            last_debug_time = current_time;
            LOG_DEBUG("Motor %d: Target=%.1f, Actual=%.1f RPM, Current=%.2f A\r\n",
                     i + 1, motor_control[i].speed_setpoint, motor_control[i].actual_speed, 
                     motor_control[i].actual_current);
        }
    }
}

// 配置电流环PID参数
void MotorControl_SetCurrentPID(uint8_t motor_idx, float kp, float ki, float kd) {
    if (motor_idx >= 4)
        return;
    
    motor_control[motor_idx].current_pid.kp = kp;
    motor_control[motor_idx].current_pid.ki = ki;
    motor_control[motor_idx].current_pid.kd = kd;
    motor_control[motor_idx].current_pid.integral = 0.0f; // 重置积分项
	
	 LOG_INFO("Motor %d current PID set to Kp=%.2f, Ki=%.2f, Kd=%.2f\r\n",
            motor_idx + 1, kp, ki, kd);
}

// 配置速度环PID参数
void MotorControl_SetSpeedPID(uint8_t motor_idx, float kp, float ki, float kd) {
    if (motor_idx >= 4)
        return;
    
    motor_control[motor_idx].speed_pid.kp = kp;
    motor_control[motor_idx].speed_pid.ki = ki;
    motor_control[motor_idx].speed_pid.kd = kd;
    motor_control[motor_idx].speed_pid.integral = 0.0f; // 重置积分项
	
	LOG_INFO("Motor %d speed PID set to Kp=%.2f, Ki=%.2f, Kd=%.2f\r\n",
            motor_idx + 1, kp, ki, kd);
}

// 获取电机控制状态
MotorControl_t* MotorControl_GetStatus(uint8_t motor_idx) {
    if (motor_idx >= 4)
        return NULL;
    
    return &motor_control[motor_idx];
}