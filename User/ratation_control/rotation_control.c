#include "rotation_control.h"
#include <string.h>
#include <stdio.h>
#include "log.h"
#include "usart.h"
// 假设MTD1驱动函数在某个头文件中声明
extern void MTD4_SetSpeedPercent(int8_t speed_percent);
extern void MTD2_SetSpeedPercent(int8_t speed_percent);

// 光电开关状态结构体
typedef struct {
    uint8_t current_state;        // 当前读取状态
    uint8_t previous_state;       // 上一次状态
    uint8_t debounce_count;       // 防抖计数
    uint8_t stable_state;         // 稳定状态
    uint8_t state_history[10];    // 状态历史记录(最新值在index 0)
    uint8_t history_count;        // 历史记录计数
} PhotoSwitch_t;

// 全局变量
static PhotoSwitch_t photo_switch1 = {0};
static PhotoSwitch_t photo_switch2 = {0};
static SystemPosition_t current_position = POSITION_UNKNOWN;
static SystemState_t system_state = STATE_IDLE;
static FlowStep_t current_step = STEP_INIT;
static uint8_t motor_running = 0;
static int8_t motor_direction = 0;  // +1 为顺时针, -1 为逆时针
static uint32_t hold_start_time = 0;
static uint32_t step_hold_times[STEP_MAX_COUNT] = {0}; // 每个步骤的停留时间
//static uint32_t hold_time_ms = 10000; // 默认10秒停留时间
static uint16_t remaining_cycles = 0;
static uint8_t position_reported = 0;
static uint8_t motor_speed_percent = 100; // 默认100%速度
static uint8_t motor2_speed_percent = 100; // MTD2默认100%速度


// 在全局变量区添加
static uint8_t braking_active = 0;           // 制动激活标志
static uint32_t braking_start_time = 0;      // 制动开始时间
static uint8_t braking_phase = 0;            // 制动阶段
#define BRAKE_PULSE_DURATION 20             // 制动脉冲持续时间(ms)
#define BRAKE_PULSE_POWER 55                 // 制动脉冲力度(%)



// GPIO定义 - 请根据实际硬件调整
#define PHOTO_SWITCH1_PORT     GPIOC
#define PHOTO_SWITCH1_PIN      GPIO_PIN_14
#define PHOTO_SWITCH2_PORT     GPIOC
#define PHOTO_SWITCH2_PIN      GPIO_PIN_15


// 外部UART句柄 - 根据实际项目调整
extern UART_HandleTypeDef huart1;


// 初始化控制系统
void RotationControl_Init(void)
{
    // 停止电机，确保安全状态
    MTD4_SetSpeedPercent(0);
	MTD2_SetSpeedPercent(0);
    motor_running = 0;
    
    // 初始化状态变量
    system_state = STATE_IDLE;
    current_step = STEP_INIT;
    current_position = POSITION_UNKNOWN;
    
	// 初始化默认的步骤停留时间
    step_hold_times[STEP_INIT] = 1000;           // 初始位置停留1秒
    step_hold_times[STEP_PUMP_PREP] = 2000;      // 抽水预备位停留2秒
    step_hold_times[STEP_PUMP_FORMAL] = 6000;    // 抽水正式位停留6秒
    step_hold_times[STEP_RETURN_ZERO_1] = 5000;  // 第一次回零停留5秒
    step_hold_times[STEP_DRAIN_PREP] = 2000;     // 排水预备位停留2秒
    step_hold_times[STEP_DRAIN_FORMAL] = 10000;  // 排水正式位停留10秒
    step_hold_times[STEP_RETURN_ZERO_2] = 1000;  // 第二次回零停留1秒
    step_hold_times[STEP_COMPLETE] = 0;          // 完成步骤无需停留
	
	
    // 初始化光电开关状态
    memset(&photo_switch1, 0, sizeof(PhotoSwitch_t));
    memset(&photo_switch2, 0, sizeof(PhotoSwitch_t));
}

// 更新光电开关状态和历史记录
static void UpdatePhotoSwitchState(PhotoSwitch_t *sw, uint8_t raw_state)
{
    sw->previous_state = sw->current_state;
    sw->current_state = raw_state;
    
    // 防抖处理
    if (sw->current_state == sw->previous_state) {
        if (sw->debounce_count < 1)  
            sw->debounce_count++;
        
        if (sw->debounce_count >= 1 && sw->stable_state != sw->current_state) {
            // 状态变化，更新历史记录
            // 将所有历史记录向后移动一位
            for (int i = 9; i > 0; i--) {
                sw->state_history[i] = sw->state_history[i-1];
            }
            sw->state_history[0] = sw->current_state;
            sw->history_count++;
            
            sw->stable_state = sw->current_state;
        }
    } else {
        sw->debounce_count = 0;
    }
}


// 检查光电开关是否匹配特定的变化模式
// pattern: 状态模式字符串，例如"101"表示1→0→1
// 返回1表示匹配，0表示不匹配
static uint8_t MatchPhotoSwitchPattern(PhotoSwitch_t *sw, const char *pattern)
{
    size_t pattern_len = strlen(pattern);
    if (sw->history_count < pattern_len)
        return 0;
    
    for (size_t i = 0; i < pattern_len; i++) {
        char expected = pattern[pattern_len - 1 - i] - '0';
        if (sw->state_history[i] != expected)
            return 0;
    }
    return 1;
}

// 启动电机
static void StartMotor(int8_t direction)
{
    motor_direction = direction;
    
    // 根据方向和速度设置电机
    // 正值为顺时针，负值为逆时针
    if (direction > 0) {
        // 顺时针旋转
        MTD4_SetSpeedPercent(motor_speed_percent);
    } else {
        // 逆时针旋转
        MTD4_SetSpeedPercent(-motor_speed_percent);
    }
	
	// 确保MTD2停止 - 只在旋转阶段停止
    MTD2_SetSpeedPercent(0);
    
    motor_running = 1;
}

// 停止电机
static void StopMotor(void)
{
    if (motor_running) {
        MTD4_SetSpeedPercent(0);
        motor_running = 0;
    }
	// 注意: 不在这里停止MTD2，因为在保持阶段需要单独控制
}

// 修改停止电机函数，实现非阻塞制动
static void StartBraking(void)
{
    if (!motor_running) return;
    
    // 设置制动状态
    braking_active = 1;
    braking_phase = 0;
    braking_start_time = HAL_GetTick();
    
    // 应用反向制动脉冲
    int8_t brake_power = BRAKE_PULSE_POWER;
    if (motor_direction > 0) {
        MTD4_SetSpeedPercent(-brake_power);
    } else {
        MTD4_SetSpeedPercent(brake_power);
    }
}


// 设置第二个电机状态
static void SetMotor2State(FlowStep_t step)
{
    // 根据当前步骤设置MTD2电机
    switch (step) {
        case STEP_PUMP_PREP:      // 抽水预备位: MTD2反转
        case STEP_PUMP_FORMAL:    // 抽水正式位: MTD2反转
        case STEP_DRAIN_PREP:     // 排水预备位: MTD2反转
            //MTD2_SetSpeedPercent(-motor2_speed_percent); // 反转
			MTD2_SetSpeedPercent(motor2_speed_percent); // 正转
            break;
            
        case STEP_DRAIN_FORMAL:   // 排水正式位: MTD2正转
            MTD2_SetSpeedPercent(motor2_speed_percent);  // 正转
            break;
            
        default:                  // 其他所有位置: MTD2停止
            MTD2_SetSpeedPercent(0); // 停止
            break;
    }
}


// 重置光电开关历史记录
static void ResetPhotoSwitchHistory(PhotoSwitch_t *sw)
{
    memset(sw->state_history, 0, sizeof(sw->state_history));
    sw->history_count = 0;
}


// 检查是否在0位
static uint8_t IsAtZeroPosition(void)
{
    return (photo_switch1.stable_state == 1 && photo_switch2.stable_state == 1);
}


// 主控制处理函数，放在定时器中断或主循环中调用
void RotationControl_Process(void)
{
	 // 处理制动逻辑
    if (braking_active) {
        uint32_t current_time = HAL_GetTick();
        uint32_t elapsed = current_time - braking_start_time;
        
        if (braking_phase == 0 && elapsed >= BRAKE_PULSE_DURATION) {
            // 制动脉冲结束，切换到零速度
            MTD4_SetSpeedPercent(0);
            braking_phase = 1;
            braking_active = 0;
            motor_running = 0;
        }
        
        // 制动期间不处理其他逻辑
        if (braking_active) return;
    }
	
	
    // 读取光电开关状态
    UpdatePhotoSwitchState(&photo_switch1, 
                          HAL_GPIO_ReadPin(PHOTO_SWITCH1_PORT, PHOTO_SWITCH1_PIN));
    UpdatePhotoSwitchState(&photo_switch2, 
                          HAL_GPIO_ReadPin(PHOTO_SWITCH2_PORT, PHOTO_SWITCH2_PIN));
    
    // 如果处于空闲状态，无需进一步处理
    if (system_state == STATE_IDLE && remaining_cycles == 0)
        return;
    
    // 状态机处理
    switch (system_state) {
        case STATE_ROTATING:
			
		 // 旋转状态下确保MTD2停止
            MTD2_SetSpeedPercent(0);
		
            // 根据当前步骤检查位置是否到达
            switch (current_step) {
                case STEP_INIT:
                    // 初始化，确保在0位
                    if (IsAtZeroPosition()) {
                        StopMotor();
                        current_position = POSITION_ZERO;
                        
						
						// 准备进入下一步
                        current_step = STEP_PUMP_PREP;
                        ResetPhotoSwitchHistory(&photo_switch1);
                        ResetPhotoSwitchHistory(&photo_switch2);
						
						
                         // 进入保持状态
//                        system_state = STATE_HOLDING;
//                        hold_start_time = HAL_GetTick();
//                        position_reported = 0;
                        
                        // 开始逆时针旋转到抽水预备位
                        StartMotor(-1); // 逆时针
						
						// 设置MTD2电机状态
                        SetMotor2State(STEP_INIT);
						
						
                    } else {
                        // 不在0位，启动回零过程
                        if (!motor_running) {
                            StartMotor(+1); // 顺时针寻找
                        }
                    }
                    break;
                    
                case STEP_PUMP_PREP:
                    // 检查是否到达抽水预备位(-50.5°)
                    // 光1模式: 1→0→1 (最后的1是瞬时值)
                    if (MatchPhotoSwitchPattern(&photo_switch1, "01")) {
                        StartBraking();
                        current_position = POSITION_PUMP_PREP;
                        
                        // 进入保持状态
                        system_state = STATE_HOLDING;
                        hold_start_time = HAL_GetTick();
                        position_reported = 0;
						
						
						 // 设置MTD2电机状态
                        SetMotor2State(STEP_PUMP_PREP);
                    }
                    break;
                    
                case STEP_PUMP_FORMAL:
                    // 检查是否到达抽水正式位(-101°)
                    // 光1模式: 1→0→1 (最后的1是瞬时值)
                    if (MatchPhotoSwitchPattern(&photo_switch1, "01")) {
                        StartBraking();
                        current_position = POSITION_PUMP_FORMAL;
                        
                        // 进入保持状态
                        system_state = STATE_HOLDING;
                        hold_start_time = HAL_GetTick();
                        position_reported = 0;
						
						// 设置MTD2电机状态
                        SetMotor2State(STEP_PUMP_FORMAL);
                    }
                    break;
                    
                case STEP_RETURN_ZERO_1:
                    // 检查是否回到0位
                    if (IsAtZeroPosition()) {
                        StartBraking();
                        current_position = POSITION_ZERO;
                        
                        // 进入保持状态
                        system_state = STATE_HOLDING;
                        hold_start_time = HAL_GetTick();
                        position_reported = 0;
						
						 // 设置MTD2电机状态
                        SetMotor2State(STEP_RETURN_ZERO_1);
                    }
                    break;
                    
                case STEP_DRAIN_PREP:
                    // 检查是否到达排水预备位(125.5°)
                    // 光2模式: 1→0→1→0→1 (最后的1是瞬时值)
                    if (MatchPhotoSwitchPattern(&photo_switch2, "0101")) {
                        StartBraking();
                        current_position = POSITION_DRAIN_PREP;
                        
                        // 进入保持状态
                        system_state = STATE_HOLDING;
                        hold_start_time = HAL_GetTick();
                        position_reported = 0;
						
						 // 设置MTD2电机状态
                        SetMotor2State(STEP_DRAIN_PREP);
                    }
                    break;
                    
                case STEP_DRAIN_FORMAL:
                    // 检查是否到达排水正式位(79°)
                    // 光2模式: 1→0→1→0 (最后的0是瞬时值)
                    if (MatchPhotoSwitchPattern(&photo_switch2, "010")) {
                        StartBraking();
                        current_position = POSITION_DRAIN_FORMAL;
                        
                        // 进入保持状态
                        system_state = STATE_HOLDING;
                        hold_start_time = HAL_GetTick();
                        position_reported = 0;
						
						 // 设置MTD2电机状态
                        SetMotor2State(STEP_DRAIN_FORMAL);
                    }
                    break;
                    
                case STEP_RETURN_ZERO_2:
                    // 检查是否回到0位
                    if (IsAtZeroPosition()) {
                        StartBraking();
                        current_position = POSITION_ZERO;
                        
                        // 进入保持状态
                        system_state = STATE_HOLDING;
                        hold_start_time = HAL_GetTick();
                        position_reported = 0;
						
						// 设置MTD2电机状态
                        SetMotor2State(STEP_RETURN_ZERO_2);
                    }
                    break;
                    
                case STEP_COMPLETE:
                    // 流程完成
                    StopMotor();
				
					// 停止MTD2电机
                    MTD2_SetSpeedPercent(0);
                    
                    // 如果还有剩余循环次数，重新开始
                    if (remaining_cycles > 0) {
                        remaining_cycles--;
                        if (remaining_cycles > 0) {
                            // 重新开始新的循环
							LOG_INFO("-Remain_Cycles: %d\r\n", remaining_cycles);
                            current_step = STEP_INIT;
                            system_state = STATE_ROTATING;
                        } else {
                            // 所有循环完成
                            system_state = STATE_IDLE;
							LOG_INFO("---All_Cycles Complete----");
                        }
                    } else {
                        system_state = STATE_IDLE;
                    }
                    break;
                    
                default:
                    // 未知步骤，重置
                    StopMotor();
				
				// 停止MTD2电机
                    MTD2_SetSpeedPercent(0);
				
                    system_state = STATE_IDLE;
                    break;
            }
            break;
            
        case STATE_HOLDING:
            // 在当前位置保持一段时间
            if (!position_reported) {
                // 发送位置到达通知
                RotationControl_ReportPosition(current_position);
                position_reported = 1;
            }
            
            // 检查是否保持足够时间
            if (HAL_GetTick() - hold_start_time >= step_hold_times[current_step]) {
				
				// 停止MTD2电机
                    MTD2_SetSpeedPercent(0);
				
                // 保持时间结束，进入下一步骤
                switch (current_step) {
					
					case STEP_INIT:
                        // 初始位置停留结束，开始逆时针旋转到抽水预备位
                        current_step = STEP_PUMP_PREP;
                        ResetPhotoSwitchHistory(&photo_switch1);
                        ResetPhotoSwitchHistory(&photo_switch2);
                        StartMotor(-1); // 逆时针
                        system_state = STATE_ROTATING;
                        break;
					
                    case STEP_PUMP_PREP:
                        // 继续逆时针旋转到抽水正式位
                        current_step = STEP_PUMP_FORMAL;
                        ResetPhotoSwitchHistory(&photo_switch1);
                        ResetPhotoSwitchHistory(&photo_switch2);
                        StartMotor(-1); // 逆时针
                        system_state = STATE_ROTATING;
                        break;
                        
                    case STEP_PUMP_FORMAL:
                        // 顺时针回到0位
                        current_step = STEP_RETURN_ZERO_1;
                        StartMotor(+1); // 顺时针
                        system_state = STATE_ROTATING;
                        break;
                        
                    case STEP_RETURN_ZERO_1:
                        // 顺时针旋转到排水预备位
                        current_step = STEP_DRAIN_PREP;
                        ResetPhotoSwitchHistory(&photo_switch1);
                        ResetPhotoSwitchHistory(&photo_switch2);
                        StartMotor(+1); // 顺时针
                        system_state = STATE_ROTATING;
                        break;
                        
                    case STEP_DRAIN_PREP:
                        // 逆时针旋转到排水正式位
                        current_step = STEP_DRAIN_FORMAL;
                        ResetPhotoSwitchHistory(&photo_switch1);
                        ResetPhotoSwitchHistory(&photo_switch2);
                        StartMotor(-1); // 逆时针
                        system_state = STATE_ROTATING;
                        break;
                        
                    case STEP_DRAIN_FORMAL:
                        // 逆时针回到0位
                        current_step = STEP_RETURN_ZERO_2;
                        StartMotor(-1); // 逆时针
                        system_state = STATE_ROTATING;
                        break;
                        
                    case STEP_RETURN_ZERO_2:
                        // 完成一个循环
                        current_step = STEP_COMPLETE;
                        system_state = STATE_ROTATING;
                        break;
                        
                    default:
                        // 未知状态，重置
                        system_state = STATE_IDLE;
                        break;
                }
            }
            break;
            
        case STATE_ERROR:
            // 错误状态处理
            StopMotor();
		    // 停止MTD2电机
            MTD2_SetSpeedPercent(0);
            // TODO: 实现错误恢复逻辑
            break;
            
        default:
            // 未知状态，重置
            StopMotor();
			MTD2_SetSpeedPercent(0);
            system_state = STATE_IDLE;
            break;
    }
}

// 开始旋转流程
void RotationControl_Start(uint16_t cycles)
{
    // 停止当前操作
    StopMotor();
    MTD2_SetSpeedPercent(0);
	
    // 设置循环次数
    remaining_cycles = cycles > 0 ? cycles : 1;
    
    // 初始化状态
    current_step = STEP_INIT;
    system_state = STATE_ROTATING;
    
    // 重置光电开关历史
    ResetPhotoSwitchHistory(&photo_switch1);
    ResetPhotoSwitchHistory(&photo_switch2);
}

// 停止旋转流程
void RotationControl_Stop(void)
{
    StopMotor();
	MTD2_SetSpeedPercent(0); // 确保MTD2也停止
    system_state = STATE_IDLE;
    remaining_cycles = 0;
}

// 设置特定步骤的停留时间(毫秒)
void RotationControl_SetStepHoldTime(FlowStep_t step, uint32_t hold_time_ms)
{
    if (step < STEP_MAX_COUNT) {
		
        step_hold_times[step] = hold_time_ms;
    }
}


// 设置电机运行速度百分比
void RotationControl_SetMotorSpeed(uint8_t speed_percent)
{
    // 限制在0-100范围内
    if (speed_percent > 100)
        speed_percent = 100;
    
    motor_speed_percent = speed_percent;
    
    // 如果电机正在运行，立即更新速度
    if (motor_running) {
        if (motor_direction > 0) {
            MTD4_SetSpeedPercent(motor_speed_percent);
        } else {
            MTD4_SetSpeedPercent(-motor_speed_percent);
        }
    }
}

// 设置第二个电机运行速度百分比
void RotationControl_SetMotor2Speed(uint8_t speed_percent)
{
    // 限制在0-100范围内
    if (speed_percent > 100)
        speed_percent = 100;
    
    motor2_speed_percent = speed_percent;
}



// 检查流程是否完成
uint8_t RotationControl_IsComplete(void)
{
    return (system_state == STATE_IDLE && remaining_cycles == 0);
}

// 报告位置到达(可通过UART或其他方式通知外部系统)
void RotationControl_ReportPosition(SystemPosition_t position)
{
    // 这里实现位置报告功能，例如通过串口发送
    const char* position_names[] = {
        "station: 0",
    "station:  (-50.5)",
    "station:  (-101)",
    "station:  (125.5)",
    "station:  (79)",
    "station: unknown"
    };
    
    // 通过UART发送位置信息
    char buffer[100];
    sprintf(buffer, "Station Information: %s\r\n", 
            position < POSITION_UNKNOWN ? position_names[position] : position_names[5]);
    
    // 使用HAL库函数发送
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 100);
}
