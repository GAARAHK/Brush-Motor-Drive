#include "drv8870.h"

// 保存各电机对应的定时器句柄
static TIM_HandleTypeDef* htim_mtd1;
static TIM_HandleTypeDef* htim_mtd2;
static TIM_HandleTypeDef* htim_mtd3;
static TIM_HandleTypeDef* htim_mtd4;

#if SOFT_START_ENABLE
// 软启动状态结构
typedef struct {
    int8_t target_percent;      // 目标速度百分比
    int8_t current_percent;     // 当前速度百分比
    uint32_t start_time;        // 启动时间
    uint8_t is_ramping;         // 是否正在软启动
    uint8_t is_load_motor;      // 是否为负载电机 (MTD2/MTD4)
    uint8_t main_motor_idx;     // 主电机索引 (仅负载电机使用)
} SoftStartState_t;

// 各电机的软启动状态
static SoftStartState_t g_soft_start[4] = {0};
#endif

static inline uint32_t tim_period(const TIM_HandleTypeDef* htim) {
    return __HAL_TIM_GET_AUTORELOAD((TIM_HandleTypeDef*)htim);
}

static inline void pwm_write(TIM_HandleTypeDef* htim, uint32_t ch, uint32_t pulse) {
    __HAL_TIM_SET_COMPARE(htim, ch, pulse);
}

static inline uint32_t pct_to_pulse(uint32_t period, uint8_t pct) {
    if (pct >= 100) return period; // 100%
    return (period + 1) * pct / 100;
}

#if SOFT_START_ENABLE
// 立即设置PWM（软启动内部使用）
static void pair_set_speed_immediate(TIM_HandleTypeDef* htim, uint32_t ch_in1, uint32_t ch_in2, int8_t percent)
{
    uint32_t arr = tim_period(htim);
    uint8_t ap = (percent < 0) ? (uint8_t)(-percent) : (uint8_t)percent;

    if (percent > 0) {
        pwm_write(htim, ch_in1, pct_to_pulse(arr, ap));
        pwm_write(htim, ch_in2, 0);
    } else if (percent < 0) {
        pwm_write(htim, ch_in1, 0);
        pwm_write(htim, ch_in2, pct_to_pulse(arr, ap));
    } else {
        pwm_write(htim, ch_in1, 0);
        pwm_write(htim, ch_in2, 0);
    }
}

// 获取电机索引
static uint8_t get_motor_index(TIM_HandleTypeDef* htim, uint32_t ch_in1) {
    if (htim == htim_mtd1 && ch_in1 == TIM_CHANNEL_1) return 0; // MTD1
    if (htim == htim_mtd2 && ch_in1 == TIM_CHANNEL_3) return 1; // MTD2
    if (htim == htim_mtd3 && ch_in1 == TIM_CHANNEL_1) return 2; // MTD3
    if (htim == htim_mtd4 && ch_in1 == TIM_CHANNEL_3) return 3; // MTD4
    return 0; // 默认
}

// 启动软启动
static void start_soft_start(uint8_t motor_idx, int8_t target_percent) {
    SoftStartState_t* state = &g_soft_start[motor_idx];
    
    // 如果目标是0，立即停止
    if (target_percent == 0) {
        state->is_ramping = 0;
        state->current_percent = 0;
        state->target_percent = 0;
        return;
    }
    
    state->target_percent = target_percent;
    state->start_time = HAL_GetTick();
    state->is_ramping = 1;
    
    // 如果是负载电机，需要延迟启动
    if (state->is_load_motor) {
        // 负载电机延迟启动
        state->start_time += LOAD_DELAY_TIME_MS;
    }
    
    state->current_percent = 0;
}
#endif

static void pair_set_speed(TIM_HandleTypeDef* htim, uint32_t ch_in1, uint32_t ch_in2, int8_t percent)
{
#if SOFT_START_ENABLE
    uint8_t motor_idx = get_motor_index(htim, ch_in1);
    start_soft_start(motor_idx, percent);
    
    // 使用当前软启动值设置PWM
    pair_set_speed_immediate(htim, ch_in1, ch_in2, g_soft_start[motor_idx].current_percent);
#else
    // 原始逻辑（软启动禁用时）
    uint32_t arr = tim_period(htim);
    uint8_t ap = (percent < 0) ? (uint8_t)(-percent) : (uint8_t)percent;

    if (percent > 0) {
        pwm_write(htim, ch_in1, pct_to_pulse(arr, ap));
        pwm_write(htim, ch_in2, 0);
    } else if (percent < 0) {
        pwm_write(htim, ch_in1, 0);
        pwm_write(htim, ch_in2, pct_to_pulse(arr, ap));
    } else {
        pwm_write(htim, ch_in1, 0);
        pwm_write(htim, ch_in2, 0);
    }
#endif
}


static void pair_start(TIM_HandleTypeDef* htim, uint32_t ch_in1, uint32_t ch_in2)
{
    HAL_TIM_PWM_Start(htim, ch_in1);
    HAL_TIM_PWM_Start(htim, ch_in2);
    pair_set_speed(htim, ch_in1, ch_in2, 0);
}

// ===== MTD1: TIM2 CH1/CH2 =====
void MTD1_Init(TIM_HandleTypeDef* htim) { 
    htim_mtd1 = htim;
    pair_start(htim, TIM_CHANNEL_1, TIM_CHANNEL_2);
#if SOFT_START_ENABLE
    // MTD1 是主电机
    g_soft_start[0].is_load_motor = 0;
#endif
}
void MTD1_SetSpeedPercent(int8_t percent) { 
    pair_set_speed(htim_mtd1, TIM_CHANNEL_1, TIM_CHANNEL_2, percent); 
}
void MTD1_Coast(void) { 
    pair_set_speed(htim_mtd1, TIM_CHANNEL_1, TIM_CHANNEL_2, 0); 
}

// ===== MTD2: TIM2 CH3/CH4 =====
void MTD2_Init(TIM_HandleTypeDef* htim) { 
    htim_mtd2 = htim;
    pair_start(htim, TIM_CHANNEL_3, TIM_CHANNEL_4);
#if SOFT_START_ENABLE
    // MTD2 是负载电机，对应主电机 MTD1
    g_soft_start[1].is_load_motor = 1;
    g_soft_start[1].main_motor_idx = 0;
#endif
}
void MTD2_SetSpeedPercent(int8_t percent) { 
    pair_set_speed(htim_mtd2, TIM_CHANNEL_3, TIM_CHANNEL_4, percent); 
}
void MTD2_Coast(void) { 
    pair_set_speed(htim_mtd2, TIM_CHANNEL_3, TIM_CHANNEL_4, 0); 
}

// ===== MTD3: TIM1 CH1/CH2 =====
void MTD3_Init(TIM_HandleTypeDef* htim) { 
    htim_mtd3 = htim;
    pair_start(htim, TIM_CHANNEL_1, TIM_CHANNEL_2);
#if SOFT_START_ENABLE
    // MTD3 是主电机
    g_soft_start[2].is_load_motor = 0;
#endif
}
void MTD3_SetSpeedPercent(int8_t percent) { 
    pair_set_speed(htim_mtd3, TIM_CHANNEL_1, TIM_CHANNEL_2, percent); 
}
void MTD3_Coast(void) { 
    pair_set_speed(htim_mtd3, TIM_CHANNEL_1, TIM_CHANNEL_2, 0); 
}

// ===== MTD4: TIM1 CH3/CH4 =====
void MTD4_Init(TIM_HandleTypeDef* htim) { 
    htim_mtd4 = htim;
    pair_start(htim, TIM_CHANNEL_3, TIM_CHANNEL_4);
#if SOFT_START_ENABLE
    // MTD4 是负载电机，对应主电机 MTD3
    g_soft_start[3].is_load_motor = 1;
    g_soft_start[3].main_motor_idx = 2;
#endif
}
void MTD4_SetSpeedPercent(int8_t percent) { 
    pair_set_speed(htim_mtd4, TIM_CHANNEL_3, TIM_CHANNEL_4, percent); 
}
void MTD4_Coast(void) { 
    pair_set_speed(htim_mtd4, TIM_CHANNEL_3, TIM_CHANNEL_4, 0); 
}

#if SOFT_START_ENABLE
// 软启动状态更新函数（在主循环中调用）
void DRV8870_SoftStartUpdate(void) {
    uint32_t current_time = HAL_GetTick();
    
    // 电机句柄和通道映射
    TIM_HandleTypeDef* htims[4] = {htim_mtd1, htim_mtd2, htim_mtd3, htim_mtd4};
    uint32_t ch_in1s[4] = {TIM_CHANNEL_1, TIM_CHANNEL_3, TIM_CHANNEL_1, TIM_CHANNEL_3};
    uint32_t ch_in2s[4] = {TIM_CHANNEL_2, TIM_CHANNEL_4, TIM_CHANNEL_2, TIM_CHANNEL_4};
    
    for (uint8_t i = 0; i < 4; i++) {
        SoftStartState_t* state = &g_soft_start[i];
        
        if (!state->is_ramping) continue;
        
        // 检查是否已过启动时间（用于负载延迟）
        if (current_time < state->start_time) continue;
        
        uint32_t elapsed = current_time - state->start_time;
        
        if (elapsed >= SOFT_START_RAMP_TIME_MS) {
            // 软启动完成
            state->is_ramping = 0;
            state->current_percent = state->target_percent;
        } else {
            // 计算当前软启动值
            int32_t target_abs = (state->target_percent < 0) ? -state->target_percent : state->target_percent;
            int32_t current_abs = (target_abs * elapsed) / SOFT_START_RAMP_TIME_MS;
            
            if (state->target_percent < 0) {
                state->current_percent = -current_abs;
            } else {
                state->current_percent = current_abs;
            }
        }
        
        // 更新PWM输出
        if (htims[i] != NULL) {
            pair_set_speed_immediate(htims[i], ch_in1s[i], ch_in2s[i], state->current_percent);
        }
    }
}
#else
// 空实现（软启动禁用时）
void DRV8870_SoftStartUpdate(void) {
    // 软启动禁用时什么都不做
}
#endif