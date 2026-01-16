#include "drv8870.h"

// 保存各电机对应的定时器句柄
static TIM_HandleTypeDef* htim_mtd1;
static TIM_HandleTypeDef* htim_mtd2;
static TIM_HandleTypeDef* htim_mtd3;
static TIM_HandleTypeDef* htim_mtd4;

// 软启动状态数组，对应4个电机
static SoftStart_State_t soft_start_states[4];

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

static inline int8_t get_direction(int8_t v) {
    return (v > 0) ? 1 : ((v < 0) ? -1 : 0);
}

// 内部函数：直接设置电机PWM（不经过软启动）
static void pair_set_speed_direct(TIM_HandleTypeDef* htim, uint32_t ch_in1, uint32_t ch_in2, int8_t percent)
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

// 修改后的pair_set_speed函数，支持软启动 + 直换相滑行保护
static void pair_set_speed(TIM_HandleTypeDef* htim, uint32_t ch_in1, uint32_t ch_in2, int8_t percent, uint8_t motor_idx)
{
    if (motor_idx >= 4) return;

    SoftStart_State_t* state = &soft_start_states[motor_idx];
    int8_t new_dir = get_direction(percent);
    int8_t old_dir = state->last_direction;

    // 如处于滑行阶段，更新目标后等待滑行结束
    if (state->coasting) {
        state->target_speed = percent;
        return;
    }

    // 检测“直接换相”：当前方向与目标方向均非0且相反，且设置了滑行时间
    if (state->coast_duration_ms > 0 &&
        old_dir != 0 && new_dir != 0 && new_dir != old_dir)
    {
        // 进入滑行：输出0，占空比拉低
        pair_set_speed_direct(htim, ch_in1, ch_in2, 0);
        state->coasting = 1;
        state->coast_start_time = HAL_GetTick();
        state->is_ramping = 0;
        state->current_speed = 0;
        state->last_direction = 0;
        state->target_speed = percent; // 滑行期间记住目标
        return;
    }

    // 以下是原有的软启动逻辑（无需滑行保护的情况）
    state->target_speed = percent;

    // 检查是否需要启动软启动过程
    if (state->config.enabled &&
        (state->current_speed != percent) &&
        (percent != 0 || state->current_speed != 0))
    {
        // 计算步进间隔
        if (state->config.step_count > 0) {
            state->step_interval_ms = state->config.ramp_time_ms / state->config.step_count;
            if (state->step_interval_ms == 0) state->step_interval_ms = 1; // 最小1ms
        }

        // 启动软启动过程
        state->is_ramping = 1;
        state->last_update_time = HAL_GetTick();

        // 如果目标是停止，可以直接停止而不用软启动
        if (percent == 0) {
            state->current_speed = 0;
            state->is_ramping = 0;
            state->last_direction = 0;
            pair_set_speed_direct(htim, ch_in1, ch_in2, 0);
            return;
        }

        // 如果当前是停止状态，从最小非零值开始
        if (state->current_speed == 0) {
            state->current_speed = (percent > 0) ? 1 : -1;
        }
    } else {
        // 不需要软启动，直接设置
        state->current_speed = percent;
        state->is_ramping = 0;
        state->last_direction = new_dir;
        pair_set_speed_direct(htim, ch_in1, ch_in2, percent);
    }
}

// 初始化软启动状态
static void init_soft_start_state(uint8_t motor_idx) {
    if (motor_idx >= 4) return;

    SoftStart_State_t* state = &soft_start_states[motor_idx];
    SoftStart_Config_t default_config = DEFAULT_SOFT_START_CONFIG;

    state->target_speed = 0;
    state->current_speed = 0;
    state->last_update_time = 0;
    state->step_interval_ms = 0;
    state->is_ramping = 0;
    state->config = default_config;

    // 新增直换相保护初始化
    state->last_direction = 0;
    state->coasting = 0;
    state->coast_start_time = 0;
    state->coast_duration_ms = 0; // 默认0=不启用保护
}

static void pair_start(TIM_HandleTypeDef* htim, uint32_t ch_in1, uint32_t ch_in2, uint8_t motor_idx)
{
    HAL_TIM_PWM_Start(htim, ch_in1);
    HAL_TIM_PWM_Start(htim, ch_in2);

    // 初始化软启动状态
    init_soft_start_state(motor_idx);

    pair_set_speed_direct(htim, ch_in1, ch_in2, 0);
}

// ===== MTD1: TIM2 CH1/CH2 =====
void MTD1_Init(TIM_HandleTypeDef* htim) { 
    htim_mtd1 = htim;
    pair_start(htim, TIM_CHANNEL_1, TIM_CHANNEL_2, 0); 
}
void MTD1_SetSpeedPercent(int8_t percent) { 
    pair_set_speed(htim_mtd1, TIM_CHANNEL_1, TIM_CHANNEL_2, percent, 0); 
}
void MTD1_Coast(void) { 
    soft_start_states[0].target_speed = 0;
    soft_start_states[0].current_speed = 0;
    soft_start_states[0].is_ramping = 0;
    soft_start_states[0].last_direction = 0;
    soft_start_states[0].coasting = 0;
    pair_set_speed_direct(htim_mtd1, TIM_CHANNEL_1, TIM_CHANNEL_2, 0); 
}

// ===== MTD2: TIM2 CH3/CH4 =====
void MTD2_Init(TIM_HandleTypeDef* htim) { 
    htim_mtd2 = htim;
    pair_start(htim, TIM_CHANNEL_3, TIM_CHANNEL_4, 1); 
}
void MTD2_SetSpeedPercent(int8_t percent) { 
    pair_set_speed(htim_mtd2, TIM_CHANNEL_3, TIM_CHANNEL_4, percent, 1); 
}
void MTD2_Coast(void) { 
    soft_start_states[1].target_speed = 0;
    soft_start_states[1].current_speed = 0;
    soft_start_states[1].is_ramping = 0;
    soft_start_states[1].last_direction = 0;
    soft_start_states[1].coasting = 0;
    pair_set_speed_direct(htim_mtd2, TIM_CHANNEL_3, TIM_CHANNEL_4, 0); 
}

// ===== MTD3: TIM1 CH1/CH2 =====
void MTD3_Init(TIM_HandleTypeDef* htim) { 
    htim_mtd3 = htim;
    pair_start(htim, TIM_CHANNEL_1, TIM_CHANNEL_2, 2); 
}
void MTD3_SetSpeedPercent(int8_t percent) { 
    pair_set_speed(htim_mtd3, TIM_CHANNEL_1, TIM_CHANNEL_2, percent, 2); 
}
void MTD3_Coast(void) { 
    soft_start_states[2].target_speed = 0;
    soft_start_states[2].current_speed = 0;
    soft_start_states[2].is_ramping = 0;
    soft_start_states[2].last_direction = 0;
    soft_start_states[2].coasting = 0;
    pair_set_speed_direct(htim_mtd3, TIM_CHANNEL_1, TIM_CHANNEL_2, 0); 
}

// ===== MTD4: TIM1 CH3/CH4 =====
void MTD4_Init(TIM_HandleTypeDef* htim) { 
    htim_mtd4 = htim;
    pair_start(htim, TIM_CHANNEL_3, TIM_CHANNEL_4, 3); 
}
void MTD4_SetSpeedPercent(int8_t percent) { 
    pair_set_speed(htim_mtd4, TIM_CHANNEL_3, TIM_CHANNEL_4, percent, 3); 
}
void MTD4_Coast(void) { 
    soft_start_states[3].target_speed = 0;
    soft_start_states[3].current_speed = 0;
    soft_start_states[3].is_ramping = 0;
    soft_start_states[3].last_direction = 0;
    soft_start_states[3].coasting = 0;
    pair_set_speed_direct(htim_mtd4, TIM_CHANNEL_3, TIM_CHANNEL_4, 0); 
}

// ===== 软启动配置函数 =====
void MTD_SetSoftStartConfig(uint8_t motor_idx, SoftStart_Config_t config) {
    if (motor_idx >= 4) return;
    soft_start_states[motor_idx].config = config;
}

void MTD_EnableSoftStart(uint8_t motor_idx, uint8_t enable) {
    if (motor_idx >= 4) return;
    soft_start_states[motor_idx].config.enabled = enable ? 1 : 0;
}

void MTD_SetSoftStartTime(uint8_t motor_idx, uint16_t ramp_time_ms) {
    if (motor_idx >= 4) return;
    soft_start_states[motor_idx].config.ramp_time_ms = ramp_time_ms;
}

// 新增：设置直换相滑行时间(ms, 0=禁用)
void MTD_SetDirectionChangeDelay(uint8_t motor_idx, uint32_t delay_ms) {
    if (motor_idx >= 4) return;
    soft_start_states[motor_idx].coast_duration_ms = delay_ms;
}

// 获取当前实际速度
int8_t MTD_GetActualSpeed(uint8_t motor_idx) {
    if (motor_idx >= 4) return 0;
    return soft_start_states[motor_idx].current_speed;
}

// ===== 软启动/换相保护处理函数 =====
void MTD_ProcessSoftStart(void) {
    uint32_t current_time = HAL_GetTick();

    for (uint8_t i = 0; i < 4; i++) {
        SoftStart_State_t* state = &soft_start_states[i];

        // 先准备对应的定时器和通道
        TIM_HandleTypeDef* htim = NULL;
        uint32_t ch_in1 = 0, ch_in2 = 0;
        switch (i) {
            case 0: htim = htim_mtd1; ch_in1 = TIM_CHANNEL_1; ch_in2 = TIM_CHANNEL_2; break;
            case 1: htim = htim_mtd2; ch_in1 = TIM_CHANNEL_3; ch_in2 = TIM_CHANNEL_4; break;
            case 2: htim = htim_mtd3; ch_in1 = TIM_CHANNEL_1; ch_in2 = TIM_CHANNEL_2; break;
            case 3: htim = htim_mtd4; ch_in1 = TIM_CHANNEL_3; ch_in2 = TIM_CHANNEL_4; break;
        }
        if (!htim) continue;

        // 若处于滑行阶段，等滑行时间到后再启动软起动
        if (state->coasting) {
            uint32_t elapsed = current_time - state->coast_start_time;
            if (elapsed >= state->coast_duration_ms) {
                // 结束滑行，准备软起动到目标方向
                state->coasting = 0;

                if (state->target_speed == 0) {
                    // 目标也是停止，保持0
                    state->current_speed = 0;
                    state->is_ramping = 0;
                    state->last_direction = 0;
                    pair_set_speed_direct(htim, ch_in1, ch_in2, 0);
                } else {
                    // 启动软起动
                    if (state->config.step_count > 0) {
                        state->step_interval_ms = state->config.ramp_time_ms / state->config.step_count;
                        if (state->step_interval_ms == 0) state->step_interval_ms = 1;
                    }
                    state->is_ramping = 1;
                    state->last_update_time = current_time;

                    // 从最小非零开始，按目标方向
                    state->current_speed = (state->target_speed > 0) ? 1 : -1;
                    pair_set_speed_direct(htim, ch_in1, ch_in2, state->current_speed);
                    state->last_direction = get_direction(state->current_speed);
                }
            }

            // 滑行中不做其它
            continue;
        }

        // 处理软启动斜坡
        if (state->is_ramping) {
            if (state->current_speed == state->target_speed) {
                state->is_ramping = 0;
                continue;
            }

            // 检查是否到了更新时间
            if (current_time - state->last_update_time >= state->step_interval_ms) {
                state->last_update_time = current_time;

                // 计算下一步的速度值
                int8_t speed_diff = state->target_speed - state->current_speed;
                int8_t step_size = (state->config.step_count > 0) ?
                                   (abs(speed_diff) / state->config.step_count + 1) : 1;

                if (speed_diff > 0) {
                    state->current_speed += step_size;
                    if (state->current_speed > state->target_speed) {
                        state->current_speed = state->target_speed;
                    }
                } else if (speed_diff < 0) {
                    state->current_speed -= step_size;
                    if (state->current_speed < state->target_speed) {
                        state->current_speed = state->target_speed;
                    }
                }

                // 应用新的速度值
                pair_set_speed_direct(htim, ch_in1, ch_in2, state->current_speed);
                state->last_direction = get_direction(state->current_speed);

                // 检查是否完成软启动
                if (state->current_speed == state->target_speed) {
                    state->is_ramping = 0;
                }
            }
        }
    }
}