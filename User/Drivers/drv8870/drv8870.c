#include "drv8870.h"

// 保存各电机对应的定时器句柄
static TIM_HandleTypeDef* htim_mtd1;
static TIM_HandleTypeDef* htim_mtd2;
static TIM_HandleTypeDef* htim_mtd3;
static TIM_HandleTypeDef* htim_mtd4;

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

static void pair_set_speed(TIM_HandleTypeDef* htim, uint32_t ch_in1, uint32_t ch_in2, int8_t percent)
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
}
void MTD4_SetSpeedPercent(int8_t percent) { 
    pair_set_speed(htim_mtd4, TIM_CHANNEL_3, TIM_CHANNEL_4, percent); 
}
void MTD4_Coast(void) { 
    pair_set_speed(htim_mtd4, TIM_CHANNEL_3, TIM_CHANNEL_4, 0); 
}