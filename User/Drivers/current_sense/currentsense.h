#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include "stm32f3xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

	// ===== 引脚（IN13 在 PB12）=====
#define LED_PC13_GPIO_Port        GPIOC
#define LED_PC13_Pin         GPIO_PIN_13
	
	
// ===== ADC2 通道映射（按你的网名）=====
#define CUR1_ADC2_CHANNEL     ADC_CHANNEL_1   // MTD1_CUR_ADC2_IN1
#define CUR2_ADC2_CHANNEL     ADC_CHANNEL_2   // MTD2_CUR_ADC2_IN2
#define CUR3_ADC2_CHANNEL     ADC_CHANNEL_3   // MTD3_CUR_ADC2_IN3
#define CUR4_ADC2_CHANNEL     ADC_CHANNEL_13  // MTD4_CUR_ADC2_IN13

// ===== 引脚（IN13 在 PB12）=====
#define CUR1_GPIO_Port        GPIOA
#define CUR1_GPIO_Pin         GPIO_PIN_4
#define CUR2_GPIO_Port        GPIOA
#define CUR2_GPIO_Pin         GPIO_PIN_5
#define CUR3_GPIO_Port        GPIOA
#define CUR3_GPIO_Pin         GPIO_PIN_6
#define CUR4_GPIO_Port        GPIOB
#define CUR4_GPIO_Pin         GPIO_PIN_12    // 指定 PB12

// 采样时间（可按前端 RC 调整）
#define CUR_ADC_SAMPLETIME    ADC_SAMPLETIME_601CYCLES_5

// ===== DMA 映射（若与你芯片不符，请改成实际通道/中断）=====
#define CUR_ADC_DMA_IRQn            DMA1_Channel2_IRQn

typedef struct {
    float vref;     // ADC 参考电压（V）
    float rsense;   // 分流电阻（Ω）
    float gain;     // INA180 增益（A2=50）
} CurrCalib_t;

void CurrentSense_InitDMA(ADC_HandleTypeDef* hadc);
void CurrentSense_SetCalibration(CurrCalib_t c);

// === 一阶低通（EMA/RC）配置 ===
// 使能/关闭滤波；默认关闭
void CurrentSense_EnableLPF(uint8_t enable);
// 直接按 alpha 设置（0<alpha<=1，1 为不滤波）
void CurrentSense_SetLPF_ByAlpha(float alpha);
// 按截止频率设置：alpha = dt/(tau+dt)，tau=1/(2πfc)
void CurrentSense_SetLPF_ByCutoff(float fc_hz, float fs_hz);

//软件触发ADC转换
void CurrentSense_SoftwareTrigger(void);

// 最新采样结果（DMA 循环更新）
extern volatile uint16_t g_curr_raw[4];     // 原始 12bit
extern volatile float    g_curr_amp[4];     // 瞬时电流（A）
extern volatile float    g_curr_amp_filt[4];// 低通后电流（A）

// DMA 中断处理函数
void CUR_ADC_DMA_IRQHandler(void);

// 状态检查函数
void CurrentSense_CheckStatus(void);

// 基本ADC测试函数
void CurrentSense_TestBasicADC(void);

void CurrentSense_Init(void);

void CurrentSense_InitDirect(void);

#ifdef __cplusplus
}
#endif
#endif // CURRENT_SENSE_H