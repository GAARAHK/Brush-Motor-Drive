#include "currentsense.h"
#include <math.h>
#include "log.h"
#include "adc.h"
#include "tim.h"
 
// 添加圆周率定义（如果编译器没有提供）
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


static ADC_HandleTypeDef* current_adc;

volatile uint16_t g_curr_raw[4]     = {0};
volatile float    g_curr_amp[4]     = {0};
volatile float    g_curr_amp_filt[4] = {0};

static CurrCalib_t s_cal = {
    .vref   = 3.3f,
    .rsense = 0.01f,   // 10 mΩ
    .gain   = 50.0f    // INA180A2
};

// 一阶低通配置
static uint8_t s_lpf_enabled = 0;
static float   s_lpf_alpha   = 1.0f; // 1.0 = 直通
static uint8_t s_lpf_inited  = 0;    // 首次对齐标志

void CurrentSense_SetCalibration(CurrCalib_t c)
{
    s_cal = c;
}

void CurrentSense_EnableLPF(uint8_t enable)
{
    s_lpf_enabled = enable ? 1 : 0;
    if (!s_lpf_enabled) {
        s_lpf_alpha  = 1.0f; // 直通
        s_lpf_inited = 0;
    }
}

void CurrentSense_SetLPF_ByAlpha(float alpha)
{
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    s_lpf_alpha = alpha;
    s_lpf_enabled = (alpha < 1.0f) ? 1 : s_lpf_enabled;
    // 不重置 inited，让它延续现有状态
}

void CurrentSense_SetLPF_ByCutoff(float fc_hz, float fs_hz)
{
    if (fc_hz <= 0.0f || fs_hz <= 0.0f) {
        CurrentSense_SetLPF_ByAlpha(1.0f); // 退化为直通
        return;
    }
    const float dt  = 1.0f / fs_hz;
    const float tau = 1.0f / (2.0f * (float)M_PI * fc_hz);
    float alpha = dt / (tau + dt);   // 0..1
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    CurrentSense_SetLPF_ByAlpha(alpha);
}

void TIM1_Init_Manual(void)
{
    // 启用 TIM1 时钟
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    
    // 复位 TIM1
    TIM1->CR1 = 0;
    TIM1->CR2 = 0;
    
    // 设置分频器和周期
    // 假设 APB2 时钟为 72MHz，我们设置 20kHz PWM
    // 分频 = 0 (预分频器 = 1)
    // 周期 = 72MHz/20kHz - 1 = 3599
    TIM1->PSC = 0;
    TIM1->ARR = 3599; // 调整为您需要的频率
    
    // 配置为产生 TRGO 更新事件
    TIM1->CR2 &= ~TIM_CR2_MMS;
    TIM1->CR2 |= (2 << 4); // 设置 TRGO 为 UPDATE 事件 (值 = 2)
    
    // 生成更新事件以应用配置
    TIM1->EGR = TIM_EGR_UG;
    
    // 启动 TIM1
    TIM1->CR1 |= TIM_CR1_CEN;
    
    LOG_INFO("TIM1 manually initialized and started with TRGO=UPDATE\r\n");
    LOG_INFO("TIM1 CR1=0x%08X, CR2=0x%08X, ARR=%d\r\n", 
             (unsigned int)TIM1->CR1, 
             (unsigned int)TIM1->CR2,
             (int)TIM1->ARR);
}

// 直接配置 DMA 和 ADC，避免使用可能有问题的 HAL 函数
void CurrentSense_InitDirect(void)
{
    // 第一步：确保 TIM1 配置和启动正确
    LOG_INFO("Initializing Current Sensing with direct register access\r\n");
    
    // 先确保 TIM1 时钟已使能并配置正确
    if ((RCC->APB2ENR & RCC_APB2ENR_TIM1EN) == 0) {
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    }
    
    // 配置并启动 TIM1
    TIM1->CR1 = 0;
    TIM1->PSC = 0;
    TIM1->ARR = 3599; // 20kHz @ 72MHz
    TIM1->CR2 = (2 << 4); // TRGO = UPDATE
    TIM1->EGR = TIM_EGR_UG; // 生成更新事件
    TIM1->CR1 |= TIM_CR1_CEN; // 启动定时器
    
    LOG_INFO("TIM1 configured: CR1=0x%08X, CR2=0x%08X\r\n", (unsigned int)TIM1->CR1, (unsigned int)TIM1->CR2);
    
    // 第二步：确保 ADC 和 DMA 时钟已使能
    RCC->AHBENR |= RCC_AHBENR_ADC12EN; // 使能 ADC12 时钟
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;  // 使能 DMA1 时钟
    
    // 第三步：复位并配置 ADC
    // 禁用 ADC
    ADC2->CR &= ~ADC_CR_ADEN;
    while(ADC2->CR & ADC_CR_ADEN); // 等待 ADC 禁用完成
    
    // 确保 ADC 电压调节器启用
    ADC2->CR &= ~ADC_CR_ADVREGEN;
    ADC2->CR |= ADC_CR_ADVREGEN_0;
    
    // 等待电压稳定
    HAL_Delay(10);
    
    // 校准 ADC
    ADC2->CR |= ADC_CR_ADCAL;
    while(ADC2->CR & ADC_CR_ADCAL); // 等待校准完成
    
    // 第四步：手动配置 ADC
    ADC2->CFGR = 0; // 清除配置
    ADC2->CFGR |= ADC_CFGR_DMAEN; // 启用 DMA
    
    // 配置外部触发
    ADC2->CFGR |= (3 << ADC_CFGR_EXTSEL_Pos); // TIM1 TRGO 触发源
    ADC2->CFGR |= (1 << ADC_CFGR_EXTEN_Pos);  // 上升沿触发
    
    // 配置转换序列 (4个通道)
    ADC2->SQR1 = (3 << ADC_SQR1_L_Pos); // 4个转换
    // 这里使用默认通道1-4，根据您的需要调整
    ADC2->SQR1 |= (1 << ADC_SQR1_SQ1_Pos); // 通道1
    ADC2->SQR1 |= (2 << ADC_SQR1_SQ2_Pos); // 通道2
    ADC2->SQR2 = (3 << ADC_SQR1_SQ3_Pos);  // 通道3
    ADC2->SQR2 |= (4 << ADC_SQR3_SQ13_Pos); // 通道4
    
    // 配置采样时间 (所有通道)
    ADC2->SMPR1 = 0x00FFFFFF; // 最长采样时间
    ADC2->SMPR2 = 0x00FFFFFF; // 最长采样时间
    
    // 使能 ADC
    ADC2->CR |= ADC_CR_ADEN;
    while(!(ADC2->ISR & ADC_ISR_ADRDY)); // 等待 ADC 就绪
    
    LOG_INFO("ADC2 configured: CFGR=0x%08X, SQR1=0x%08X\r\n", 
             (unsigned int)ADC2->CFGR, (unsigned int)ADC2->SQR1);
    
    // 第五步：手动配置 DMA
    // 假设我们使用 DMA1 Channel2 for ADC2
    DMA1_Channel2->CCR = 0; // 清除配置
    
    // 设置 DMA 通道参数
    DMA1_Channel2->CPAR = (uint32_t)&ADC2->DR;     // 源地址 (ADC 数据寄存器)
    DMA1_Channel2->CMAR = (uint32_t)&g_curr_raw;   // 目标地址 (缓冲区)
    DMA1_Channel2->CNDTR = 4;                      // 4个数据项
    
    // 配置 DMA
    DMA1_Channel2->CCR = DMA_CCR_MINC |   // 内存地址增量
                          DMA_CCR_MSIZE_0 | // 16位内存数据
                          DMA_CCR_PSIZE_0 | // 16位外设数据
                          DMA_CCR_CIRC;     // 循环模式
    
    // 启用 DMA 通道
    DMA1_Channel2->CCR |= DMA_CCR_EN;
    
    LOG_INFO("DMA configured: CCR=0x%08X, CNDTR=%d\r\n", 
             (unsigned int)DMA1_Channel2->CCR, (int)DMA1_Channel2->CNDTR);
    
    // 第六步：启动转换 (首次软件触发)
    LOG_INFO("Starting ADC conversion\r\n");
    ADC2->CR |= ADC_CR_ADSTART;
    
    LOG_INFO("Current sensing initialization complete\r\n");
}


void CurrentSense_Init(void)
{
    // 第一步：确保 TIM1 配置和启动正确
    LOG_INFO("Initializing Current Sensing with TIM1 trigger\r\n");
    
    // 先检查 TIM1 是否已配置
    if ((RCC->APB2ENR & RCC_APB2ENR_TIM1EN) == 0) {
        LOG_INFO("TIM1 clock not enabled - enabling now\r\n");
        RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    }
    
    // 初始化并启动 TIM1 (使用手动方式确保正确配置)
    TIM1_Init_Manual();
    
    // 创建静态 DMA 缓冲区
//     g_curr_raw[4] = {0};
    
    // 第二步：停止任何正在进行的转换
    HAL_ADC_Stop_DMA(&hadc2);
    
    // 第三步：校准 ADC
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
        LOG_ERROR("ADC calibration failed\r\n");
    }
    
    // 第四步：配置 ADC 为 TIM1 TRGO 触发
    ADC_ChannelConfTypeDef sConfig = {0};
    
    // 配置通道
    // 注意：这里示例使用通道 1-4，请替换为您实际使用的通道
    uint32_t channels[4] = {ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3, ADC_CHANNEL_13};
    
    // 配置扫描序列
    hadc2.Instance->SQR1 = 0; // 清除旧配置
    hadc2.Instance->SQR1 |= (3 << ADC_SQR1_L_Pos); // 设置4个转换
    
    // 配置每个通道
    for (int i = 0; i < 4; i++) {
        sConfig.Channel = channels[i];
        sConfig.Rank = i + 1;
        sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
        sConfig.SingleDiff = ADC_SINGLE_ENDED;
        
        if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
            LOG_ERROR("Failed to configure ADC channel %d\r\n", i);
        }
    }
    
    // 第五步：配置 ADC 外部触发
    hadc2.Instance->CFGR &= ~(ADC_CFGR_EXTSEL | ADC_CFGR_EXTEN);
    hadc2.Instance->CFGR |= (ADC_EXTERNALTRIGCONV_T1_TRGO | ADC_EXTERNALTRIGCONVEDGE_RISING);
    
    // 第六步：启动 ADC DMA
    LOG_INFO("Starting ADC with DMA (TIM1 TRGO trigger)\r\n");
    HAL_StatusTypeDef status = HAL_ADC_Start_DMA(&hadc2, (uint32_t*)g_curr_raw, 4);
    
    if (status != HAL_OK) {
        LOG_ERROR("Failed to start ADC DMA, error: %d\r\n", status);
    } else {
        LOG_INFO("ADC DMA started successfully\r\n");
    }
    
    // 验证配置
    LOG_INFO("TIM1 Status: CR1=0x%08X, CR2=0x%08X\r\n", 
             (unsigned int)TIM1->CR1, (unsigned int)TIM1->CR2);
    LOG_INFO("ADC2 Config: CFGR=0x%08X, SQR1=0x%08X\r\n", 
             (unsigned int)ADC2->CFGR, (unsigned int)ADC2->SQR1);
}

void CurrentSense_InitDMA(ADC_HandleTypeDef* hadc)
{
    // 打印当前 ADC 配置信息
//    LOG_INFO("ADC Config Check:\r\n");
//    LOG_INFO("- Resolution: %d bits\r\n", (hadc->Init.Resolution == ADC_RESOLUTION_12B) ? 12 : 
//                                        (hadc->Init.Resolution == ADC_RESOLUTION_10B) ? 10 : 
//                                        (hadc->Init.Resolution == ADC_RESOLUTION_8B) ? 8 : 6);
//    LOG_INFO("- DataAlign: %s\r\n", (hadc->Init.DataAlign == ADC_DATAALIGN_RIGHT) ? "RIGHT" : "LEFT");
//    LOG_INFO("- ScanConvMode: %s\r\n", (hadc->Init.ScanConvMode == ADC_SCAN_ENABLE) ? "ENABLED" : "DISABLED");
//    LOG_INFO("- External Trigger: %d\r\n", hadc->Init.ExternalTrigConv);
//    LOG_INFO("- NbrOfConversion: %d\r\n", hadc->Init.NbrOfConversion);
//    LOG_INFO("- ContinuousConvMode: %s\r\n", (hadc->Init.ContinuousConvMode == ENABLE) ? "ENABLED" : "DISABLED");
    
    current_adc = hadc;
    
	
    
	
	// 检查DMA句柄是否正确链接
    if (hadc->DMA_Handle == NULL) {
        LOG_INFO("ERROR: DMA Handle is NULL!\r\n");
        return;
    }
    LOG_INFO("DMA Handle OK, Instance: 0x%08X\r\n", (uint32_t)hadc->DMA_Handle->Instance);
    
	
	
    // 重新校准
    HAL_StatusTypeDef cal_status = HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED);
    LOG_INFO("ADC Calibration status: %d\r\n", cal_status);
	
    
    
    // 临时添加：确保数组不是全零
//    g_curr_raw[0] = 123;
//    g_curr_raw[1] = 456;
//    g_curr_raw[2] = 789;
//    g_curr_raw[3] = 1000;
//    LOG_INFO("Initialized g_curr_raw: %u %u %u %u\r\n", 
//             g_curr_raw[0], g_curr_raw[1], g_curr_raw[2], g_curr_raw[3]);
    
    // 启动 ADC2 DMA 循环传输 - 使用外部触发模式
    HAL_StatusTypeDef status = HAL_ADC_Start_DMA(hadc, (uint32_t*)g_curr_raw, 4);
    if (status != HAL_OK) {
        LOG_INFO("ERROR: Failed to start ADC DMA, status: %d\r\n", status);
        LOG_INFO("ADC ErrorCode: %lu\r\n", hadc->ErrorCode);
        if (hadc->DMA_Handle != NULL) {
            LOG_INFO("DMA ErrorCode: %lu\r\n", hadc->DMA_Handle->ErrorCode);
        }
    } else {
        LOG_INFO("SUCCESS: ADC DMA started\r\n");
    }
//	

}

	
	
			 
			 


// DMA 中断处理函数 - 需在 stm32f3xx_it.c 中调用
void CUR_ADC_DMA_IRQHandler(void)
{

    HAL_DMA_IRQHandler(current_adc->DMA_Handle);
}

// ==== DMA 全序列完成回调：换算 + 一阶低通 ====
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc != current_adc) return;
	
//	// 打印原始值以调试
//    static uint32_t debug_count = 0;
//    if (debug_count++ % 1000 == 0) { // 每1000次打印一次
//        LOG_DEBUG("RAW ADC: %u %u %u %u\r\n", 
//                 g_curr_raw[0], g_curr_raw[1], g_curr_raw[2], g_curr_raw[3]);
//    }
	// 使用简化的滑动平均代替单点采样
    #define AVG_COUNT 4
    static uint16_t buffer[4][AVG_COUNT] = {0};
    static uint8_t idx = 0;
    
    // 保存当前值到缓冲区
    buffer[0][idx] = g_curr_raw[0];
    buffer[1][idx] = g_curr_raw[1];
    buffer[2][idx] = g_curr_raw[2];
    buffer[3][idx] = g_curr_raw[3];
    idx = (idx + 1) % AVG_COUNT;
    
    // 计算平均值
    uint32_t sum[4] = {0};
    for (int i = 0; i < AVG_COUNT; i++) {
        sum[0] += buffer[0][i];
        sum[1] += buffer[1][i];
        sum[2] += buffer[2][i];
        sum[3] += buffer[3][i];
    }
	

     // 使用平均值进行转换
    const float k = s_cal.vref / (4095.0f * s_cal.gain * s_cal.rsense);
    float x0 = (sum[0] / AVG_COUNT) * k;
    float x1 = (sum[1] / AVG_COUNT) * k;
    float x2 = (sum[2] / AVG_COUNT) * k;
    float x3 = (sum[3] / AVG_COUNT) * k;

    g_curr_amp[0] = x0;
    g_curr_amp[1] = x1;
    g_curr_amp[2] = x2;
    g_curr_amp[3] = x3;

    if (!s_lpf_enabled || s_lpf_alpha >= 1.0f) {
        // 直通
        g_curr_amp_filt[0] = x0;
        g_curr_amp_filt[1] = x1;
        g_curr_amp_filt[2] = x2;
        g_curr_amp_filt[3] = x3;
        s_lpf_inited = 1;
        return;
    }

    if (!s_lpf_inited) {
        // 初次对齐，避免大阶跃
        g_curr_amp_filt[0] = x0;
        g_curr_amp_filt[1] = x1;
        g_curr_amp_filt[2] = x2;
        g_curr_amp_filt[3] = x3;
        s_lpf_inited = 1;
        return;
    }

    // EMA：y += alpha * (x - y)
    const float a = s_lpf_alpha;
    g_curr_amp_filt[0] += a * (x0 - g_curr_amp_filt[0]);
    g_curr_amp_filt[1] += a * (x1 - g_curr_amp_filt[1]);
    g_curr_amp_filt[2] += a * (x2 - g_curr_amp_filt[2]);
    g_curr_amp_filt[3] += a * (x3 - g_curr_amp_filt[3]);
}

//// 在 currentsense.c 中添加软件触发，在主循环中触发
void CurrentSense_SoftwareTrigger(void)
{
    if (current_adc != NULL) {
        HAL_ADC_Start(current_adc); // 软件触发转换
    }
}


// 添加状态检查函数
void CurrentSense_CheckStatus(void)
{
    if (current_adc == NULL) {
        LOG_INFO("ERROR: ADC not initialized\r\n");
        return;
    }
    
    LOG_INFO("ADC Status Check:\r\n");
    LOG_INFO("- ADC State: %d\r\n", current_adc->State);
    LOG_INFO("- ADC Error: %d\r\n", current_adc->ErrorCode);
    
    if (current_adc->DMA_Handle != NULL) {
        LOG_INFO("- DMA State: %d\r\n", current_adc->DMA_Handle->State);
        LOG_INFO("- DMA Error: %d\r\n", current_adc->DMA_Handle->ErrorCode);
    } else {
        LOG_INFO("- DMA Handle: NULL\r\n");
    }
}

// 添加重启DMA的函数
void CurrentSense_RestartDMA(void)
{
    if (current_adc == NULL) {
        LOG_INFO("ERROR: ADC not initialized\r\n");
        return;
    }
    
    LOG_INFO("Attempting to restart ADC DMA...\r\n");
    
    // 停止当前DMA
    HAL_ADC_Stop_DMA(current_adc);
    HAL_Delay(10);
    
    // 清除错误
    current_adc->ErrorCode = HAL_ADC_ERROR_NONE;
    if (current_adc->DMA_Handle != NULL) {
        current_adc->DMA_Handle->ErrorCode = HAL_DMA_ERROR_NONE;
    }
    
    // 重新校准
    HAL_ADCEx_Calibration_Start(current_adc, ADC_SINGLE_ENDED);
    
    // 重新启动
    HAL_StatusTypeDef status = HAL_ADC_Start_DMA(current_adc, (uint32_t*)g_curr_raw, 4);
    if (status == HAL_OK) {
        LOG_INFO("ADC DMA restarted successfully\r\n");
    } else {
        LOG_INFO("Failed to restart ADC DMA, status: %d\r\n", status);
    }
}


// 添加简单的ADC测试函数
void CurrentSense_TestBasicADC(void)
{
    if (current_adc == NULL) {
        LOG_INFO("ERROR: ADC not initialized for test\r\n");
        return;
    }
    
    LOG_INFO("Testing basic ADC without DMA...\r\n");
    
    // 停止任何正在进行的DMA
    HAL_ADC_Stop_DMA(current_adc);
    
    // 尝试基本的单次转换
    HAL_StatusTypeDef status = HAL_ADC_Start(current_adc);
    if (status != HAL_OK) {
        LOG_INFO("ERROR: Failed to start basic ADC, status: %d\r\n", status);
        return;
    }
    
    // 等待转换完成
    status = HAL_ADC_PollForConversion(current_adc, 100);
    if (status == HAL_OK) {
        uint32_t value = HAL_ADC_GetValue(current_adc);
        LOG_INFO("SUCCESS: Basic ADC read value: %lu\r\n", value);
    } else {
        LOG_INFO("ERROR: ADC poll timeout, status: %d\r\n", status);
    }
    
    HAL_ADC_Stop(current_adc);
}

//void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc)
//{
//     LOG_INFO("ADC Error Callback: ErrorCode = %lu\r\n", hadc->ErrorCode);
//    
//    // 解析错误代码
//    if (hadc->ErrorCode & HAL_ADC_ERROR_INTERNAL) {
//        LOG_INFO("- HAL_ADC_ERROR_INTERNAL\r\n");
//    }
//    if (hadc->ErrorCode & HAL_ADC_ERROR_OVR) {
//        LOG_INFO("- HAL_ADC_ERROR_OVR (Overrun)\r\n");
//    }
//    if (hadc->ErrorCode & HAL_ADC_ERROR_DMA) {
//        LOG_INFO("- HAL_ADC_ERROR_DMA\r\n");
//        if (hadc->DMA_Handle != NULL) {
//            LOG_INFO("- DMA Error Code: %lu\r\n", hadc->DMA_Handle->ErrorCode);
//        }
//    }
//    if (hadc->ErrorCode & HAL_ADC_ERROR_JQOVF) {
//        LOG_INFO("- HAL_ADC_ERROR_JQOVF\r\n");
//    }
//	
//	if (hadc->ErrorCode & HAL_ADC_ERROR_OVR) {
//        // 清除溢出标志
//        __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_OVR);
//    }
//    
//    // 重启ADC DMA,可能会导致无限重启
////    HAL_ADC_Stop_DMA(hadc);
////    HAL_Delay(1);
////    HAL_ADC_Start_DMA(hadc, (uint32_t*)g_curr_raw, 4);
//}

//// 添加DMA错误回调
void HAL_ADC_DMAErrorCallback(ADC_HandleTypeDef* hadc)
{
    LOG_INFO("DMA Error Callback: ADC ErrorCode = %lu\r\n", hadc->ErrorCode);
    if (hadc->DMA_Handle != NULL) {
        LOG_INFO("DMA ErrorCode = %lu\r\n", hadc->DMA_Handle->ErrorCode);
    }
}

