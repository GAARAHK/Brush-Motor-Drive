/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "drv8870.h"  
#include "currentsense.h"
#include "log.h"
#include "at_command.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "rotation_control.h"  
#include "motor_control.h" 
#include "speed_sensor.h" 
#include "cmd_motor.h" 

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

extern void  SpeedSensors_Init(void);

extern void Init_SpeedControl();

// 全局速度传感器实例
extern SpeedSensor_t g_speed_sensors[4];

extern  uint16_t g_device_id;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

  
   Log_Init(&huart1);
	
	 // ��ʼ��AT�������
   AT_Init(&huart1);  //
    //  UART1
   // 初始化电机命令模块 (从Flash读取设备ID)
   MotorCmd_Init();
   //LOG_INFO("\r\nF334 + DRV8870, PWM=20kHz, ADC2 DMA (TIM1 TRGO), IN13=PB12\r\n");

  // ��ʼ����·�������
   MTD1_Init(&htim2);
   MTD2_Init(&htim2);
   MTD3_Init(&htim1);
   MTD4_Init(&htim1);

   // ���ó�ʼ�ٶȣ�20%/30%/40%/50%
//   MTD1_SetSpeedPercent(+90);
//   MTD2_SetSpeedPercent(-90);
//   MTD3_SetSpeedPercent(+90);
//   MTD4_SetSpeedPercent(-90);

  //电流采样ADC2+DMA��TIM1 
   CurrentSense_SetCalibration((CurrCalib_t){3.3f, 0.01f, 50.0f});
   CurrentSense_InitDMA(&hadc2);

   // // 截止频率20Hz，采样20kHz
   CurrentSense_SetLPF_ByCutoff(20.0f, 20000.0f);
   CurrentSense_EnableLPF(1);
   
   // 初始化换气阀旋转控制模块
    RotationControl_Init();

    // 设置换气阀双电机速度(使用100%速度)
    RotationControl_SetMotorSpeed(100);
    RotationControl_SetMotor2Speed(100); // MTD2 100%速度
    
   // 启动旋转流程，执行42700个循环
   //RotationControl_Start(42700);
   
   // 初始化多模式电机控制
   // MotorControl_Init();
    
    // 设置电机2为扭矩模式
    //MotorControl_SetMode(1, CONTROL_MODE_SPEED);
	//MotorControl_SetDuty(1, -50.0f);    // 50%占空比
    //MotorControl_SetCurrent(1, 0.12f); // 设置0.1A电流
	 //MotorControl_SetSpeed(1, 1500.0f); // 设置1000RPM速度
    //MotorControl_Enable(1, 1);        // 使能电机2
    
	// 设置电机3为扭矩模式
    //MotorControl_SetMode(2, CONTROL_MODE_TORQUE);
	//MotorControl_SetDuty(2, -50.0f);    // 50%占空比
    //MotorControl_SetCurrent(2, 0.1f); // 设置0.1A电流
    //MotorControl_Enable(2, 1);        // 使能电机4
	Init_SpeedControl();
	
    // 启动换气阀定时器中断
    HAL_TIM_Base_Start_IT(&htim3);
    // 启动电流环定时器
    //HAL_TIM_Base_Start_IT(&htim6);
	
	// 配置软启动参数（可选）
    SoftStart_Config_t soft_config = {
        .ramp_time_ms = 800,  // 800ms软启动时间
        .step_count = 25,     // 25个步骤
        .enabled = 1          // 启用软启动
    };
    
    // 为所有电机设置软启动配置
    for (uint8_t i = 0; i < 4; i++) {
        MTD_SetSoftStartConfig(i, soft_config);
    }
    
    LOG_INFO("Soft start initialized for all motors (800ms ramp time)\r\n");
	
	// 初始化四个电机的直接换相保护：检测到正反直接换相时，先滑行再软起动
// 这里设置4个电机统一滑行2秒，你也可以按电机分别设置不同时间
	for (uint8_t i = 0; i < 4; i++) {
		MTD_SetDirectionChangeDelay(i, 2000);  // 2s 滑行
	}
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  
      AT_MainLoopHandler();
	   // 处理软启动更新（需要定期调用）
      MTD_ProcessSoftStart();
	   // 每500ms输出一次电机状态
//        static uint32_t last_print = 0;
//        if (HAL_GetTick() - last_print > 1000) {
//            last_print = HAL_GetTick();
//            
//            // 获取并打印电机1状态
//            MotorControl_t* m2 = MotorControl_GetStatus(1);
//            LOG_INFO("M2: I=%.3fA, Speed=%.1fRPM, Mode=%d\r\n", 
//                     m2->actual_current, m2->actual_speed, m2->mode);
//			
//			// 获取并打印电机4状态
//            MotorControl_t* m3 = MotorControl_GetStatus(2);
//            LOG_INFO("M3: I=%.3fA, Speed=%.1fRPM, Mode=%d\r\n", 
//                     m3->actual_current, m3->actual_speed, m3->mode);
//			
//			LOG_INFO("DeviceId=%d\r\n", 
//                     g_device_id);
//        }
		
		// 在主循环中添加
//			static uint32_t last_check = 0;
//			if (HAL_GetTick() - last_check > 500) {
//				last_check = HAL_GetTick();
//				// 读取霍尔传感器引脚状态
//				GPIO_PinState hall_state = HAL_GPIO_ReadPin(GPIOA, HALL_01_Pin);
//				LOG_INFO("Hall sensor pin state: %d", hall_state);
//			}
		// 每5ms运行一次速度环计算(200Hz)
//        static uint32_t last_speed_update = 0;
//        if (HAL_GetTick() - last_speed_update >= 5) {
//            last_speed_update = HAL_GetTick();
//            MotorControl_SpeedLoopUpdate();
//        }
		
	  HAL_Delay(1); // 防止CPU占用过高

  }
	  
	  
  
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// 定时器中断回调
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) { // 假设使用TIM3作为10ms周期定时器
		

        // 调用旋转控制处理函数
        RotationControl_Process();
    }
	else if (htim->Instance == TIM6) { // 5kHz电流环控制
		
//		 // 速度环更新(200Hz，每5ms执行一次)
        static uint8_t speed_div = 0;
        if (++speed_div >= 25) {
            speed_div = 0;
            MotorControl_SpeedLoopUpdate();
			
        }
//		
		
        MotorControl_CurrentLoopUpdate();
		
		
    }
}


// ADC错误回调
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC2)
  {
    LOG_ERROR("ADC Error: 0x%lX\r\n", hadc->ErrorCode);
    
    if (hadc->ErrorCode & HAL_ADC_ERROR_DMA)
    {
      LOG_ERROR("DMA Error: 0x%lX\r\n", hadc->DMA_Handle->ErrorCode);
    }
    
  
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
